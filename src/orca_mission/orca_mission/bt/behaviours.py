"""行為樹的葉節點。

兩種：
- TaskAction：包一個執行器（tasks/），透過 TaskRunner 的 start／status／cancel 操作。
  被中斷（逾時、上層改走別條路）時 py_trees 會呼叫 terminate(INVALID)，這裡就取消執行器。
- 任務層節點：WaitReady、Confirm、Zero、Arm、Finish、Seen。歸零、解鎖、切模式只有這些能做。

節點透過 Context 拿資料、下指令；Context 由 node.py 提供（測試時用假的）。
"""
import threading
import time

import py_trees
from py_trees.common import Status as BtStatus

from ..tasks.base import Status as TaskStatus
from ..vehicle import Command


class Context:
    """節點共用的環境。

    runner      TaskRunner
    snapshot    () → 這一次 tick 的 Snapshot
    zero        () → 物件，有 done()、result() → (成功與否, 訊息)；呼叫 /auv_adapter/zero
    zero_here   () → None；歸零成功後馬上呼叫，跟 adapter 同時記零點
    interactive 是否要人確認檢查清單（模擬時 false）
    log         (str) → None
    """

    def __init__(self, runner=None, snapshot=None, zero=None, zero_here=None,
                 interactive=False, log=print):
        self.runner, self.snapshot, self.zero, self.zero_here = runner, snapshot, zero, zero_here
        self.interactive, self.log = interactive, log


class Leaf(py_trees.behaviour.Behaviour):
    """參數表 PARAMS：{名稱: 預設值}，REQUIRED 列出必填的。loader 用來檢查 YAML。"""
    PARAMS = {}
    REQUIRED = ()

    def __init__(self, name, ctx, **params):
        super().__init__(name)
        self.ctx = ctx
        self.params = {**self.PARAMS, **params}

    def fail(self, message):
        self.feedback_message = message
        if self.ctx:
            self.ctx.log(f'--- {self.name} 失敗：{message}')
        return BtStatus.FAILURE


class TaskAction(Leaf):
    def __init__(self, name, ctx, task_cls, **params):
        super().__init__(name, ctx, **params)
        self.task_cls = task_cls
        self.handle = None

    def initialise(self):
        self.error = None
        try:
            task = self.task_cls(**self.params)
            self.handle = self.ctx.runner.start(task, self.ctx.snapshot())
        except RuntimeError as e:       # 例如還沒解鎖就要開始動作
            self.error = str(e)

    def update(self):
        if self.error:
            return self.fail(self.error)
        status, text = self.ctx.runner.status(self.handle)
        self.feedback_message = text
        if status == TaskStatus.RUNNING:
            return BtStatus.RUNNING
        return BtStatus.SUCCESS if status == TaskStatus.SUCCEEDED else BtStatus.FAILURE

    def terminate(self, new_status):
        if new_status == BtStatus.INVALID and self.handle is not None:
            self.ctx.runner.cancel(self.handle)
        self.handle = None


class WaitReady(Leaf):
    """等到開始條件都成立。逾時用 YAML 的 timeout 設（中斷時會印出還缺什麼）。"""
    PARAMS = {'cameras': [], 'detections': False, 'disarmed': True}

    def missing(self, snap):
        p = self.params
        out = []
        if not snap.connected:
            out.append('飛控沒連線（/mavros/state）')
        if not snap.adapter_alive:
            out.append('adapter 沒在送 MANUAL_CONTROL')
        if p['disarmed'] and snap.armed:
            out.append('飛控是解鎖狀態，先上鎖')
        for cam in p['cameras']:
            if cam not in snap.cameras_ready:
                out.append(f'相機 {cam} 沒有 camera_info')
        if p['detections'] and snap.now - snap.detections_t > 2.0:
            out.append('收不到 /detections（object_detector 在跑嗎？）')
        return out

    def update(self):
        missing = self.missing(self.ctx.snapshot())
        self.feedback_message = '、'.join(missing) if missing else '就緒'
        return BtStatus.RUNNING if missing else BtStatus.SUCCESS

    def terminate(self, new_status):
        # self.status 還是被中斷前的狀態；已經成功的節點在樹結束時也會收到 INVALID
        if new_status == BtStatus.INVALID and self.status == BtStatus.RUNNING and self.ctx:
            self.ctx.log(f'--- {self.name} 沒等到：{self.feedback_message}')


class Confirm(Leaf):
    """列出檢查清單，等人按 Enter（interactive 為 false 時直接成功）。"""
    PARAMS = {'checklist': []}

    def initialise(self):
        self.answer = None
        if not self.ctx.interactive:
            return
        lines = ['開始前確認：'] + [f'  - {item}' for item in self.params['checklist']]
        print('\n'.join(lines), flush=True)

        def ask():
            try:
                input('  >>> 都確認了按 Enter（Ctrl-C 取消）')
                self.answer = True
            except EOFError:
                self.answer = False
        threading.Thread(target=ask, daemon=True).start()

    def update(self):
        if not self.ctx.interactive or self.answer:
            return BtStatus.SUCCESS
        if self.answer is False:
            return self.fail('沒有終端機可以確認（docker compose run 要加 -it，或用 --no-checklist）')
        self.feedback_message = '等使用者確認'
        return BtStatus.RUNNING


class Zero(Leaf):
    """呼叫 /auv_adapter/zero，成功後同時在這裡記零點。要在浮在水面、靜止時做。"""

    def initialise(self):
        self.future = self.ctx.zero()

    def update(self):
        if not self.future.done():
            self.feedback_message = '等 /auv_adapter/zero'
            return BtStatus.RUNNING
        ok, message = self.future.result()
        if not ok:
            return self.fail(f'歸零失敗：{message}')
        self.ctx.zero_here()
        self.ctx.log(f'--- 歸零：{message}')
        return BtStatus.SUCCESS


class Arm(Leaf):
    """切 DEPTH_HOLD、設好目標深度與航向，再解鎖（adapter 解鎖後才把目標送給飛控）。"""
    PARAMS = {'depth': 0.0, 'yaw': 0.0, 'mode_timeout': 5.0, 'arm_timeout': 5.0}

    def initialise(self):
        p = self.params
        self.ctx.runner.activate(Command(0.0, 0.0, p['depth'], p['yaw']))
        self.t0 = self.ctx.snapshot().now
        self.armed_sent = None

    def update(self):
        snap, p = self.ctx.snapshot(), self.params
        if self.armed_sent is None:
            if snap.mode == 'ALT_HOLD':
                self.ctx.runner.arm()
                self.armed_sent = snap.now
            elif snap.now - self.t0 > p['mode_timeout']:
                return self.fail(f'沒有切進 ALT_HOLD（目前 {snap.mode}），深度計正常嗎？')
            self.feedback_message = '切 ALT_HOLD'
            return BtStatus.RUNNING
        if snap.armed:
            self.ctx.log('--- 已解鎖')
            return BtStatus.SUCCESS
        if snap.now - self.armed_sent > p['arm_timeout']:
            return self.fail('解鎖失敗')
        self.feedback_message = '解鎖中'
        return BtStatus.RUNNING


class Finish(Leaf):
    """MANUAL、全 0、DISARM，等飛控上鎖（船浮上來）。樹結束時 node.py 也一定會做一次。"""
    PARAMS = {'timeout_s': 5.0}

    def initialise(self):
        self.ctx.runner.finish()
        self.t0 = self.ctx.snapshot().now

    def update(self):
        snap = self.ctx.snapshot()
        if not snap.armed:
            return BtStatus.SUCCESS
        if snap.now - self.t0 > self.params['timeout_s']:
            return self.fail('飛控沒有上鎖')
        self.feedback_message = '上鎖中'
        return BtStatus.RUNNING


class Seen(Leaf):
    """條件：現在看得到 labels 其中之一（max_age 秒內拍的、分數 ≥ min_score）。"""
    PARAMS = {'labels': [], 'camera': None, 'min_score': 0.5, 'max_age': 0.5}
    REQUIRED = ('labels',)

    def update(self):
        p = self.params
        s = self.ctx.snapshot().best(p['labels'], p['camera'], p['max_age'], p['min_score'])
        self.feedback_message = f'{s.label} {s.score:.2f}' if s else '沒看到'
        return BtStatus.SUCCESS if s else BtStatus.FAILURE


class Timeout(py_trees.decorators.Decorator):
    """跟 py_trees.decorators.Timeout 一樣，但時間取自 Snapshot.now（離線測試可以快轉）。"""

    def __init__(self, name, child, duration, ctx):
        super().__init__(name=name, child=child)
        self.duration, self.ctx = duration, ctx

    def now(self):
        return self.ctx.snapshot().now if self.ctx else time.monotonic()

    def initialise(self):
        self.finish_time = self.now() + self.duration

    def update(self):
        left = self.finish_time - self.now()
        if self.decorated.status == BtStatus.RUNNING and left < 0:
            self.decorated.stop(BtStatus.INVALID)
            self.feedback_message = f'逾時（{self.duration:g} s）'
            if self.ctx:
                self.ctx.log(f'--- {self.decorated.name} 逾時（{self.duration:g} s）')
            return BtStatus.FAILURE
        self.feedback_message = f'剩 {max(left, 0):.0f} s'
        return self.decorated.status


BEHAVIOURS = {cls.__name__: cls for cls in (WaitReady, Confirm, Zero, Arm, Finish, Seen)}
