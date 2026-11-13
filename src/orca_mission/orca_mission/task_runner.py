"""TaskRunner：/auv_control 唯一的主人。不 import ROS（node.py 把 step() 的輸出發出去）。

- 行為樹的動作節點用 start()／status()／cancel() 操作執行器，同一時間只有一個在跑；
  start 新的會先取消舊的。
- step() 以 20 Hz 被呼叫：推進執行器一步，結果經 Shaper 限幅、限制轉速後回傳 Output。
- 執行器結束（成功、失敗、取消）後保持它最後的目標深度與航向，推力歸 0。
- 歸零之後、activate() 之前什麼都不送（Output 是 None）；activate() 之後一直送 DEPTH_HOLD；
  finish() 之後一直送 MANUAL、全 0，並送一次 DISARM。
- 深度或航向沒資料超過 state_timeout：正在跑的執行器失敗。

模式、解鎖只有任務層（行為樹的 Arm、Finish 節點，以及 node.py 的收尾）能動，執行器不行。
"""
import dataclasses
import enum

from .tasks.base import Status
from .vehicle import Command, Shaper


class Mode(enum.Enum):
    IDLE = 'IDLE'           # 不送
    ACTIVE = 'ACTIVE'       # DEPTH_HOLD + 目標
    FINISH = 'FINISH'       # MANUAL、全 0、DISARM


@dataclasses.dataclass(frozen=True)
class Output:
    mode: str               # 'DEPTH_HOLD' 或 'MANUAL'
    arm: str                # 'ARM'、'DISARM' 或 ''（不變）
    cmd: Command


class TaskRunner:
    def __init__(self, shaper=None, state_timeout=2.0, log=print):
        self.shaper = shaper or Shaper()
        self.state_timeout = state_timeout
        self.log = log
        self.mode = Mode.IDLE
        self.cmd = Command()            # 目前保持的指令（已經過 Shaper）
        self.arm_event = ''
        self.task = None
        self.handle = 0
        self.results = {}               # handle → (Status, message)，結束的執行器
        self.bad_since = None
        self.last_t = None

    # ---------- 任務層專用 ----------

    def activate(self, cmd):
        """開始送 DEPTH_HOLD 與 cmd 的目標（解鎖前先送，adapter 解鎖後才把目標送給飛控）。"""
        self.cmd = cmd.stopped()
        self.shaper.reset(cmd.target_yaw)
        self.mode = Mode.ACTIVE

    def arm(self):
        self.arm_event = 'ARM'

    def finish(self):
        """取消執行器，之後一直送 MANUAL、全 0，並送一次 DISARM。"""
        self.cancel()
        if self.mode != Mode.FINISH:
            self.mode = Mode.FINISH
            self.arm_event = 'DISARM'

    # ---------- 執行器（行為樹的動作節點用） ----------

    def start(self, task, snap):
        if self.mode != Mode.ACTIVE:
            raise RuntimeError(f'還沒進入 DEPTH_HOLD（目前 {self.mode.value}），不能開始 {task.name}')
        self.cancel()
        self.handle += 1
        self.task = (self.handle, task)
        self.bad_since = None
        task.start(snap, self.cmd.stopped())
        self.log(f'--- {task.name} 開始')
        return self.handle

    def status(self, handle):
        if self.task and self.task[0] == handle:
            return Status.RUNNING, self.task[1].feedback
        return self.results.get(handle, (Status.CANCELED, ''))

    def cancel(self, handle=None):
        if self.task and (handle is None or self.task[0] == handle):
            self._end(Status.CANCELED, '取消')

    def running(self):
        return self.task[1] if self.task else None

    def _end(self, status, message):
        handle, task = self.task
        task.status, task.message = status, message
        self.results[handle] = (status, message)
        if len(self.results) > 50:
            self.results.pop(min(self.results))
        self.task = None
        self.cmd = self.cmd.stopped()
        self.log(f'--- {task.name} {status.value}：{message}')

    # ---------- 20 Hz ----------

    def step(self, snap):
        dt = 0.0 if self.last_t is None else snap.now - self.last_t
        self.last_t = snap.now
        if self.mode == Mode.IDLE:
            return None
        arm, self.arm_event = self.arm_event, ''
        if self.mode == Mode.FINISH:
            return Output('MANUAL', arm, Command())

        if self.task:
            handle, task = self.task
            if snap.state_ok:
                self.bad_since = None
            else:
                self.bad_since = snap.now if self.bad_since is None else self.bad_since
            if self.bad_since is not None and snap.now - self.bad_since > self.state_timeout:
                self._end(Status.FAILED, '收不到飛控的深度或航向')
            else:
                try:
                    want = task.tick(snap)
                except Exception as e:      # 執行器的 bug 不該讓推進器停在最後一筆
                    self._end(Status.FAILED, f'{type(e).__name__}: {e}')
                    raise
                self.cmd = self.shaper(want, dt)
                if task.status != Status.RUNNING:
                    self._end(task.status, task.message)
        else:
            self.cmd = self.shaper(self.cmd, dt)
        return Output('DEPTH_HOLD', arm, self.cmd)

