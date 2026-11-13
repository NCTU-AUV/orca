"""執行器（Task）的共同介面。

一個 Task 是「一段有開始、有結束的動作」，形式跟 ROS action 一樣：
  goal     = 建構時給的參數（Params dataclass，行為樹 YAML 裡那個節點的參數）
  feedback = self.feedback（給人看的一行字，狀態 JSON 和 log 會帶）
  result   = self.status（SUCCEEDED／FAILED／CANCELED）＋ self.message

TaskRunner 以 20 Hz 呼叫 step(snap)，Task 回傳這一步的 Command（目標深度、目標航向、surge、sway）。
閉迴路（姿態、定深、定向）在飛控；Task 只決定「目標」和正規化的推力，不算力矩。
Task 不碰 ROS，也不能歸零、解鎖、切模式（那些只有任務層的行為節點能做）。
"""
import dataclasses
import enum
import math

from ..vehicle import Command


class Status(enum.Enum):
    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


class Task:
    @dataclasses.dataclass
    class Params:
        pass

    def __init__(self, params=None, **kwargs):
        self.p = params if params is not None else self.Params(**kwargs)
        self.status = Status.RUNNING
        self.feedback = ''
        self.message = ''
        self.hold = Command()       # 開始時正在送的指令（目前的目標深度、航向）
        self.last = self.hold       # 上一步回傳的指令
        self.t0 = None
        self._since = None

    @property
    def name(self):
        return type(self).__name__

    def start(self, snap, hold):
        """TaskRunner 在開始時呼叫一次。hold：目前正在送的指令（推力已歸 0）。"""
        self.hold = self.last = hold
        self.t0 = snap.now
        self.on_start(snap)

    def on_start(self, snap):
        pass

    def tick(self, snap):
        """TaskRunner 每一步呼叫這個（不要覆寫）。"""
        self.last = self.step(snap)
        return self.last

    def step(self, snap):
        """子類別實作：回傳這一步的 Command；結束時呼叫 succeed()／fail() 並回傳它的結果。"""
        raise NotImplementedError

    # ---------- 小工具 ----------

    def elapsed(self, snap):
        return snap.now - self.t0

    def succeed(self, message='', cmd=None):
        """成功結束。cmd：結束時的目標（預設是上一步的）；推力一律歸 0。"""
        self.status, self.message = Status.SUCCEEDED, message
        return (cmd or self.last).stopped()

    def fail(self, message, cmd=None):
        self.status, self.message = Status.FAILED, message
        return (cmd or self.last).stopped()

    def held(self, ok, now, seconds):
        """條件 ok 連續成立滿 seconds 秒才回傳 True。"""
        if not ok:
            self._since = None
            return False
        self._since = now if self._since is None else self._since
        return now - self._since >= seconds


def params_of(task_cls):
    """Task 的參數表：{名稱: 預設值（沒有預設值時是 dataclasses.MISSING）}。"""
    return {f.name: (f.default if f.default is not dataclasses.MISSING else
                     f.default_factory() if f.default_factory is not dataclasses.MISSING else
                     dataclasses.MISSING)
            for f in dataclasses.fields(task_cls.Params)}


def finite_or(v, default):
    return v if v is not None and math.isfinite(v) else default
