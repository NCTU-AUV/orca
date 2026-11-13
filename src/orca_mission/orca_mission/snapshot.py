"""每次 tick 給行為樹和執行器的唯讀快照。不 import ROS。"""
import math
from dataclasses import dataclass, field

from .perception import find


@dataclass(frozen=True)
class Snapshot:
    now: float                      # 秒；跟偵測的 stamp 同一個時鐘（ROS 時間）
    connected: bool = False         # mavros 連上飛控
    armed: bool = False
    mode: str = ''                  # 飛控模式（ALT_HOLD、MANUAL…）
    zeroed: bool = False
    depth: float = math.nan         # [m]，往下為正；沒有資料時 NaN
    heading: float = math.nan       # [deg]，順時針為正；沒有資料時 NaN
    adapter_alive: bool = False
    cameras_ready: frozenset = frozenset()      # 收到 camera_info 的相機
    detections_t: float = -math.inf            # 最後一次收到 /detections
    frames: dict = field(default_factory=dict)  # Perception.snapshot()
    last_seen: dict = field(default_factory=dict)   # label → Sighting

    @property
    def state_ok(self):
        return math.isfinite(self.depth) and math.isfinite(self.heading)

    def sightings(self, labels, camera=None, max_age=0.5, min_score=0.0):
        return find(self.frames, self.now, labels, camera, max_age, min_score)

    def best(self, labels, camera=None, max_age=0.5, min_score=0.0):
        found = self.sightings(labels, camera, max_age, min_score)
        return found[0] if found else None
