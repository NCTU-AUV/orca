"""偵測結果 → 方位。這個檔案不 import ROS，可以單獨測。

輸入是 orca_perception 的 /detections（vision_msgs/Detection2DArray）：
- bbox 是原始影像的像素座標（detector 不做 letterbox，不用還原）；
- header 沿用影像的：stamp 是拍攝時間，frame_id 是哪一台相機。
  每台相機的 frame_id 要不一樣（config/cameras.yaml）。

每一筆偵測換算成 Sighting：
- bearing／elevation：相對相機光軸的水平、垂直角度 [deg]，用該相機 camera_info 的內參。
  內參是空氣中的；平面視窗在水下會把角度放大約 1.33 倍，閉迴路會收斂，只是等效增益變小。
- heading：拍攝當下的航向（VehicleState.heading_at 內插）。
- abs_bearing：目標的絕對方位 = heading + 相機安裝的 yaw + bearing，只有前視相機有意義
  （朝下的相機是 NaN）。遙測約 4 Hz，轉向中內插的誤差會變大；要用絕對方位時，
  先讓船停止轉向再量比較準。

另外保留「各 label 最後一次被看到」的那筆（last_seen），不做任何估測。
"""
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class CameraConfig:
    name: str                   # front、bottom
    frame_id: str               # 影像 header.frame_id
    info_topic: str             # camera_info 的 topic
    mount_yaw: float = 0.0      # 光軸相對船頭，往右為正 [deg]
    mount_pitch: float = 0.0    # 光軸仰角，0 = 前視、-90 = 朝下 [deg]

    @property
    def forward_looking(self):
        return abs(self.mount_pitch) < 45.0


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Sighting:
    label: str
    score: float
    camera: str
    t: float                # 拍攝時間（跟 Snapshot.now 同一個時鐘）
    frame: int              # 這台相機的第幾個偵測結果（每則 /detections +1）
    u: float                # bbox 中心 [px]
    v: float
    offset_x: float         # 畫面中央 0、右緣 +1、左緣 -1
    offset_y: float         # 畫面中央 0、下緣 +1、上緣 -1
    bearing: float          # 相對光軸，往右為正 [deg]
    elevation: float        # 相對光軸，往上為正 [deg]
    width: float            # bbox 寬 / 畫面寬
    height: float           # bbox 高 / 畫面高
    heading: float          # 拍攝當下的航向 [deg]，不知道時 NaN
    abs_bearing: float      # 絕對方位 [deg]（前視相機），否則 NaN


@dataclass
class _Frame:
    seq: int = 0
    t: float = -math.inf
    sightings: list = field(default_factory=list)


class Perception:
    def __init__(self, cameras):
        self.cameras = {c.name: c for c in cameras}
        self.by_frame_id = {c.frame_id: c for c in cameras}
        self.intrinsics = {}                        # name → Intrinsics
        self.frames = {c.name: _Frame() for c in cameras}
        self.last_seen = {}                         # label → Sighting
        self.unknown_frame_ids = set()

    def on_camera_info(self, camera, width, height, k):
        if width and height and k[0] > 0 and k[4] > 0:
            self.intrinsics[camera] = Intrinsics(width, height, k[0], k[4], k[2], k[5])

    def on_detections(self, stamp, frame_id, detections, heading_at):
        """detections：[(label, score, 中心 u, 中心 v, 寬, 高)]（像素）。
        heading_at(t) → 航向 [deg] 或 NaN。回傳換算好的 Sighting 列表；不認得的相機回傳 None。"""
        cam = self.by_frame_id.get(frame_id)
        if cam is None:
            self.unknown_frame_ids.add(frame_id)
            return None
        k = self.intrinsics.get(cam.name)
        frame = self.frames[cam.name]
        frame.seq += 1
        frame.t = stamp
        if k is None:                   # 還沒收到 camera_info
            frame.sightings = []
            return []
        heading = heading_at(stamp)
        out = []
        for label, score, u, v, w, h in detections:
            bearing = math.degrees(math.atan2(u - k.cx, k.fx))
            elevation = math.degrees(math.atan2(k.cy - v, k.fy))
            absb = math.nan
            if cam.forward_looking and math.isfinite(heading):
                absb = (heading + cam.mount_yaw + bearing + 180.0) % 360.0 - 180.0
            out.append(Sighting(
                label=label, score=score, camera=cam.name, t=stamp, frame=frame.seq, u=u, v=v,
                offset_x=(u - k.cx) / (k.width / 2), offset_y=(v - k.cy) / (k.height / 2),
                bearing=bearing, elevation=elevation, width=w / k.width, height=h / k.height,
                heading=heading, abs_bearing=absb))
        out.sort(key=lambda s: -s.score)
        frame.sightings = out
        for s in out:
            prev = self.last_seen.get(s.label)
            if prev is None or s.t >= prev.t:
                self.last_seen[s.label] = s
        return out

    def snapshot(self):
        """給 Snapshot 用：每台相機最新一次的偵測（唯讀的 tuple）。"""
        return {name: (f.seq, f.t, tuple(f.sightings)) for name, f in self.frames.items()}


def find(frames, now, labels, camera=None, max_age=0.5, min_score=0.0):
    """frames（Perception.snapshot()）裡符合條件的 Sighting，分數高的在前。
    只看各相機「最新那一次」的偵測，而且要在 max_age 秒內拍的。"""
    labels = {labels} if isinstance(labels, str) else set(labels)
    out = []
    for name, (_, t, sightings) in frames.items():
        if camera is not None and name != camera:
            continue
        if now - t > max_age:
            continue
        out.extend(s for s in sightings if s.label in labels and s.score >= min_score)
    out.sort(key=lambda s: -s.score)
    return out
