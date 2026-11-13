// Foxglove User Script：從 /detections 算前視相機裡門的位置，跟 orca_mission 過門用的量相同
// （src/orca_mission/orca_mission/perception.py、tasks/gate.py）：
//   offset_x：門框中心在畫面的水平位置，中央 0、右緣 +1、左緣 -1
//   width：門框寬 / 畫面寬。ApproachGate 到 near_width（0.7）就開始盲走
//   lateral：offset_x / (2 × width) = 船偏離門中線幾個門寬。CenterGate 在 ±tol（0.1）內算對準
// 只看前視相機（frame_id camera_color_optical_frame），標籤 gate／qualification_gate、分數 ≥ 0.5 裡最高的一個。
// 前視的每一幀都輸出一筆；這一幀沒有門時 seen = false、數值是 NaN（曲線會斷開）。
// 改了這個檔要重新產生 layout：python3 tools/foxglove/make_layout.py
import { Input } from "./types";

export const inputs = ["/detections", "/camera/camera/color/camera_info"];
export const output = "/studio_script/gate";

const FRAME_ID = "camera_color_optical_frame";
const LABELS = ["gate", "qualification_gate"];
const MIN_SCORE = 0.5;

type Output = {
  seen: boolean;
  label: string;
  score: number;
  offset_x: number;
  width: number;
  lateral: number;
};

// 內參從 camera_info 來；還沒收到時用模擬前視相機的 640 寬、光心在正中央
let imageWidth = 640;
let cx = 320;

export default function script(
  event: Input<"/detections"> | Input<"/camera/camera/color/camera_info">,
): Output | undefined {
  if (event.topic === "/camera/camera/color/camera_info") {
    const info = event.message;
    const k02 = info.k[2] ?? 0; // 光心 cx
    if (info.width > 0 && k02 > 0) {
      imageWidth = info.width;
      cx = k02;
    }
    return undefined;
  }
  const msg = event.message;
  if (msg.header.frame_id !== FRAME_ID) {
    return undefined;
  }
  let best: { label: string; score: number; u: number; w: number } | undefined;
  for (const d of msg.detections) {
    const h = d.results[0]?.hypothesis;
    if (h == undefined || !LABELS.includes(h.class_id) || h.score < MIN_SCORE) {
      continue;
    }
    if (best == undefined || h.score > best.score) {
      best = { label: h.class_id, score: h.score, u: d.bbox.center.position.x, w: d.bbox.size_x };
    }
  }
  if (best == undefined) {
    return { seen: false, label: "", score: NaN, offset_x: NaN, width: NaN, lateral: NaN };
  }
  const offset = (best.u - cx) / (imageWidth / 2);
  const width = best.w / imageWidth;
  return {
    seen: true,
    label: best.label,
    score: best.score,
    offset_x: offset,
    width,
    lateral: width > 0 ? offset / (2 * width) : NaN,
  };
}
