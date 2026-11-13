// Foxglove User Script：把 /mission/status（std_msgs/String，內容是 JSON，2 Hz）拆成欄位，
// 讓 Plot、State Transitions 可以直接用。欄位對應 src/orca_mission/orca_mission/node.py 的 on_status。
// 改了這個檔要重新產生 layout：python3 tools/foxglove/make_layout.py
import { Input } from "./types";

export const inputs = ["/mission/status"];
export const output = "/studio_script/mission";

type Output = {
  node: string; // 行為樹目前的節點（樹裡的 name，例如「平移對準門」）
  feedback: string; // 執行器的回饋
  tree: string; // 整棵樹的狀態：RUNNING、SUCCESS、FAILURE、INVALID
  runner: string; // TaskRunner：IDLE（不送）、ACTIVE（DEPTH_HOLD + 目標）、FINISH（上鎖）
  fc_mode: string; // 飛控回報的模式
  armed: boolean;
  depth: number; // 深度 [m]，歸零後、往下為正
  heading: number; // 航向 [deg]，歸零後
  surge: number;
  sway: number;
  target_depth: number;
  target_yaw: number;
};

function num(v: unknown): number {
  return typeof v === "number" ? v : NaN;
}

export default function script(event: Input<"/mission/status">): Output | undefined {
  let s: any;
  try {
    s = JSON.parse(event.message.data);
  } catch {
    return undefined;
  }
  const cmd = s.cmd ?? {};
  return {
    node: String(s.node ?? ""),
    feedback: String(s.feedback ?? ""),
    tree: String(s.status ?? ""),
    runner: String(s.runner ?? ""),
    fc_mode: String(s.fc_mode ?? ""),
    armed: s.armed === true,
    depth: num(s.depth),
    heading: num(s.heading),
    surge: num(cmd.surge),
    sway: num(cmd.sway),
    target_depth: num(cmd.target_depth),
    target_yaw: num(cmd.target_yaw),
  };
}
