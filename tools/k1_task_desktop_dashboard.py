#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Float32, String
except Exception:  # noqa: BLE001 - dashboard must still serve without ROS sourced.
    rclpy = None
    Node = object
    Float32 = String = Odometry = None
    ExternalShutdownException = Exception


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>K1 任务控制台</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #05070b;
      --panel: #0f1724;
      --panel2: #121d2e;
      --line: #29364d;
      --text: #f8fafc;
      --muted: #9aa8bd;
      --ok: #22c55e;
      --warn: #f59e0b;
      --bad: #ef4444;
      --blue: #38bdf8;
      --violet: #a78bfa;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: "Microsoft YaHei", "Noto Sans CJK SC", "Segoe UI", Arial, sans-serif;
      letter-spacing: 0;
    }
    header {
      height: 72px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 28px;
      border-bottom: 1px solid var(--line);
      background: #080d15;
    }
    h1 { margin: 0; font-size: 28px; font-weight: 800; }
    .sub { color: var(--muted); font-size: 15px; margin-top: 4px; }
    main {
      display: grid;
      grid-template-columns: 1.1fr .9fr;
      gap: 16px;
      padding: 16px;
    }
    .grid { display: grid; gap: 16px; }
    .cards { display: grid; grid-template-columns: repeat(4, minmax(120px, 1fr)); gap: 12px; }
    .card, .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
    }
    .label { color: var(--muted); font-size: 14px; margin-bottom: 8px; }
    .value { font-size: 32px; line-height: 1.1; font-weight: 900; }
    .unit { font-size: 16px; color: var(--muted); margin-left: 4px; }
    .ok { color: var(--ok); }
    .warn { color: var(--warn); }
    .bad { color: var(--bad); }
    .blue { color: var(--blue); }
    .violet { color: var(--violet); }
    .tasks { display: grid; gap: 10px; }
    .task {
      display: grid;
      grid-template-columns: 28px 1fr auto;
      gap: 12px;
      align-items: center;
      background: var(--panel2);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
    }
    .dot {
      width: 18px; height: 18px; border-radius: 50%;
      background: #334155; border: 2px solid #64748b;
    }
    .dot.ok { background: var(--ok); border-color: #86efac; }
    .dot.warn { background: var(--warn); border-color: #fde68a; }
    .dot.bad { background: var(--bad); border-color: #fecaca; }
    .task h3 { margin: 0; font-size: 18px; }
    .task p { margin: 4px 0 0; color: var(--muted); font-size: 13px; }
    .pill {
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 6px 10px;
      color: var(--muted);
      background: #09111d;
      font-size: 13px;
      white-space: nowrap;
    }
    table { width: 100%; border-collapse: collapse; }
    td { padding: 10px 8px; border-bottom: 1px solid var(--line); font-size: 15px; }
    td:first-child { color: var(--muted); width: 42%; }
    .footer { color: var(--muted); font-size: 13px; padding: 0 16px 16px; }
    @media (max-width: 1100px) {
      main { grid-template-columns: 1fr; }
      .cards { grid-template-columns: repeat(2, minmax(120px, 1fr)); }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>K1 复杂受限空间任务控制台</h1>
      <div class="sub">2m x 2m 复赛复刻场景 | 建图、风险识别、探图预览、处置准备</div>
    </div>
    <div id="clock" class="pill">--</div>
  </header>
  <main>
    <section class="grid">
      <div class="cards">
        <div class="card"><div class="label">电源</div><div id="battery" class="value">--</div><div id="batterySub" class="label">等待 /battery_voltage</div></div>
        <div class="card"><div class="label">前向距离</div><div id="front" class="value">--</div><div id="guardState" class="label">安全守护未启动</div></div>
        <div class="card"><div class="label">YOLO 帧率</div><div id="fps" class="value">--</div><div id="riskCount" class="label">风险事件 --</div></div>
        <div class="card"><div class="label">位置</div><div id="odom" class="value">--</div><div id="yaw" class="label">odom 未启动</div></div>
      </div>
      <div class="panel">
        <div class="label">任务步骤</div>
        <div class="tasks" id="tasks"></div>
      </div>
    </section>
    <section class="grid">
      <div class="panel">
        <div class="label">当前风险</div>
        <table>
          <tbody>
            <tr><td>告警</td><td id="alarm">--</td></tr>
            <tr><td>类别</td><td id="riskClass">--</td></tr>
            <tr><td>深度/距离</td><td id="riskDist">--</td></tr>
            <tr><td>事件数</td><td id="events">--</td></tr>
            <tr><td>Run 目录</td><td id="runDir">--</td></tr>
          </tbody>
        </table>
      </div>
      <div class="panel">
        <div class="label">系统状态</div>
        <table>
          <tbody>
            <tr><td>底盘待命</td><td id="procBase">--</td></tr>
            <tr><td>D435</td><td id="procD435">--</td></tr>
            <tr><td>YOLO EP</td><td id="procYolo">--</td></tr>
            <tr><td>Nav2/SLAM</td><td id="procNav2">--</td></tr>
            <tr><td>RRT</td><td id="procRrt">--</td></tr>
            <tr><td>最后刷新</td><td id="updated">--</td></tr>
          </tbody>
        </table>
      </div>
    </section>
  </main>
  <div class="footer">本界面只显示任务状态，不直接下发底盘速度。运动仍通过 /cmd_vel_raw 或 /input_cmd_vel 进入安全守护。</div>
<script>
const taskNames = [
  ["base", "底盘/雷达建图", "SLAM、/scan、/odom 与安全守护"],
  ["d435", "D435 RGB-D", "风险识别输入"],
  ["yolo", "YOLOv8n 本地推理", "SpaceMIT EP / CPU fallback"],
  ["rrt", "RRT/Nav2 探图", "2m 场景目标预览或执行"],
  ["risk", "风险点落图", "结构化事件与地图风险点"],
  ["report", "报告/处置准备", "人工处置任务与语音播报"]
];
function text(id, value, cls) {
  const el = document.getElementById(id);
  el.textContent = value;
  if (cls !== undefined) el.className = cls;
}
function fmt(n, d=2) {
  const x = Number(n);
  return Number.isFinite(x) ? x.toFixed(d) : "--";
}
function procText(ok) { return ok ? "运行中" : "未启动"; }
function procClass(ok) { return ok ? "ok" : "warn"; }
function renderTasks(data) {
  const p = data.processes || {};
  const riskEvents = Number(data.risk?.event_count || 0);
  const statuses = {
    base: data.ros?.odom?.fresh || data.ros?.guard?.fresh,
    d435: !!p.d435,
    yolo: !!p.yolo || Number(data.risk?.infer_fps || 0) > 0,
    rrt: !!p.rrt,
    risk: riskEvents > 0 || !!data.ros?.risk_alarm?.fresh,
    report: riskEvents > 0
  };
  const box = document.getElementById("tasks");
  box.innerHTML = taskNames.map(([id, name, desc]) => {
    const ok = !!statuses[id];
    const dot = ok ? "ok" : "warn";
    const state = ok ? "就绪" : "等待";
    return `<div class="task"><div class="dot ${dot}"></div><div><h3>${name}</h3><p>${desc}</p></div><div class="pill">${state}</div></div>`;
  }).join("");
}
async function refresh() {
  try {
    const res = await fetch("/api/status?ts=" + Date.now(), {cache: "no-store"});
    const data = await res.json();
    const voltage = data.ros?.battery_voltage_v;
    const rawVoltage = data.ros?.battery_raw_voltage_v;
    const percent = data.ros?.battery_percent;
    if (Number.isFinite(Number(voltage))) {
      const low = !!data.ros?.battery_low;
      const warn = low || Number(percent) < 25;
      text("battery", fmt(voltage, 2) + "V", low ? "value bad" : (warn ? "value warn" : "value ok"));
      if (low) {
        text("batterySub", `低电压提醒: <= ${fmt(data.ros?.battery_warn_v, 2)}V`);
      } else if (Number.isFinite(Number(rawVoltage))) {
        text("batterySub", `估算 ${fmt(percent, 0)}% | raw ${fmt(rawVoltage, 2)}V + ${fmt(data.ros?.battery_offset_v, 2)}V`);
      } else {
        text("batterySub", Number.isFinite(Number(percent)) ? `估算 ${fmt(percent, 0)}%` : "3S 电池电压");
      }
    } else {
      text("battery", "--", "value warn");
      text("batterySub", "等待 /battery_voltage");
    }
    const front = data.ros?.guard?.front_min_range_m ?? data.ros?.guard?.front_p10_range_m;
    text("front", Number.isFinite(Number(front)) ? fmt(front, 2) + "m" : "--", "value blue");
    text("guardState", data.ros?.guard?.state || "安全守护未启动");
    if (data.risk?.infer_fps) text("fps", fmt(data.risk.infer_fps, 2), "value violet"); else text("fps", "--", "value warn");
    text("riskCount", "风险事件 " + (data.risk?.event_count ?? "--"));
    const od = data.ros?.odom || {};
    if (od.fresh) {
      text("odom", `${fmt(od.x, 1)},${fmt(od.y, 1)}`, "value");
      text("yaw", `yaw ${fmt(od.yaw_deg, 0)}°`);
    } else {
      text("odom", "--", "value warn");
      text("yaw", "odom 未启动");
    }
    const latest = data.risk?.latest_event || data.ros?.risk_alarm?.payload || {};
    text("alarm", (data.risk?.alarm_active || latest.alarm) ? "ALARM" : "未报警", (data.risk?.alarm_active || latest.alarm) ? "bad" : "ok");
    text("riskClass", latest.class_name || "--");
    text("riskDist", latest.distance_m ? fmt(latest.distance_m, 3) + "m" : "--");
    text("events", data.risk?.event_count ?? "--");
    text("runDir", data.run_dir || "--");
    const p = data.processes || {};
    text("procD435", procText(p.d435), procClass(p.d435));
    text("procBase", procText(p.base), procClass(p.base));
    text("procYolo", procText(p.yolo), procClass(p.yolo));
    text("procNav2", procText(p.nav2_or_mapping), procClass(p.nav2_or_mapping));
    text("procRrt", procText(p.rrt), procClass(p.rrt));
    text("updated", data.updated_at || "--");
    text("clock", data.local_time || "--");
    renderTasks(data);
  } catch (err) {
    text("updated", "连接失败: " + err.message, "bad");
  }
}
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
"""


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        if not path.exists():
            return None
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def proc_running(pattern: str) -> bool:
    try:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0
    except Exception:
        return False


def estimate_3s_percent(voltage: Optional[float]) -> Optional[float]:
    if voltage is None:
        return None
    # Conservative display estimate for a 3S LiPo pack under light load.
    return max(0.0, min(100.0, (float(voltage) - 9.6) / (12.6 - 9.6) * 100.0))


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class DashboardState:
    def __init__(self, repo_dir: Path):
        self.repo_dir = repo_dir
        self.lock = threading.Lock()
        self.battery_raw_voltage_v: Optional[float] = None
        self.battery_offset_v = env_float("K1_BATTERY_VOLTAGE_OFFSET_V", 0.83)
        self.battery_warn_v = env_float("K1_BATTERY_WARN_V", 3.7 * 3.0)
        self.battery_time = 0.0
        self.guard_payload: Optional[Dict[str, Any]] = None
        self.guard_time = 0.0
        self.odom_payload: Optional[Dict[str, Any]] = None
        self.odom_time = 0.0
        self.risk_alarm_payload: Optional[Dict[str, Any]] = None
        self.risk_alarm_time = 0.0

    def run_dir(self) -> Optional[Path]:
        current_file = self.repo_dir / ".current_real_k1_rrt_nav2_run_dir"
        try:
            if current_file.exists():
                text = current_file.read_text(encoding="utf-8").strip()
                if text:
                    return Path(text)
        except Exception:
            pass
        return None

    def snapshot(self) -> Dict[str, Any]:
        now = time.time()
        with self.lock:
            guard = dict(self.guard_payload or {})
            odom = dict(self.odom_payload or {})
            risk_alarm = dict(self.risk_alarm_payload or {})
            raw_voltage = self.battery_raw_voltage_v
            guard_fresh = now - self.guard_time <= 2.5
            odom_fresh = now - self.odom_time <= 2.5
            alarm_fresh = now - self.risk_alarm_time <= 5.0
        corrected_voltage = None
        if raw_voltage is not None:
            corrected_voltage = round(float(raw_voltage) + self.battery_offset_v, 3)
        if guard:
            guard["fresh"] = guard_fresh
        if odom:
            odom["fresh"] = odom_fresh

        run_dir = self.run_dir()
        risk = {}
        if run_dir is not None:
            risk = (
                read_json(run_dir / "yolo_risk" / "alarm_state.json")
                or read_json(run_dir / "alarm_state.json")
                or {}
            )

        return {
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "local_time": datetime.now().strftime("%H:%M:%S"),
            "run_dir": None if run_dir is None else str(run_dir),
            "ros": {
                "battery_voltage_v": corrected_voltage,
                "battery_raw_voltage_v": raw_voltage,
                "battery_offset_v": round(self.battery_offset_v, 3),
                "battery_warn_v": round(self.battery_warn_v, 3),
                "battery_low": corrected_voltage is not None and corrected_voltage <= self.battery_warn_v,
                "battery_percent": estimate_3s_percent(corrected_voltage),
                "guard": guard,
                "odom": odom,
                "risk_alarm": {"fresh": alarm_fresh, "payload": risk_alarm},
            },
            "risk": risk,
            "processes": {
                "base": proc_running("wheeltec_tank_base_safe.py") or proc_running("tank_base_safe.launch.py"),
                "d435": proc_running("realsense2_camera.*rs_launch.py"),
                "yolo": proc_running("run_prelim_remote_mapping_yolo_arm_demo.py"),
                "nav2_or_mapping": proc_running("n10p_tank_nav2_slam.launch.py")
                or proc_running("n10p_tank_mapping_safety_guard.launch.py"),
                "rrt": proc_running("sim_rrt_frontier_explorer.py"),
            },
        }


class RosStatusNode(Node):
    def __init__(self, state: DashboardState):
        super().__init__("k1_task_dashboard_status")
        self.state = state
        self.create_subscription(Float32, "/battery_voltage", self.battery_cb, 10)
        self.create_subscription(String, "/safety/front_obstacle", self.guard_cb, 10)
        self.create_subscription(String, "/perception/risk_alarm", self.risk_alarm_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

    def battery_cb(self, msg: Float32) -> None:
        with self.state.lock:
            self.state.battery_raw_voltage_v = round(float(msg.data), 3)
            self.state.battery_time = time.time()

    def guard_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        with self.state.lock:
            self.state.guard_payload = payload
            self.state.guard_time = time.time()

    def risk_alarm_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        with self.state.lock:
            self.state.risk_alarm_payload = payload
            self.state.risk_alarm_time = time.time()

    def odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = yaw_from_quat(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        payload = {
            "x": round(float(pose.position.x), 3),
            "y": round(float(pose.position.y), 3),
            "yaw_deg": round(math.degrees(yaw), 1),
            "linear_x_mps": round(float(twist.linear.x), 3),
            "angular_z_radps": round(float(twist.angular.z), 3),
        }
        with self.state.lock:
            self.state.odom_payload = payload
            self.state.odom_time = time.time()


def run_ros_thread(state: DashboardState) -> None:
    if rclpy is None:
        return
    try:
        rclpy.init()
        node = RosStatusNode(state)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except Exception:
        return


def make_handler(state: DashboardState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path.startswith("/api/status"):
                body = json.dumps(state.snapshot(), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
                return
            if self.path in ("/", "/index.html"):
                self._send(200, HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            self._send(404, b"not found\n", "text/plain; charset=utf-8")

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-dir", default="/home/soc/edge-ai-robot-k1")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8780)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = DashboardState(Path(args.repo_dir))
    threading.Thread(target=run_ros_thread, args=(state,), daemon=True).start()
    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    print(f"k1_task_dashboard listening on http://{args.host}:{args.port}/", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
