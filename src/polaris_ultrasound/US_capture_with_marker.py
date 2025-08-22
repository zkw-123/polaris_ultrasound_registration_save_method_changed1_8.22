#!/usr/bin/env python3
# coding: UTF-8
import os, time, json, argparse, datetime, threading
import cv2
import numpy as np

# ---------- Polaris 订阅（后台线程，读取 JSON 字符串话题） ----------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from datetime import timezone

class PolarisBuffer(Node):
    """
    订阅 /ndi_transforms(JSON in std_msgs/String)，缓存最新 transforms。
    latest = {
      'timestamp_ros': float,
      'timestamp_hw': float,            # 如果消息里有 original_timestamp
      'transforms': {tool_name: {matrix, translation, quality, tool_id}}
    }
    """
    def __init__(self, topic_name='/ndi_transforms'):
        super().__init__('polaris_buffer_json_save')
        self._lock = threading.Lock()
        self.latest = None
        self.sub = self.create_subscription(StringMsg, topic_name, self._cb, 10)

    def _cb(self, msg: StringMsg):
        try:
            d = json.loads(msg.data)
            ts_ros = float(d.get('timestamp', 0.0))
            ts_hw  = float(d.get('original_timestamp', ts_ros))
            tfmap = {}
            for t in d.get('transforms', []):
                name = t.get('tool_name', '')
                if not name:
                    continue
                tfmap[name] = {
                    'matrix': t.get('matrix', []),
                    'translation': t.get('translation', []),
                    'quality': t.get('quality', 0.0),
                    'tool_id': t.get('tool_id', '')
                }
            with self._lock:
                self.latest = {
                    'timestamp_ros': ts_ros,
                    'timestamp_hw': ts_hw,
                    'transforms': tfmap
                }
        except Exception as e:
            self.get_logger().warn(f'Polaris parse error: {e}')

    def get_latest(self):
        with self._lock:
            return json.loads(json.dumps(self.latest)) if self.latest is not None else None

def start_polaris_thread(topic_name='/ndi_transforms'):
    rclpy.init(args=None)
    buf = PolarisBuffer(topic_name=topic_name)
    exec_ = rclpy.executors.SingleThreadedExecutor()
    exec_.add_node(buf)
    stop_flag = {'stop': False}

    def run():
        try:
            while not stop_flag['stop']:
                exec_.spin_once(timeout_sec=0.05)
        finally:
            exec_.remove_node(buf)
            buf.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    th = threading.Thread(target=run, daemon=True)
    th.start()

    def stop():
        stop_flag['stop'] = True
        th.join(timeout=2.0)

    return buf, th, stop

# ---------- 工具函数：UTC 文件名时间戳（与你提供的风格一致） ----------
def utc_filename_ts(epoch_sec: float) -> str:
    if epoch_sec <= 0:
        now = datetime.datetime.now(timezone.utc)
    else:
        now = datetime.datetime.fromtimestamp(epoch_sec, tz=timezone.utc)
    return now.strftime("%Y-%m-%dT%H-%M-%S.%fZ")  # 去冒号，保留微秒

# ---------- 交互采集主程序（无 mask，仅保存原始帧） ----------
def main():
    ap = argparse.ArgumentParser(description="Interactive ultrasound capture with per-frame Polaris JSON (no mask)")
    ap.add_argument("--device_id", type=int, default=6)
    ap.add_argument("--save_path", type=str, default="/ros2_ws/calibration_data/robot_ultrasound")
    ap.add_argument("--duration", type=float, default=5.0, help="seconds per point")
    ap.add_argument("--sample_rate", type=float, default=30.0, help="approx saving FPS")
    ap.add_argument("--polaris_topic", type=str, default="/ndi_transforms")
    ap.add_argument("--probe_name", type=str, default="probe")     # 关键工具名（写入 JSON 的主对象名）
    ap.add_argument("--phantom_name", type=str, default="phantom") # 若存在则一并写入
    args = ap.parse_args()

    os.makedirs(args.save_path, exist_ok=True)

    # 启动 Polaris 订阅
    buf, th, stop_polaris = start_polaris_thread(args.polaris_topic)

    # 打开相机（无 mask）
    cap = cv2.VideoCapture(args.device_id)
    
    # 强制设置分辨率为 1080p
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    if not cap.isOpened():
        print(f"Failed to open video device {args.device_id}")
        stop_polaris()
        return

    print("Interactive capture started. Type a point name to record, or 'q' / empty to quit.")
    try:
        while True:
            point = input("current point: ").strip()
            if point == "" or point.lower() == "q":
                print("Exit.")
                break

            session_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            point_dir = os.path.join(args.save_path, point, session_ts)
            os.makedirs(point_dir, exist_ok=True)
            print(f"[{point}] Saving {args.duration:.1f}s to: {point_dir}")

            frame_idx = 0
            t0 = time.time()
            next_tick = t0

            while True:
                now = time.time()
                if now - t0 >= args.duration:
                    break

                ok, frame = cap.read()
                if not ok:
                    continue

                img_ts = time.time()
                base = f"{frame_idx:06d}_{img_ts:.6f}"
                img_fn = f"us_{base}.png"
                cv2.imwrite(os.path.join(point_dir, img_fn), frame)

                # 取最新 Polaris，按你的 JSON 风格写文件（每帧一份）
                pol = buf.get_latest()
                # 文件名遵循你示例：pointName_UTC.json（与图像一一对应关系写在 JSON 内）
                json_fn = f"{point}_{utc_filename_ts(img_ts)}.json"
                json_path = os.path.join(point_dir, json_fn)

                save_data = {
                    "point_name": point,
                    "image_file": img_fn,         # 明确对应的图像文件
                    "image_ts": img_ts,           # 图像采集时间（秒）
                }

                if pol is not None:
                    save_data["timestamp_ros"] = pol.get("timestamp_ros", 0.0)
                    # 主体：probe（或你指定的名字）
                    tfmap = pol.get("transforms", {})
                    if args.probe_name in tfmap:
                        save_data[args.probe_name] = tfmap[args.probe_name]
                    # 可选加入 phantom
                    if args.phantom_name in tfmap:
                        save_data[args.phantom_name] = tfmap[args.phantom_name]
                else:
                    # 没有 Polaris 数据时也写一个最小 JSON，便于后续检查缺失
                    save_data["timestamp_ros"] = 0.0

                with open(json_path, "w") as f:
                    json.dump(save_data, f, indent=2)

                frame_idx += 1

                # 简单定率控制
                next_tick += 1.0 / max(args.sample_rate, 1e-6)
                sleep_t = max(0.0, next_tick - time.time())
                if sleep_t > 0:
                    time.sleep(sleep_t)

            print(f"[{point}] Done. Saved {frame_idx} frames.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cap.release()
        stop_polaris()
        print("Closed camera and Polaris subscriber.")

if __name__ == "__main__":
    main()

