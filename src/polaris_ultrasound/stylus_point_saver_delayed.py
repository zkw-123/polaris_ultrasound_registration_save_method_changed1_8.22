#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import datetime
import numpy as np
from std_msgs.msg import String
import threading
from datetime import timezone
import time

class StylusPointSaverDelayed(Node):
    def __init__(self):
        super().__init__('stylus_point_saver_delayed')

        # 参数声明
        self.declare_parameter('save_path', '/ros2_ws/calibration_data/polaris/stylus')
        self.declare_parameter('stylus_name', 'stylus')
        self.declare_parameter('phantom_name', 'phantom')  # 可选保存
        self.declare_parameter('delay_time', 3.0)  # 默认延迟 3 秒

        self.save_path = self.get_parameter('save_path').value
        self.stylus_name = self.get_parameter('stylus_name').value
        self.phantom_name = self.get_parameter('phantom_name').value
        self.delay_time = float(self.get_parameter('delay_time').value)

        os.makedirs(self.save_path, exist_ok=True)

        self.latest_transforms = {}  # tool_name → {'matrix': ..., 'translation': ..., 'quality': ...}
        self.latest_timestamp = 0.0

        # 订阅 Polaris 数据
        self.subscription = self.create_subscription(String, 'ndi_transforms', self.ndi_callback, 10)

        self.get_logger().info(
            f"StylusPointSaverDelayed initialized. "
            f"Type point name and press Enter to save after {self.delay_time:.1f} seconds."
        )

        # 启动输入监听线程
        threading.Thread(target=self.input_loop, daemon=True).start()

    def ndi_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_timestamp = float(data.get('timestamp', 0.0))
            self.latest_transforms.clear()

            for t in data.get('transforms', []):
                tool = t.get('tool_name', '')
                self.latest_transforms[tool] = {
                    'matrix': t.get('matrix', []),
                    'translation': t.get('translation', []),
                    'quality': t.get('quality', 0.0)
                }
        except Exception as e:
            self.get_logger().error(f"Failed to parse Polaris message: {e}")

    def input_loop(self):
        while rclpy.ok():
            try:
                point_name = input("Enter point name (e.g., point1): ").strip()
            except EOFError:
                break
            if point_name.lower() == 'exit':
                print("Exiting input loop.")
                break
            if not point_name:
                print("Point name cannot be empty.")
                continue

            # 等待指定时间
            self.get_logger().info(f"Waiting {self.delay_time:.1f} seconds before saving '{point_name}'...")
            time.sleep(self.delay_time)

            # 保存
            self.save_point_data(point_name)

    def _format_time_for_filename(self, ts_sec: float):
        """将 epoch 秒转为 UTC 格式的文件名字符串"""
        if ts_sec <= 0:
            now = datetime.datetime.now(timezone.utc)
        else:
            now = datetime.datetime.fromtimestamp(ts_sec, tz=timezone.utc)
        return now.strftime("%Y-%m-%dT%H-%M-%S.%fZ")

    def save_point_data(self, point_name):
        # 检查 stylus 是否存在
        if self.stylus_name not in self.latest_transforms:
            self.get_logger().warn("Stylus data not available.")
            return

        # 检查数据有效性
        matrix = self.latest_transforms[self.stylus_name]["matrix"]
        mat_np = np.array(matrix, dtype=float)

        if np.isnan(mat_np).any():
            self.get_logger().error("Stylus pose invalid (NaN detected).")
            return
        if np.allclose(mat_np, 0):
            self.get_logger().error("Stylus pose invalid (all zeros).")
            return

        # 保存 JSON
        timestamp_str = self._format_time_for_filename(self.latest_timestamp)
        filename = f"{point_name}_{timestamp_str}.json"
        filepath = os.path.join(self.save_path, filename)

        save_data = {
            "point_name": point_name,
            "timestamp_ros": self.latest_timestamp,
            "stylus": self.latest_transforms[self.stylus_name]
        }
        if self.phantom_name in self.latest_transforms:
            save_data["phantom"] = self.latest_transforms[self.phantom_name]

        with open(filepath, 'w') as f:
            json.dump(save_data, f, indent=2)

        self.get_logger().info(f"Saved point '{point_name}' to {filepath}")


def main(args=None):
    rclpy.init(args=args)
    node = StylusPointSaverDelayed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

