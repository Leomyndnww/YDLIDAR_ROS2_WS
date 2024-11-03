#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LidarObjectDistanceAngle(Node):
    def __init__(self):
        super().__init__('lidar_object_distance_angle')

        # 手动设置 QoSProfile，指定 history 和 depth
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,  # 保留最后 depth 个消息
            depth=10,  # 设置深度为 10
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile  # 使用定义的 QoS 配置
        )
        self.get_logger().info("Lidar object distance-angle listener has been started.")

    def scan_callback(self, msg):
        object_distances_angles = []  # 用于存储物体的距离和角度

        # 遍历激光雷达的每个扫描点
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:  # 过滤掉超出范围的数据
                # 计算当前角度
                angle = msg.angle_min + i * msg.angle_increment
                # 将距离和角度加入列表
                object_distances_angles.append((distance, math.degrees(angle)))  # 将角度转换为度数

        # 打印每一个物体的距离和角度信息，每行一个
        for dist, ang in object_distances_angles:
            self.get_logger().info(f"(Distance: {dist:.2f} m, Angle: {ang:.2f}°)")

def main(args=None):
    rclpy.init(args=args)
    lidar_distance_angle = LidarObjectDistanceAngle()

    try:
        rclpy.spin(lidar_distance_angle)
    except KeyboardInterrupt:
        pass

    lidar_distance_angle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
