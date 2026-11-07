#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
# (BSD-3-Clause license, see repository)
"""Drone Controller Node.

This module implements a ROS2 node for controlling a drone using the
Aerostack2 framework. It handles arming, offboard mode, control mode
setting, and position commands.
"""

from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from drone_course_msgs.msg import Point
from drone_course_msgs.srv import RequestPath
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class DroneController(Node):
    """Drone Controller class for managing drone operations."""

    def __init__(self):
        super().__init__('drone_controller')
        # QoS profile for reliable topics
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        # QoS profile for best effort topics
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        # Callback group for handling synchronous service calls from timer
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        # ★ TODO 1:
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone0/self_localization/pose',
            self.state_pose_callback,
            best_effort_qos,
        )

        # Publishers
        # ★ TODO 2
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/drone0/motion_reference/pose',
            reliable_qos,
        )

        # Class variables
        self.state_pose: PoseStamped = PoseStamped()
        self.control_mode_set: bool = False
        self.path: list[Point] = [Point(x=4.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=5.0, z=2.1),
                                  Point(x=4.0, y=5.0, z=2.1)]
        self.path_idx: int = 0

        # Timer at 10Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.run, callback_group=self.callback_group)

        self.get_logger().info('Drone Controller Node initialized')

    def state_pose_callback(self, msg):
        """Callback for state pose messages."""
        self.state_pose = msg

    def run(self):
        """Main control loop."""

        # Desired position reference
        position_ref = self.path[self.path_idx]

        # Generate motion reference command
        position_msg = PoseStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'earth'
        position_msg.pose.position.x = position_ref.x
        position_msg.pose.position.y = position_ref.y
        position_msg.pose.position.z = position_ref.z
        position_msg.pose.orientation.w = 1.0  # Neutral orientation
        position_msg.pose.orientation.x = 0.0
        position_msg.pose.orientation.y = 0.0
        position_msg.pose.orientation.z = 0.0

        # Read current drone state
        current_x = self.state_pose.pose.position.x
        current_y = self.state_pose.pose.position.y
        current_z = self.state_pose.pose.position.z

        position_error_x = position_ref.x - current_x
        position_error_y = position_ref.y - current_y
        position_error_z = position_ref.z - current_z
        position_error = (position_error_x**2 + position_error_y**2 + position_error_z**2) ** 0.5

        # Publish motion reference command
        # ★ TODO 3: hedefi yayınla
        self.pose_pub.publish(position_msg)

        if position_error < 0.2:
            # ★ TODO 4: sıradaki noktaya geç, sonuncudaysa orada kal
            if self.path_idx < len(self.path) - 1:
                self.path_idx += 1
                self.get_logger().info(
                    f'Waypoint reached, going to point {self.path_idx}: '
                    f'({self.path[self.path_idx].x}, {self.path[self.path_idx].y}, '
                    f'{self.path[self.path_idx].z})'
                )
            else:
                self.get_logger().info('Circuit completed', throttle_duration_sec=2.0)


def main(args=None):
    """Main entry point for the drone controller node."""
    rclpy.init(args=args)
    drone_controller = DroneController()

    # Use MultiThreadedExecutor to allow synchronous service calls from timer
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(drone_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()