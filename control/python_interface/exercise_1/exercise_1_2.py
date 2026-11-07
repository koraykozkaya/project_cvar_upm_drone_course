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

        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone0/self_localization/pose',
            self.state_pose_callback,
            best_effort_qos,
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, '/drone0/motion_reference/pose', reliable_qos
        )

        # Service clients

        # Control Mode Service Client
        self.control_mode_client = self.create_client(
            SetControlMode, '/drone0/controller/set_control_mode', callback_group=self.callback_group
        )

        # Path Service Client
        # ★ TODO 1
        self.path_request_client = self.create_client(
            RequestPath, '/request_path', callback_group=self.callback_group
        )

        # Class variables
        self.state_pose: PoseStamped = PoseStamped()
        self.control_mode_set: bool = False
        self.path: list[Point] = []
        self.path_idx: int = 0

        # Timer at 10Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.run, callback_group=self.callback_group)

        self.get_logger().info('Drone Controller Node initialized')

    def state_pose_callback(self, msg):
        """Callback for state pose messages."""
        self.state_pose = msg

    def run(self):
        """Main control loop."""
        # ★ TODO 2
        if not len(self.path) > 0:
           
            if not self.path_request_client.service_is_ready():
                self.get_logger().warn('Path service is not available', throttle_duration_sec=1.0)
                return

            self.get_logger().info('Requesting path from service...')
            req = RequestPath.Request()  
            response: RequestPath.Response = self.path_request_client.call(req)
            self.path = list(response.path)

            if len(self.path) == 0:
                self.get_logger().warn('Received an empty path')
                return
            for i, p in enumerate(self.path):
                self.get_logger().info(f'  point {i}: ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})')

        # Check if control mode is set, if not, call the service to set it
        if not self.control_mode_set:
            self.get_logger().info('Calling control mode service...')

            # Check if service is available
            if not self.control_mode_client.service_is_ready():
                self.get_logger().warn('Control mode service is not available')
                return

            # Generate control mode service request
            request = SetControlMode.Request()
            control_mode: ControlMode = ControlMode()
            control_mode.control_mode = ControlMode.POSITION
            control_mode.yaw_mode = ControlMode.YAW_ANGLE
            control_mode.reference_frame = ControlMode.LOCAL_ENU_FRAME
            request.control_mode = control_mode

            response: SetControlMode.Response = self.control_mode_client.call(request)
            success = response.success
            if not success:
                self.get_logger().warn('Failed to set control mode')
                return
            self.get_logger().info('Control mode set successfully')
            self.control_mode_set = True

        # Desired position reference
        if len(self.path) > 0:
            position_ref = self.path[self.path_idx]
        else:
            position_ref = Point(x=0.0, y=0.0, z=1.0)

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
        self.pose_pub.publish(position_msg)

        if position_error < 0.2:
            self.get_logger().info(
                'Drone has reached the target position', throttle_duration_sec=1.0
            )

            self.path_idx = (self.path_idx + 1) % len(self.path)


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