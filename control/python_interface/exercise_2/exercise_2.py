#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Drone Controller Node.

This module implements a ROS2 node for controlling a drone using the
Aerostack2 framework. It handles arming, offboard mode, control mode
setting, and position commands.
"""

from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from drone_course_msgs.srv import RequestPath
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# PID gains (same for x, y and z)
KP = 1.0
KI = 0.02
KD = 0.0

# limits
MAX_SPEED_XY = 1.5  # m/s
MAX_SPEED_Z = 1.0   # m/s
MAX_INTEGRAL = 1.0  # so the integral does not grow too much


def clamp(value, maximum):
    return max(-maximum, min(maximum, value))


class DroneController(Node):
    """Drone Controller class for managing drone operations.

    This class handles subscribing to drone state information,
    publishing motion commands, and calling services to control the
    drone's operational state.
    """

    def __init__(self):
        """Initialize the DroneController node.

        Sets up subscribers, publishers, service clients, and the main
        control timer.
        """
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
        # timer in its own group, so run() never runs twice at the same time
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # Subscribers

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone0/self_localization/pose',
            self.state_pose_callback,
            best_effort_qos,
        )

        # Publishers

        self.twist_pub = self.create_publisher(
            TwistStamped, '/drone0/motion_reference/twist', reliable_qos
        )

        # Service clients

        self.control_mode_client = self.create_client(
            SetControlMode, '/drone0/controller/set_control_mode', callback_group=self.callback_group
        )

        # I check if this service is ready inside run(), like in exercise 1.2
        self.path_request_client = self.create_client(
            RequestPath, '/request_path', callback_group=self.callback_group
        )

        # Class variables
        self.state_pose: PoseStamped = PoseStamped()
        self.pose_received: bool = False
        self.control_mode_set: bool = False
        self.path: list = []
        self.path_idx: int = 0

        # PID memory for x, y, z
        self.integral = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.prev_time = None

        # Timer at 100Hz (0.01 seconds)
        self.timer = self.create_timer(0.01, self.run, callback_group=self.timer_group)

        self.get_logger().info('Drone Controller Node initialized')

    def state_pose_callback(self, msg):
        """Callback for state pose messages."""
        self.state_pose = msg
        self.pose_received = True

    def reset_pid(self):
        # called when we go to a new waypoint, old values are not valid anymore
        self.integral = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.prev_time = None

    def publish_speed(self, vx, vy, vz):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'earth'
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_pub.publish(twist_msg)

    def run(self):
        """Main control loop (100 Hz)."""

        # Request path from service
        if not len(self.path) > 0:
            if not self.path_request_client.service_is_ready():
                self.get_logger().warn('Path service is not available', throttle_duration_sec=1.0)
                return
            req = RequestPath.Request()
            response: RequestPath.Response = self.path_request_client.call(req)
            self.path = list(response.path)
            if len(self.path) == 0:
                return
            for i, p in enumerate(self.path):
                self.get_logger().info(f'point {i}: ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})')

        # Check if control mode is set, if not, call the service to set it
        if not self.control_mode_set:
            self.get_logger().info('Calling control mode service...')

            if not self.control_mode_client.service_is_ready():
                self.get_logger().warn('Control mode service is not available')
                return

            request = SetControlMode.Request()
            control_mode: ControlMode = ControlMode()
            control_mode.control_mode = ControlMode.SPEED
            control_mode.yaw_mode = ControlMode.YAW_SPEED
            control_mode.reference_frame = ControlMode.LOCAL_ENU_FRAME
            request.control_mode = control_mode

            response: SetControlMode.Response = self.control_mode_client.call(request)
            if not response.success:
                self.get_logger().warn('Failed to set control mode')
                return
            self.get_logger().info('Control mode set successfully')
            self.control_mode_set = True

        # wait for the first pose, otherwise the error is computed from (0,0,0)
        if not self.pose_received:
            return

        # Desired position reference
        position_ref = self.path[self.path_idx]
        position_ref = (position_ref.x, position_ref.y, position_ref.z)

        # Read current drone state
        current_x = self.state_pose.pose.position.x
        current_y = self.state_pose.pose.position.y
        current_z = self.state_pose.pose.position.z

        position_error_x = position_ref[0] - current_x
        position_error_y = position_ref[1] - current_y
        position_error_z = position_ref[2] - current_z
        position_error = (position_error_x**2 + position_error_y**2 + position_error_z**2) ** 0.5

        # PID speed control
        # dt should be around 0.01 s but I measure it to be sure
        now = self.get_clock().now().nanoseconds * 1e-9
        first_step = self.prev_time is None
        if first_step:
            dt = 0.01
        else:
            dt = max(now - self.prev_time, 1e-3)
        self.prev_time = now

        errors = [position_error_x, position_error_y, position_error_z]
        commands = [0.0, 0.0, 0.0]
        for i in range(3):
            e = errors[i]

            p_term = KP * e

            # integral with a limit (anti-windup)
            self.integral[i] = clamp(self.integral[i] + e * dt, MAX_INTEGRAL)
            i_term = KI * self.integral[i]

            # no derivative in the first step, there is no previous error yet
            # (without this the first command is very big)
            if first_step:
                d_term = 0.0
            else:
                d_term = KD * (e - self.prev_error[i]) / dt
            self.prev_error[i] = e

            commands[i] = p_term + i_term + d_term

        # limit the speed
        velocity_command_x = clamp(commands[0], MAX_SPEED_XY)
        velocity_command_y = clamp(commands[1], MAX_SPEED_XY)
        velocity_command_z = clamp(commands[2], MAX_SPEED_Z)

        self.publish_speed(velocity_command_x, velocity_command_y, velocity_command_z)

        if position_error < 0.2:
            self.path_idx = (self.path_idx + 1) % len(self.path)
            p = self.path[self.path_idx]
            self.get_logger().info(f'Waypoint reached, next: ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})')
            self.reset_pid()


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
        # in speed mode the drone keeps the last speed, so send zero before closing
        try:
            for _ in range(10):
                drone_controller.publish_speed(0.0, 0.0, 0.0)
        except Exception:
            pass
        drone_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()