#!usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class TrapezoidalMotion(Node):

    def __init__(self):
        super().__init__('trapezoidal_motion')

        # =========================================================
        # publishers and subscriber
        # =========================================================

        self.dt = 0.001

        self.joint_publisher_ = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)
        self.joint_state_subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer_ = self.create_timer(self.dt, self.joint_publisher_callback)

        # ====================================================
        # Joint State Parameters
        # ====================================================

        self.joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6', 'joint7', 'finger_joint1',
                            'finger_joint2']

        self.actual_joint_angles = None
        self.joint_state_initialized = False
        self.joint_state_dict = {}

        # ====================================================
        # Trajectory Generator Parameters
        # ====================================================

        self.declare_parameter("motion", "velocity")
        self.declare_parameter("total_time", 10.0)
        self.declare_parameter("Vmax", 0.5)
        self.declare_parameter("Amax", 1.0)
        self.declare_parameter(
            "final_joint_angles",
            [-1.57, 0.0, 0.0, -0.35, 0.0, 1.57, 1.57]
        )

        self.motion = self.get_parameter("motion").value
        self.total_time = self.get_parameter("total_time").value
        self.Vmax = self.get_parameter("Vmax").value
        self.Amax = self.get_parameter("Amax").value
        self.final_joint_angles = self.get_parameter(
            "final_joint_angles"
        ).value

        # self.motion = "velocity"  # by default

        # self.total_time = 10.0
        # self.Vmax = 0.5
        # self.Amax = 1.0
        # self.final_joint_angles = [-1.57, 0.0, 0.0, -0.35, 0.0, 1.57, 1.57]

        self.traj_time = 0.0

        self.ta = None
        self.tc = None
        self.td = None
        self.Smax = None

        self.initial_joint_angles = None
        # self.joint_limit_min = [-2.9007, -1.83609, -
        #                         2.9007, -3.0770, -2.87630, 0.43982, -3.0508]
        # self.joint_limit_max = [2.9007, 1.83609,
        #                         2.9007, -0.11693, 2.87630, 4.6216, 3.0508]

    def joint_state_callback(self, msg):
        """
        Creates a dictionary that maps joint names with their index as key value pairs
        since joint names are in random order. 

        Args:
            msg (JointState): State interface of each joint position and velocity
        """

        if self.actual_joint_angles is None:
            self.get_logger().warning("Waiting for the current joint angles . . . . ")

        if not self.joint_state_initialized:
            for joint_name in self.joint_names:
                self.joint_state_dict[joint_name] = msg.name.index(joint_name)
            self.joint_state_initialized = True

        self.actual_joint_angles = [
            msg.position[self.joint_state_dict[joint_name]] for joint_name in self.joint_names]

    def motion_parameters(self):
        """Calculate the complete trapezoidal motion profile."""

        q_ini = self.initial_joint_angles
        q_fin = np.array(self.final_joint_angles)

        self.Smax = np.max(np.abs(q_fin - q_ini))

        if self.motion == "time":

            # =========================================
            # Time-based profile
            # =========================================

            ta = 0.2 * self.total_time
            td = ta

            Vmax = self.Smax / (self.total_time - ta)

            Amax = Vmax / ta

            tc = self.total_time - ta - td

            total_time = self.total_time

        elif self.motion == "velocity":

            # =========================================
            # Velocity-based profile
            # =========================================

            S_required = (self.Vmax ** 2) / self.Amax

            if self.Smax >= S_required:

                # Trapezoidal profile

                ta = self.Vmax / self.Amax
                td = ta

                tc = (
                    self.Smax - self.Vmax * ta
                ) / self.Vmax

                Vmax = self.Vmax
                Amax = self.Amax

            else:

                # Triangular profile

                ta = np.sqrt(self.Smax / self.Amax)
                td = ta
                tc = 0.0

                # Actual peak velocity
                Vmax = self.Amax * ta
                Amax = self.Amax

            total_time = ta + tc + td

        else:
            raise ValueError(
                "motion must be either 'time' or 'velocity'"
            )

        return total_time, Vmax, Amax, ta, tc, td, self.Smax

    def trajectory_generator(self):

        q_ini = self.initial_joint_angles
        q_fin = np.array(self.final_joint_angles)

        joint_dist = q_fin - q_ini

        dist_ratio = joint_dist / self.Smax

        if self.traj_time <= self.ta:

            q_ins = 0.5 * self.Amax * self.traj_time ** 2

        elif self.traj_time <= (self.ta + self.tc):

            q_a = 0.5 * self.Vmax * self.ta

            q_ins = q_a + self.Vmax * (
                self.traj_time - self.ta
            )

        elif self.traj_time <= self.total_time:

            time_des = self.traj_time - self.ta - self.tc

            q_a = 0.5 * self.Vmax * self.ta
            q_c = self.Vmax * self.tc

            q_ins = (
                q_a
                + q_c
                + self.Vmax * time_des
                - 0.5 * self.Amax * time_des ** 2
            )

        else:
            print("Trajectory completed .... ")
            q_ins = self.Smax

        q_cmd = q_ini + q_ins * dist_ratio
        self.traj_time += self.dt

        return q_cmd

    def joint_publisher_callback(self):

        if self.actual_joint_angles is None:
            return

        if self.initial_joint_angles is None:
            self.initial_joint_angles = np.array(
                self.actual_joint_angles[0:7])

            self.total_time, self.Vmax, self.Amax, self.ta, self.tc, self.td, self.Smax = self.motion_parameters()

        gripper_cmd = self.actual_joint_angles[7:9]

        q_cmd = self.trajectory_generator()
        msg = Float64MultiArray()
        msg.data = q_cmd.tolist() + gripper_cmd
        self.joint_publisher_.publish(msg=msg)


def main(args=None):

    rclpy.init(args=args)
    node = TrapezoidalMotion()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
