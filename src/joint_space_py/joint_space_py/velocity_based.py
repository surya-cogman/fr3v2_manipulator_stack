import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class VelocityBased(Node):

    def __init__(self):
        super().__init__('velocity_based')

        self.dt = 0.001

        # ====================================================
        # Publisher and Subscriber
        # ====================================================

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

        self.Vmax = 0.5
        self.Amax = 1.0

        self.ta = None
        self.td = self.ta
        self.tc = None

        self.traj_time = 0.0

        self.joint_limit_min = [-2.9007, -1.83609, -
                                2.9007, -3.0770, -2.87630, 0.43982, -3.0508]
        self.joint_limit_max = [2.9007, 1.83609,
                                2.9007, -0.11693, 2.87630, 4.6216, 3.0508]

        self.initial_joint_angles = None

        self.final_joint_angles = [-1.57, 0.0, 0.0, -0.35, 0.0, 1.57, 1.57]

        # ====================================================
        # Trajectory Generator Parameters
        # ====================================================
        #
        self.time_data = []
        self.position_data = []

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

        q_ini = self.initial_joint_angles
        q_fin = np.array(self.final_joint_angles)

        Smax = np.max(np.abs(q_fin - q_ini))

        # Check whether Vmax can be reached
        S_required = (self.Vmax ** 2) / self.Amax

        if Smax >= S_required:
            # =========================
            # Trapezoidal profile
            # =========================

            self.ta = self.Vmax / self.Amax
            self.td = self.ta

            self.tc = (Smax - self.Vmax * self.ta) / self.Vmax

        else:
            # =========================
            # Triangular profile
            # =========================

            self.ta = np.sqrt(Smax / self.Amax)
            self.td = self.ta
            self.tc = 0.0

            # Actual peak velocity
            self.Vmax = self.Amax * self.ta

        total_time = self.ta + self.tc + self.td

        return Smax, total_time

    def trajectory_generator(self):

        Smax, total_time = self.motion_parameters()

        q_ini = self.initial_joint_angles
        q_fin = np.array(self.final_joint_angles)

        joint_dist = q_fin - q_ini

        dist_ratio = joint_dist / Smax

        if self.traj_time <= self.ta:

            q_ins = 0.5 * self.Amax * self.traj_time ** 2

        elif self.traj_time <= (self.ta + self.tc):

            q_a = 0.5 * self.Vmax * self.ta

            q_ins = q_a + self.Vmax * (
                self.traj_time - self.ta
            )

        elif self.traj_time <= total_time:

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
            q_ins = Smax

        q_cmd = q_ini + q_ins * dist_ratio
        self.traj_time += self.dt

        return q_cmd

    def joint_publisher_callback(self):

        if self.actual_joint_angles is None:
            return

        if self.initial_joint_angles is None:
            self.initial_joint_angles = np.array(self.actual_joint_angles[0:7])

        gripper_cmd = self.actual_joint_angles[7:9]

        msg = Float64MultiArray()

        q_cmd = self.trajectory_generator()

        self.time_data.append(self.traj_time)
        self.position_data.append(q_cmd.copy())

        msg.data = q_cmd.tolist() + gripper_cmd
        self.joint_publisher_.publish(msg=msg)

    def plot_results(self):

        time = np.array(self.time_data)

        position = np.array(self.position_data)

        velocity = np.gradient(position, self.dt, axis=0)

        plt.figure()

        for i in range(7):
            plt.plot(time, position[:, i], label=f'Joint{i+1}')

        plt.xlabel('Time (sec)')
        plt.ylabel("Position (rad)")
        plt.title("Joint Position")
        plt.legend()
        plt.grid()

        plt.figure()

        for i in range(7):
            plt.plot(time, velocity[:, i], label=f'Joint{i+1}')

        plt.xlabel('Time (sec)')
        plt.ylabel("Velocity (rad/sec)")
        plt.title("Joint Velocity")
        plt.legend()
        plt.grid()

        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityBased()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass

    node.plot_results()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
