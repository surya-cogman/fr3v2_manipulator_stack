#!usr/bin/env python3

import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TrapezoidalMotion(Node):
    """_summary_

    Args:
        Node (_type_): _description_
    """

    def __init__(self):

        super().__init__('time_based')

        self.dt = 0.001

        # ===================================================================
        # ros publisher and subscriber
        # ===================================================================

        self.publisher_ = self.create_publisher(
            Float64MultiArray, 'position_controller/commands', 10)
        self.subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.joint_subscriber, 10)
        self.timer_ = self.create_timer(self.dt, self.joint_publisher)

        # =======================================================================
        # Joint state parameters
        # =======================================================================

        self.joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6', 'joint7', 'finger_joint1', 'finger_joint2']

        self.arm_joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'joint7'
        ]

        self.gripper_joint_names = [
            'finger_joint1',
            'finger_joint2'
        ]

        self.actual_joint_angles = None
        self.joint_state_initialized = False
        self.joint_state_dict = {}

        # ========================================================================
        # Trapezoidal Trajectory parameters
        # ========================================================================

        self.joint_limit_min = [-2.9007, -1.83609, -
                                2.9007, -3.0770, -2.87630, 0.43982, -3.0508]
        self.joint_limit_max = [2.9007, 1.83609,
                                2.9007, -0.11693, 2.87630, 4.6216, 3.0508]
        self.total_traj_time = 10
        self.ta = 0.2 * self.total_traj_time
        self.tc = 0.6 * self.total_traj_time
        self.td = self.ta

        self.traj_time = 0.0
        self.trajectory_initial_angles = None
        self.final_joint_angles = [0.0, 0.0, 0.0, -0.35, 0.0, 1.57, 1.57]

        # ========================================================================
        # Matplotlib parameters
        # ========================================================================

        self.time_data = []
        self.position_data = []

    def joint_subscriber(self, msg):
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

    def compute_profile_limits(self):
        """ This function takes the total time duration(T) of motion as input in sec

            T = ta + tc + td
            ta = ascend time
            td = descend time
            tc = cruise time 
            In trapezoidal profile ta = td

            T = 2 * ta + tc  
            The area under velocity profile gives Total distance covered

            S = 0.5 * ta * Vmax + Vmax * tc + 0.5 * td * Vmax
            S = Vmax*(tc + ta)
            S = Vmax * (T - 2*ta + ta)
            S = Vmax * (T - ta)

            Vmax = S / (T - ta)

            Now for maximum acceleration during ascend phase

            Vmax = Amax * ta

            Amax = Vmax / ta
        Returns:
            Vmax and Amax for the trapezoidal velocity profile

        """

        # During the entire trajectory only 60% time it is in constant velocity phase(cruise)
        # ascend and descend  each 20%

        q_ini = self.trajectory_initial_angles  # arm joint angles

        # q_ini = np.array(self.arm_joint_names)
        q_fin = np.array(self.final_joint_angles)

        Smax = np.max(np.abs(q_fin - q_ini))

        Vmax = Smax/(self.total_traj_time - self.ta)

        Amax = Vmax / self.ta

        return Vmax, Amax, Smax

    def trajectory_generator(self):
        """Generates joint motion profile and returns the joint angle at each step
        """

        Vmax, Amax, Smax = self.compute_profile_limits()

        q_ini = self.trajectory_initial_angles
        q_fin = np.array(self.final_joint_angles)

        total_distance = q_fin - q_ini

        distance_ratio = total_distance/Smax

        # during acceleration phase

        if self.traj_time <= self.ta:
            q_ins = 0.5 * Amax * (self.traj_time ** 2)

        elif self.traj_time > self.ta and self.traj_time <= (self.ta + self.tc):

            q_a = 0.5 * Vmax * self.ta

            q_ins = q_a + Vmax * (self.traj_time - self.ta)

        elif self.traj_time > (self.ta + self.tc) and self.traj_time <= (self.total_traj_time):

            time_des = self.traj_time - self.ta - self.tc
            qc = 0.5 * Vmax * self.ta + Vmax * (self.tc)

            q_ins = qc + Vmax * (time_des) - 0.5 * Amax * (time_des ** 2)

        else:
            print("Trajectory time completed .....")
            q_ins = Smax

        q_cmd = q_ini + q_ins * distance_ratio
        self.traj_time += self.dt

        return q_cmd

    def joint_publisher(self):
        """_summary_
        """

        if self.actual_joint_angles is None:
            return

        if self.trajectory_initial_angles is None:
            self.trajectory_initial_angles = np.array(
                self.actual_joint_angles[0:7])

        q_cmd = self.trajectory_generator()

        self.time_data.append(self.traj_time)
        self.position_data.append(q_cmd.copy())
        gripper_cmd = self.actual_joint_angles[7:9]

        msg = Float64MultiArray()
        msg.data = q_cmd.tolist() + gripper_cmd
        self.publisher_.publish(msg=msg)
        self.get_logger().info(f"Command: {msg.data}")

    def plot_results(self):

        time = np.array(self.time_data)
        position = np.array(self.position_data)

        velocity = np.gradient(position, self.dt, axis=0)
        acceleration = np.gradient(velocity, self.dt, axis=0)

        # Position
        plt.figure()

        for i in range(7):
            plt.plot(time, position[:, i], label=f'Joint {i+1}')

        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.title("Joint Position")
        plt.legend()
        plt.grid()

        # Velocity
        plt.figure()

        for i in range(7):
            plt.plot(time, velocity[:, i], label=f'Joint {i+1}')

        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.title("Joint Velocity")
        plt.legend()
        plt.grid()

        # Acceleration
        plt.figure()

        for i in range(7):
            plt.plot(time, acceleration[:, i], label=f'Joint {i+1}')

        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (rad/s²)")
        plt.title("Joint Acceleration")
        plt.legend()
        plt.grid()

        plt.show()


def main(args=None):
    """_summary_

    Args:
        args (_type_, optional): _description_. Defaults to None.
    """

    rclpy.init(args=args)
    node = TrapezoidalMotion()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("ROS2 stopped")

    print("Calling plot_results()")

    node.plot_results()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
