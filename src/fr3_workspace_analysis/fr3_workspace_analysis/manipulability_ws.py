#!/usr/bin/env python3

import pinocchio as pin
import numpy as np

from robot import FR3V2ROBOT
from kinematics import FR3V2Kinematics
from sampling import JointSampler
from visualizer import WorkspaceVisualizer


def main():

    urdf_path = "/home/surya-cogman/ws/fr3_manipulation_stack/src/franka_description/final_urdf/fr3v2/fr3v2.urdf"

    num_samples = 50000

    robot = FR3V2ROBOT(urdf_path)

    model = robot.get_model()
    data = robot.get_data()

    kin = FR3V2Kinematics(model, data)
    sampler = JointSampler(model)

    samples = sampler.sample(num_samples)

    workspace_points = []
    manipulability_values = []

    print("Generating Manipulability Workspace...\n")

    for i, q in enumerate(samples):

        kin.forward_kinematics(q)

        pose = kin.get_frame_pose("hand_tcp")

        w = kin.manipulability(q, "hand_tcp")

        workspace_points.append(pose.translation.copy())
        manipulability_values.append(w)

        if (i + 1) % 5000 == 0:
            print(f"{i+1}/{num_samples} completed")

    workspace_points = np.array(workspace_points)
    manipulability_values = np.array(manipulability_values)

    print("\nManipulability Statistics")
    print("------------------------------")
    print(f"Minimum : {manipulability_values.min():.6f}")
    print(f"Maximum : {manipulability_values.max():.6f}")
    print(f"Average : {manipulability_values.mean():.6f}")

    # Random robot pose for visualization
    q_robot = sampler.random_configuration()

    kin.forward_kinematics(q_robot)

    pin.updateGeometryPlacements(
        model,
        data,
        robot.get_visual_model(),
        robot.get_visual_data()
    )

    viewer = WorkspaceVisualizer()

    viewer.add_robot(
        robot.get_visual_model(),
        robot.get_visual_data()
    )

    viewer.add_manipulability_workspace(
        workspace_points,
        manipulability_values
    )

    viewer.adding_title("fr3v2 manipulability workspace",18)

    viewer.show()


if __name__ == "__main__":
    main()