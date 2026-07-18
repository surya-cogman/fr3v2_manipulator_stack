#!/usr/bin/env python3

import os
import numpy as np
import pinocchio as pin

from robot import FR3V2ROBOT
from kinematics import FR3V2Kinematics
from sampling import JointSampler
from visualizer import WorkspaceVisualizer


def main():

    # ------------------------------------------------------------------
    # URDF Path
    # ------------------------------------------------------------------
    urdf_path = "/home/surya-cogman/ws/fr3_manipulation_stack/src/franka_description/final_urdf/fr3v2/fr3v2.urdf"      # <-- Change this

    # ------------------------------------------------------------------
    # Number of Samples
    # ------------------------------------------------------------------
    num_samples = 50000

    # ------------------------------------------------------------------
    # Initialize Robot
    # ------------------------------------------------------------------
    robot = FR3V2ROBOT(urdf_path)

    model = robot.get_model()
    data = robot.get_data()

    kin = FR3V2Kinematics(model, data)
    sampler = JointSampler(model)

    # ------------------------------------------------------------------
    # Generate Random Joint Configurations
    # ------------------------------------------------------------------
    samples = sampler.sample(num_samples)

    # ------------------------------------------------------------------
    # Compute Reachable Workspace
    # ------------------------------------------------------------------
    workspace_points = []

    print("Generating Reachable Workspace...\n")

    for i, q in enumerate(samples):

        kin.forward_kinematics(q)

        pose = kin.get_frame_pose("hand_tcp")

        workspace_points.append(pose.translation.copy())

        if (i + 1) % 5000 == 0:
            print(f"{i + 1}/{num_samples} samples completed")

    workspace_points = np.array(workspace_points)

        # ------------------------------------------------------------
    # Reach Statistics
    # ------------------------------------------------------------
    distances = np.linalg.norm(workspace_points, axis=1)

    min_reach = np.min(distances)
    max_reach = np.max(distances)
    min_index = np.argmin(distances)
    max_index = np.argmax(distances)

    print(f"Minimum Point : {workspace_points[min_index]}")
    print(f"Maximum Point : {workspace_points[max_index]}")

    print("\nReach Statistics")
    print("----------------------------")
    print(f"Minimum Reach : {min_reach:.4f} m")
    print(f"Maximum Reach : {max_reach:.4f} m")

    print("\nFinished!")
    print("Workspace Shape :", workspace_points.shape)

    # ------------------------------------------------------------
    # Robot pose for visualization
    # ------------------------------------------------------------
    q_robot = sampler.random_configuration()

    kin.forward_kinematics(q_robot)

    pin.updateGeometryPlacements(
        model,
        data,
        robot.get_visual_model(),
        robot.get_visual_data()
    )

    # ------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------
    visualizer = WorkspaceVisualizer()

    visualizer.add_robot(
        robot.get_visual_model(),
        robot.get_visual_data()
    )

    visualizer.plot_workspace(workspace_points)
    visualizer.adding_title("FR3 Reachable Workspace",22)

    visualizer.show()

   

if __name__ == "__main__":
    main()