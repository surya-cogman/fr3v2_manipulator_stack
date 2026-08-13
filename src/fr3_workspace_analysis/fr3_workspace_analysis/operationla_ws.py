#!/usr/bin/env python3

import pinocchio as pin
import numpy as np

from robot import FR3V2ROBOT
from kinematics import FR3V2Kinematics
from sampling import JointSampler
from visualizer import WorkspaceVisualizer


def main():

    # -------------------------------------------------------
    # URDF
    # -------------------------------------------------------
    urdf_path = "/home/surya-cogman/ws/fr3_manipulation_stack/src/franka_description/final_urdf/fr3v2/fr3v2.urdf"

    num_samples = 50000

    # -------------------------------------------------------
    # Manipulability Threshold
    # -------------------------------------------------------
    threshold = 0.1

    # -------------------------------------------------------
    # Robot
    # -------------------------------------------------------
    robot = FR3V2ROBOT(urdf_path)

    model = robot.get_model()
    data = robot.get_data()

    kin = FR3V2Kinematics(model, data)
    sampler = JointSampler(model)

    samples = sampler.sample(num_samples)

    operational_points = []

    print("Generating Operational Workspace...\n")

    for i, q in enumerate(samples):

        kin.forward_kinematics(q)

        pose = kin.get_frame_pose("hand_tcp")

        w = kin.manipulability(q, "hand_tcp")

        if w >= threshold:

            operational_points.append(
                pose.translation.copy()
            )

        if (i + 1) % 5000 == 0:

            print(f"{i+1}/{num_samples} completed")

    operational_points = np.array(operational_points)

    print("\nFinished")
    print("----------------------------")
    print("Operational Points :", len(operational_points))

    # -------------------------------------------------------
    # Bounding Box
    # -------------------------------------------------------

    xmin = operational_points[:, 0].min()
    xmax = operational_points[:, 0].max()

    ymin = operational_points[:, 1].min()
    ymax = operational_points[:, 1].max()

    zmin = operational_points[:, 2].min()
    zmax = operational_points[:, 2].max()

    length = xmax - xmin
    width = ymax - ymin
    height = zmax - zmin

    volume = length * width * height

    print("\nBounding Box")
    print("----------------------------")

    print(f"Length : {length:.3f} m")
    print(f"Width  : {width:.3f} m")
    print(f"Height : {height:.3f} m")

    print(f"Volume : {volume:.4f} m³")

    # -------------------------------------------------------
    # Robot for visualization
    # -------------------------------------------------------

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

    viewer.add_operational_workspace(
        operational_points
    )

    viewer.add_bounding_box(
        xmin,
        xmax,
        ymin,
        ymax,
        zmin,
        zmax
    )

    viewer.show()


if __name__ == "__main__":

    main()