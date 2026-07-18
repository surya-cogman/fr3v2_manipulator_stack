#!/usr/bin/env python3

import pinocchio as pin
import numpy as np


class FR3V2Kinematics:
    """_summary_"""

    def __init__(self, model, data):

        self.model = model
        self.data = data

    def forward_kinematics(self, q):
        """
        Compute forward kinematics for a given joint configuration.
        """

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def get_frame_pose(self, frame_name):
        """
        Returns the SE3 pose of the requested frame.
        """

        frame_id = self.model.getFrameId(frame_name)

        return self.data.oMf[frame_id]

    def get_frame_jacobian(self, q, frame_name):

        self.forward_kinematics(q)

        frame_id = self.model.getFrameId(frame_name)

        J = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        return J
    
    def manipulability(self, q, frame_name):

        J = self.get_frame_jacobian(q, frame_name)

        # Translational Jacobian
        Jv = J[:3, :]

        return np.sqrt(np.linalg.det(Jv @ Jv.T))
