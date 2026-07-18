#!usr/bin/env python3

import pinocchio as pin
from ament_index_python.packages import get_package_share_directory


class FR3V2ROBOT:
    def __init__(self, urdf_path: str):

        self.urdf_path = urdf_path
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        franka_description_path = get_package_share_directory('franka_description')

        self.visual_model = pin.buildGeomFromUrdf(
            self.model,
            self.urdf_path,
            pin.GeometryType.VISUAL,
            franka_description_path
        )

        self.visual_data = pin.GeometryData(self.visual_model)

    def get_model(self):
        return self.model

    def get_data(self):
        return self.data
    
    def get_visual_model(self):
        return self.visual_model

    def get_visual_data(self):
        return self.visual_data
