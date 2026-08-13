import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_path = get_package_share_directory("fr3_control")

    yaml_file = os.path.join(package_path, 'config',
                             'trapezoidal_motion.yaml')

    motion_node = Node(
        package="joint_space_py",
        executable="trapezoidal_motion",
        name="trapezoidal_motion",
        parameters=[yaml_file]
    )
    return LaunchDescription(
        [motion_node]
    )
