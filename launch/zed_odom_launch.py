import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import sys
import pathlib
import os
import yaml
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():

    # pkg_path = get_package_share_directory( 'gz_drone_bringup' )
    # yaml_file = os.path.join( pkg_path, 'config', 'classic_cfg.yaml' )
    # with open(yaml_file, 'r') as file:
    #     config = yaml.load(file, Loader=yaml.FullLoader)
    #     params = config['odom_republisher_simu']['ros__parameters'] 

    zed_cam_launch_path = os.path.join(
        get_package_share_directory('gz_drone_bringup'),
        'launch',
        'zed_camera.launch.py'
    )


    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_cam_launch_path),
            launch_arguments={'camera_model': 'zedm', 'publish_map_tf': 'false' }.items()
        ),

        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0","zed_camera_link", "base_link"]
        ),

        launch_ros.actions.Node(
            package = "gz_drone_bringup",
            executable = "odom_republisher_zed",
            name = "odom_republisher_zed",
            # parameters=[yaml_file],
            output = "screen"
        )
    ])