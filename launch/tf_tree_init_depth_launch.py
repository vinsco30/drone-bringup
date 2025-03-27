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

    pkg_path = get_package_share_directory( 'gz_drone_bringup' )
    yaml_file = os.path.join( pkg_path, 'config', 'depth_cfg.yaml' )
    with open(yaml_file, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
        params = config['odom_republisher_simu']['ros__parameters'] 

    return LaunchDescription([
        # dummy static transformation from ENU frames to NED frames
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments=["0", "0", "0", "1.57", "0", "-3.14", params['prefix_tf']+"/map", params['prefix_tf']+"/map_ned"]
        ),
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments=["0.12", "0.03", "0", "0", "0", "0", params['prefix_tf']+"/base_link", params['prefix_tf']+"/camera_link"]
        ),
        #x500_depth_0/OakD-Lite/base_link/StereoOV7251 ---- x500_depth_0/OakD-Lite/base_link/StereoOV7251 ---- x500_depth_0/OakD-Lite/base_link/IMX214

        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/model/x500_depth_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"],
            output = "screen"
        ),
        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/camera@sensor_msgs/msg/Image@gz.msgs.Image"],
            remappings = [("/camera", params['prefix_tf']+"/camera/image_raw")],
            output = "screen"
        ),
        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image"],
            remappings = [("/depth_camera", params['prefix_tf']+"/camera/depth/image_raw")],
            output = "screen"
        ),
        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
            remappings = [("/camera_info", params['prefix_tf']+"/camera/camera_info")],
            output = "screen"
        ),
        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"],
            remappings = [("/depth_camera/points", params['prefix_tf']+"/camera/depth/points")],
            output = "screen"
        ),
        launch_ros.actions.Node(
            package = "gz_drone_bringup",
            executable = "odom_republisher_simu",
            name = "odom_republisher_simu",
            parameters=['/root/ros2_ws/src/gz_drone_bringup/config/depth_cfg.yaml'],
            output = "screen"
        )
    ])