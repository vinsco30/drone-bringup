import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():

    return LaunchDescription([
        # dummy static transformation from ENU frames to NED frames
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments=["0", "0", "0", "1.57", "0", "-3.14", "map", "map_ned"]
        ),
        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/model/x500_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"],
            output = "screen"
        ),
        launch_ros.actions.Node(
            package = "gz_drone_bringup",
            executable = "odom_republisher_simu",
            output = "screen"
        )
    ])