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
    yaml_file = os.path.join( pkg_path, 'config', 'flamingo_cfg.yaml' )
    with open(yaml_file, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
        params = config['odom_republisher_simu']['ros__parameters'] 

    return LaunchDescription([
        # dummy static transformation from ENU frames to NED frames
        # launch_ros.actions.Node(
        #     package = "tf2_ros",
        #     executable = "static_transform_publisher",
        #     arguments=["0", "0", "0", "1.57", "0", "-3.14", params['prefix_tf']+"/map", params['prefix_tf']+"/map_ned"]
        # ),
        # launch_ros.actions.Node(
        #     package = "tf2_ros",
        #     executable = "static_transform_publisher",
        #     arguments=["0.12", "0.03", "0", "0", "0", "0", params['prefix_tf']+"/base_link", params['prefix_tf']+"/camera_link"]
        # ),

        #x500_depth_0/OakD-Lite/base_link/StereoOV7251 ---- x500_depth_0/OakD-Lite/base_link/StereoOV7251 ---- x500_depth_0/OakD-Lite/base_link/IMX214

        # launch_ros.actions.Node(
        #     package = "tf2_ros",
        #     executable = "static_transform_publisher",
        #     arguments=["0", "0", "0", "0", "0", "0", params['prefix_tf']+"/camera_link", "x500_depth_0/OakD-Lite/base_link/IMX214"]
        # ),

        # launch_ros.actions.Node(
        #     package = "tf2_ros",
        #     executable = "static_transform_publisher",
        #     arguments=["0", "0", "0", "0", "0", "0", params['prefix_tf']+"/camera_link", "x500_depth_0/OakD-Lite/base_link/StereoOV7251"]
        # ),

        # launch_ros.actions.Node(
        #     package = "tf2_ros",
        #     executable = "static_transform_publisher",
        #     arguments=["-9.5", "-3.5", "0.0", "0", "0", "0", params['prefix_tf']+"/odom", params['prefix_tf']+"/odom_center"]
        # ),

        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
            parameters=[{"qos_overrides./clock.publisher.reliability": 'best_effort'  }],
            output = "screen"
        ),

        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/model/x500_flamingo_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"],
            parameters=[{"qos_overrides./model/x500_flamingo_0/odometry.publisher.reliability": 'best_effort'  }],
            output = "screen"
        ),

        launch_ros.actions.Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = ["/world/default/model/x500_flamingo_0/joint/link2_link3_joint/sensor/force_torque_sensor/forcetorque@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrench"],
            remappings = [("/world/default/model/x500_flamingo_0/joint/link2_link3_joint/sensor/force_torque_sensor/forcetorque", params['prefix_tf']+"/sensor/force_torque")],
            parameters=[{"qos_overrides./flamingo/sensor/force_torque.publisher.reliability": 'best_effort'  }],
            output = "screen"
        ),

        # launch_ros.actions.Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments = ["/camera@sensor_msgs/msg/Image@gz.msgs.Image"],
        #     remappings = [("/camera", params['prefix_tf']+"/camera/image_raw")],
        #     # parameters=[{"qos_overrides./uav/camera/image_raw.publisher.reliability": 'best_effort'  }],
        #     output = "screen"
        # ),
        # launch_ros.actions.Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments = ["/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image"],
        #     remappings = [("/depth_camera", params['prefix_tf']+"/camera/depth/image_raw")],
        #     parameters=[{"qos_overrides./flamingo/camera/depth/image_raw.publisher.reliability": 'best_effort'  }],
        #     output = "screen"
        # ),
        # launch_ros.actions.Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments = ["/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
        #     remappings = [("/camera_info", params['prefix_tf']+"/camera/camera_info")],
        #     parameters=[{"qos_overrides./flamingo/camera/camera_info.publisher.reliability": 'best_effort'  }],
        #     output = "screen"
        # ),
        # launch_ros.actions.Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments = ["/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"],
        #     remappings = [("/depth_camera/points", params['prefix_tf']+"/camera/depth/points")],
        #     output = "screen",
        #     parameters=[{'use_sim_time': True}]
        # ),
        launch_ros.actions.Node(
            package = "gz_drone_bringup",
            executable = "odom_republisher_simu",
            name = "odom_republisher_simu",
            parameters=[yaml_file, {'use_sim_time': True}],
            output = "screen",
        )
    ])