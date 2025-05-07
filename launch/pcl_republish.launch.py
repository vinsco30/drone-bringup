from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_cloud_transport',
            executable='republish',
            name='pcl_republish_ktm',
            parameters=[
                {'in_transport': 'draco'},
                {'out_transport': 'raw'}
            ],
            remappings=[
                ('/in/draco', '/zed/zed_node/point_cloud/cloud_registered/draco'),
                ('/out', '/pcl_raw_out'),
            ]
        )
    ])


