from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            output='screen',
            parameters=[{
                'frame_id': 'uav/odom',  # Frame di riferimento
                'resolution': 0.05,  # Risoluzione della mappa (5cm/voxel)
                'sensor_model.max_range': 5.0,  # Range massimo del sensore
                'filter_ground': False,  # Opzionale: filtra il piano del terreno
            }],
            remappings=[
                ('cloud_in', '/uav/camera/depth/points'),  # Topic di input
            ]
        )
    ])