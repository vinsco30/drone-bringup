from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='image_republish_ktm',
            arguments=['compressedDepth', 'raw'],  # Formato input -> output
            # parameters=[{
            #     'qos_overrides./parameter_events.publisher.reliability': 'reliable',
            # }],
            
            remappings=[
                ('/in/compressedDepth', '/zed/zed_node/rgb/image_rect_color/compressedDepth'), 
                ('/out', '/img_raw_out')
            ]
        )
    ])