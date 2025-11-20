from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bartender_robot',
            executable='bartender_robot',
            name='bartender_robot',
            output='screen',
            parameters=[{
                'device_name': '/dev/ttyUSB0',
                'baudrate': 115200,
                'publish_rate': 50.0,
                'joints': [
                    'XL_430_base_Revolute-29',
                    'XL_430_shoulder_Revolute-30',
                    'XL_430_upper_Revolute-31',
                    'XL_430_lower_Revolute-32'
                ],
                'dynamixel_ids': [1, 2, 3, 4]  # Dynamixel IDs matching the joints above
            }]
        )
    ])
