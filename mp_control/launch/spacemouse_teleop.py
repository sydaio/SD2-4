from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

   # spacenav_node = Node(
   #     package='spacenav',
   #     executable='spacenav_node',
   #     name='spacenav'
   # )

    twist_scaler_node = Node(
        package='twist_scaler',
        executable='twist_scaler',
        name='twist_scaler',

        parameters=[
            {'force_scale': 1.0},
            {'torque_scale': 1.0},
            {'frame_id': 'base_link'},
            {'input_twist_topic': '/spacenav/twist'},
            {'output_twist_topic': '/cmd_vel'},
            {'use_sim_time': False}
        ]
    )

    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='twist_relay',
        arguments=['/cmd_vel', '/mecanum_drive_controller/reference']
    )

    return LaunchDescription([
        #spacenav_node,
        twist_scaler_node, 
        relay_node
    ])