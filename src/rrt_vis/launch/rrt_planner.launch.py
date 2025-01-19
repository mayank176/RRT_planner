from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2'
    )


    rviz_config_path = os.path.expanduser('~/.rviz2/rrt_vis_conf.rviz')

    return LaunchDescription([
        rviz_arg,
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config_path]
        ),
        
        Node(
            package='mpc',
            executable='mpc_node',
            name='mpc_node'
        ),
        
        Node(
            package='rrt_vis',
            executable='rrt_vis_node',
            name='rrt_vis_node'
        )
    ])
