from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('rrt_vis')
    config_file = os.path.join(pkg_dir, 'config', 'rrt_params.yaml')
    
    print(f"Loading config from: {config_file}")
    with open(config_file, 'r') as f:
        print("Config file contents:")
        print(f.read())


    print(f"Loading config from: {config_file}")
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2'
    )

    rviz_config_path = os.path.expanduser('~/Documents/RRT_planner/src/rrt_vis/rviz/rrt_vis_conf.rviz')

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
            name='quadrotor_controller',
            parameters=[config_file]
        ),
        
        Node(
            package='rrt_vis',
            executable='rrt_vis_node',
            name='path_visualizer',
            parameters =[config_file]
        )
    ])
