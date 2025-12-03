from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

    
        Node(
            package='assignment1_rt',
            executable='turtle_spawn',
            name='turtle_spawn',
            prefix='gnome-terminal --',
            output='screen'
        ),

        Node(
            package='assignment1_rt',
            executable='distance_monitor',
            name='distance_monitor',
            prefix='gnome-terminal --',
            output='screen'
        ),

        
        Node(
            package='assignment1_rt',
            executable='ui_node',
            name='ui_node',
            prefix='gnome-terminal --',
            output='screen'
        ),
    ])
