import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([
     
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

  
        Node(
            package='assignment1_rt',
            executable='turtle_spawn',
            name='turtle_spawn',
            output='screen'
        ),

       
        Node(
            package='assignment1_rt',
            executable='distance_monitor',
            name='distance_monitor',
            output='screen'
        ),

        
        Node(
            package='assignment1_rt',
            executable='ui_node',
            name='ui_node',
            output='screen',
            prefix='gnome-terminal --wait --',
            on_exit=Shutdown()
        ),
    ])
