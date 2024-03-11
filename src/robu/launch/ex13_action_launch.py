from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Setze die ROS_DOMAIN_ID Systemumgebungsvariable auf deine Katalognummer
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', '19')
    os.environ['ROS_DOMAIN_ID'] = ros_domain_id

    return LaunchDescription([
        # Starte den fibonacci_server Node
        Node(
            package='robu',
            executable='fibonacci_server',
            name='fibonacci_action_server'
        ),

        # Starte den fibonacci_client Node
        Node(
            package='robu',
            executable='fibonacci_client',
            name='fibonacci_action_client'
        ),

        # Warte 5 Sekunden
        ExecuteProcess(
            cmd=['sleep', '5'],
            output='screen',
            on_exit=[ # Ausführung der CLI-Anweisung zum Aufruf der Action fibonacci mit dem Goal order=5
                ExecuteProcess(
                cmd=['ros2', 'action', 'send_goal', 'fibonacci', 'robu_interfaces/action/Fibonacci', 'order:=5'],
                output='screen'
                )]
        ),
       
    ])