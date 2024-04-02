from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnviromentVariable , DeclareLaunchArgumnet
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
  mypackage = get_package_share_directory('robu')

  envar_domain_id = SetEnviromentVariable(name='ROS_DOMAIN_ID', value='19')

  node_action_server = Node(
      package='robu',
      executable='fibonacci_server',
      emulate_tty=True,           #optional
      output='screen'             #optional
  )

  node_action_client = Node(
      package='robu',
      executable='fibonacci_client',
      emulate_tty=True,           #optional
      output='screen'             #optional
  )

  exec_action = ExecuteProcess(
      cmd=['sleep', '5'],
      output='screen',
      on_exit=[ExecuteProcess(
        cmd=['ros2','action', 'send_goal', '--feedback', 'fibonacci', 'robu_interfaces/action/Fibonacci', '{order: 10}'],
        output='screen'
  )]
  )


  ld =LaunchDescription()
  ld.add_action(envar_domain_id)
  ld.add_action(node_action_client)
  ld.add_action(node_action_server)
  ld.add_action(exec_action)

  return ld