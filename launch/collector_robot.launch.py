
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'),
                    'launch'),
                    '/turtlebot3_world.launch.py'
            ])
    ),
    Node(
        package='collection_robot',
        executable='collection_robot_node',
        name='collection_robot_node'
    ),
    DeclareLaunchArgument(
      'rosbag_record',
      default_value = TextSubstitution(text = "False"),
      choices = ['True', 'False'],
      description = "Argument to enable/disable recording of all ros2 topics"
    ),
    ExecuteProcess(
      condition=IfCondition(LaunchConfiguration('rosbag_record')),
      cmd=['ros2', 'bag', 'record', '-a', '-x /camera.+'],
      shell=True
    )
  ])
