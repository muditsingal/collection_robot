import os
import random

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_block(i):
    model_path = os.path.join(get_package_share_directory('collection_robot'), 'models')
    x_pos = -2.5 -(i*2.0) -random.uniform(0, 0.2)
    y_pos = random.uniform(-0.1, 0.1)

    return Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'trash_block{}'.format(i),
              '-file', model_path + '/trash_block/model.sdf',
              '-x', str(x_pos),
              '-y', str(y_pos),
              '-z', '0.0',
              '-Y', '0.0'],
              output='screen'
    )

def generate_launch_description():

    trash_cnt = 3
    blocks = []
    for i in range(trash_cnt):
        blocks.append(generate_block(i))

    return LaunchDescription(blocks)
