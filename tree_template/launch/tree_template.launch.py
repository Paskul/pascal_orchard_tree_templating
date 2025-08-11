# tree_template/launch/tree_template.launch.py
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('tree_template')
    params_file = os.path.join(pkg_share, 'config', 'tree_template_config.yaml')
    print(params_file)

    # Node 1: image->trellis updater
    tree_template_base = Node(
        package='tree_template',
        executable='tree_template',  # maps to generate_trellis_collision_obj:main in setup.py
        name='tree_template_base',   # MUST match YAML key
        output='screen',
        parameters=[params_file]
    )

    # Node 2: scene/collision publisher + service
    tree_template_from_ros2 = Node(
        package='tree_template',
        executable='tree_template_from_ros2',  # maps to generate_trellis_collision_obj_from_msg:main
        name='tree_template_from_ros2',        # MUST match YAML key
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        tree_template_from_ros2,
        tree_template_base
    ])
