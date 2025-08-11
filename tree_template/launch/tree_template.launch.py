# tree_template/launch/tree_template.launch.py
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_template = get_package_share_directory('tree_template')
    pkg_share_visual = get_package_share_directory('pascal_odom')
    params_file_template = os.path.join(pkg_share_template, 'config', 'tree_template_config.yaml')
    params_file_visual = os.path.join(pkg_share_visual, 'config', 'visual_config.yaml')

    # visual flow for up/down
    visual_flow = Node(
        package='pascal_odom',
        executable='visual_z_estimate',
        name='visual_flow',
        output='screen',
        parameters=[params_file_visual]
    )

    # image->trellis updater
    tree_template_base = Node(
        package='tree_template',
        executable='tree_template',  # maps to generate_trellis_collision_obj:main in setup.py
        name='tree_template_base',   # MUST match YAML key
        output='screen',
        parameters=[params_file_template]
    )

    # scene/collision publisher + service
    tree_template_from_ros2 = Node(
        package='tree_template',
        executable='tree_template_from_ros2',  # maps to generate_trellis_collision_obj_from_msg:main
        name='tree_template_from_ros2',        # MUST match YAML key
        output='screen',
        parameters=[params_file_template]
    )

    return LaunchDescription([
        visual_flow,
        tree_template_from_ros2,
        tree_template_base
    ])
