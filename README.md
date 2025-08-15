# Pascal Orchard Tree Templating

A ROS 2 package for generating and managing orchard tree templates. This repository provides tools to assist in the modeling, simulation, and templating of orchard trees for robotic motion planning applications. **Now with ROS2 topic handling, and yaw support!**

## Features

- Generate parameterized tree templates for orchards as MoveIt collision objects
- Integration with ROS 2 for easy simulation and visualization

## Requirements

- ROS 2 Humble or newer
- Python 3.8+
- Additional dependencies listed in `package.xml` and `requirements.txt`

## Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Paskul/pascal_orchard_tree_templating.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

Source your workspace and launch the two main nodes:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run tree_template tree_template
```

as well as, simultaneously running:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run tree_template tree_template
```

Or, **launch the recommended way via. launch file** to localize the launch and ensure the correct launch of all packages (such as visual up/down estimates, used for a correct trunk base location), done by:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch tree_template tree_template.launch.py 
```

Launching with `tree_template.launch.py` launches both tree templating nodes: `generate_trellis_collision_obj.py` and `generate_trellis_collision_obj_from_msg.py`. In order for templating to work correctly, a visual estimate of up/down movement is needed, leading to reliance on an external package, `pascal_odom`. Knowing this, in our launch, we look for `pascal_odom` and launch its visual estimate node as part of this pipeline. `pascal_odom` is installed through the repository [seeTreeVisualFlowTracking](https://github.com/Paskul/seeTreeVisualFlowTracking). We rely on horizontal (x,y) published the same way, found through repository [trunk_width_estimation](https://github.com/Jostan86/trunk_width_estimation). Parameters set for each node are found in their respective packages `config` folder.


### Parameters for `tree_template_base`
| Parameter | Default |
| ------------- | ------------- |
| `collision_topic`  | collision_object |
| `leader_branch_radii`  | 0.08 |
| `leader_branch_len`  | 2.0 |
| `num_side_branches`  | 4 |
| `side_branch_radii`  | 0.04 |
| `side_branch_len`  | 2.0 |
| `tree_frame_id`  | camera_link |
| `tilt_angle_def_deg`  | 18.435 |
| `initial_position`  | [0.0, 2.0, 0.0] |
| `initial_yaw`  | 0.0 |


### Parameters for `tree_template_from_ros2`
| Parameter | Default |
| ------------- | ------------- |
| `camera_delta_topic`  | /camera_vertical_delta |
| `tree_data_topic`  | /tree_image_data |
| `clicked_point_topic`  | /clicked_point |
| `tilt_angle_deg`  | 18.435 |


## Notes
- In all nodes, having a prior Float32 on Z (up/down) change is required to publish on `camera_delta_topic` to set the proper base location. This is easy to overlook
- There is another dependency on the actual `SeeTree` tree data published on `tree_data_topic` to make estimates about the base on a horizontal plane. We pull (x,y) information from `tree_data_topic`, and together with `camera_delta_topic`, is used to make a full (x,y,z) base point estimation. Again, this is easy to overlook.


## Changes
A generalized list of Pascal's REU edits to the original tree templating repo includes:

1. UpdateTrellisPosition service now supports yaw, alongside x,y,z for base coordinate position.
2. An additional `generate_trellis_collision_obj_from_msg.py` is adjacent to `generate_trellis_collision_obj.py`. '...from_msg.py' supports various message types to support wider use cases, such as supporting coordinate information from SeeTree, visual delta (up/down) estimates, user clicked points (to estimate yaw), as well as custom tilt angles for different trellis types.
3. `generate_trellis_collision_obj_from_msg.py` now supports keeping an 'anchor' of our visual-delta estimate, tracking the lowest point, and 'pinning' the base to be calculated as sticking to that minimum. This is useful in the case of observing the base of the trunk with no prior knowledge, supporting later camera movement back upwards with a reference to a true-tracked base. If then moving upwards from the base, horizontally planar points (x,z) are recalculated as being 'projected backwards' due to the degree of trellis (set as 18 deg), accumulating the difference in true (x,z) as this difference from 'anchor to true' grows, this operation is just some trigonometry. For example, the package pascal_odom publishes this delta vertical topic to be used.
4. `generate_trellis_collision_obj.py` supports config parameters and pulls from `generate_trellis_collision_obj_from_msg.py`. There is now further support for publishing tree templates in custom frames, currently set as `camera_link` in the config. I started a sketch of calculations to try and support differences in frames as best as possible (at least between `camera_link` and `base_link` based on standard axis layouts).

## Todo
There are multiple ways this workflow could be improved:
1. Classification of tree trunks is not differentiated. Posts are counted as trunks and have not been filtered yet
2. Yaw is partially developed. Currently, there is optional functionality to listen to `clicked_point_topic`. In RVIZ2, it's expected for users to manually select to a point left of the trunk, noting it's distance from the center of the trunk. Then, users should select a point on the right of the trunk, equidistant of distance from the same trunk at the same spot, where a calculation of yaw between both points is performed and used.
