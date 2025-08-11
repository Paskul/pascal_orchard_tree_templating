# Pascal Orchard Tree Templating

Pascal's REU edits to the Tree templating repo. A list of changes include:

1. UpdateTrellisPosition service now supports yaw, alongside x,y,z for base coordinate position.
2. An additional `generate_trellis_collision_obj_from_msg.py` is adjacent to `generate_trellis_collision_obj.py`. '...from_msg.py' supports various message types, such as planar coordinate information from SeeTree, visual delta (up/down) estimates, user clicked points (to esimate yaw), as well as custom tilt angles for different trellis types.
3. `generate_trellis_collision_obj_from_msg.py` additionally supports keeping an 'anchor' of our visual-delta estimate, tracking the lowest point and 'pinning' the base to be calculated as sticking to that minimum. This is useful in the case of observing the base of the trunk with no prior knowledge, supporting movement back upwards with a reference to a true tracked base. If then moving upwards from the base, horizontally plannar points (x,z) are recalculated as being 'projected backwards' due to the degree of trellis (set as 18 deg) accumulating difference in true (x,z) as this difference from 'anchor to true' grows. For example, package pascal_odom publishes this delta vertical topic to be used.
4. `generate_trellis_collision_obj.py` supports config paramaters. There is now further support for publishing tree templates in custom frames, currently set as `camera_link` in config. I started a sketch of calculations to try and support differences in frames as best as possible (at least between `camera_link` and `base_link`).

## Below is Marcus' prior README

A ROS 2 package for generating and managing orchard tree templates. This repository provides tools to assist in the modeling, simulation, and templating of orchard trees for robotic motion planning applications.

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
git clone https://github.com/your-org/orchard_tree_templating.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

Source your workspace and launch the main node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run tree_template tree_template
```

### Example

To place the default template in space. The following coordinate corresponds to the base of the template.

```bash
ros2 service call /update_trellis_position tree_template_interfaces/srv/UpdateTrellisPosition "{x: 0.5, y: 0.5, z: 0.5}"
```

The node will then output `Tree position updated to: x=0.5, y=0.5, z=0.5`
