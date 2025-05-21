# Orchard Tree Templating

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