
# ROS2 Turtlebot - Walker

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

### Overview
A simple walker algorithm much where the robot will move forward until it reaches an obstacle (but not colliding), then rotate in place until the way ahead is clear, then move forward again and repeat.

### Dependencies
<ul>
  <li>Ubuntu 20.04</li>
  <li>ROS2 Foxy</li>
  <li>Gazebo</li>
  <li>Turtlebot3 Package</li>
</ul>

Set TURTLEBOT3_MODEL environment variable as,
```
  echo  "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

### Build and compile instructions
```
    cd "your_colcon_workspace"/src
    git clone https://github.com/tanujthakkar/turtlebot-walker
    cd ..
    colcon build
    source "your_colcon_workspace"/install/setup.bash
```

### Running Walker
Running walker,
```
  ros2 launch turtlebot_walker walker.py
```

With rosbag recording on,
```
  ros2 launch turtlebot_walker walker.py record:=True
```

Inspect rosbag using,
```
  ros2 bag info path_to_bag_directory
```
