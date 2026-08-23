# traffic_map (ROS 2)

OpenDRIVE lane and moving-vehicle marker visualization for ROS 2 and RViz 2.

## Requirements (Ubuntu 22.04 / ROS 2 Humble)

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions libpugixml-dev
```

## Run

```bash
./run.sh
```

The script builds the package, starts the marker node, and opens RViz 2 with the
correct fixed frame and Marker display. ROS 2 does not require `roscore`.

To use another XODR file:

```bash
FILEPATH=/absolute/path/to/map.xodr ./run.sh
```

To select another installed ROS 2 distribution:

```bash
ROS_DISTRO=jazzy ./run.sh
```

## Record data

```bash
ros2 bag record /visualization_marker /camera_info
```
