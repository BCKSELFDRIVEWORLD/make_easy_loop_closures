# SLAM Loop Closure Assistant

ROS2 package for guided SLAM mapping with optimal loop closures.

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/BCKSELFDRIVEWORLD/slam_loop_closure_assistant.git
cd .. && colcon build --packages-select slam_loop_closure_assistant
```

## Usage

```bash
ros2 launch slam_loop_closure_assistant loop_closure_assistant.launch.py \
  slam_config_path:=/path/to/mapper_params.yaml
```

## RViz

Add `/loop_closure_assistant/markers` (MarkerArray)

## Topics

| Topic | Type |
|-------|------|
| `/loop_closure_assistant/markers` | MarkerArray |
| `/loop_closure_assistant/status` | String |

## License

MIT - (c) 2026 BCK
