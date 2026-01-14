# Make Easy Loop Closures (MELC)

A ROS2 package that guides users during SLAM mapping to achieve optimal loop closures. Works with SLAM Toolbox.

![Screencast from 01-13-2026 01-03-47 PM](https://github.com/user-attachments/assets/4832a9f4-99cb-4200-ba40-3580a8e1f912)



## How It Works

MELC monitors your robot's path during SLAM mapping. When you've traveled far enough without closing a loop, it shows a **red target area** indicating where you should return to create a loop closure.

## Step-by-step: How MELC Guides Loop Closures

### 1) Red target appears (loop suggestion)
When the robot travels long enough without a loop closure, MELC selects the best historical area (based on age / distance / density scoring) and shows a red target zone.
Goal: go back into this zone to trigger a loop closure.

<img width="2497" height="745" alt="Screenshot from 2026-01-14 09-19-30" src="https://github.com/user-attachments/assets/4ebab325-bc4f-44c9-9b6b-ba4b08b46d1e" />

### 2) Entering the red zone (attempting loop closure)
As soon as the robot enters the red target radius, MELC switches to “target reached” mode and starts actively watching the SLAM graph for new constraints.

<img width="2470" height="746" alt="Screenshot from 2026-01-14 09-20-56" src="https://github.com/user-attachments/assets/b0b25aa9-0df2-40ea-866f-d6f532c21dc3" />

### 3) Edge increase detected (loop closure confirmed)
If SLAM Toolbox adds enough new edges (configurable via min_edge_increase), MELC confirms that the loop closure is happening.
At this stage, the target becomes yellow, indicating “stay here / keep scanning” while edges are being added.

<img width="2470" height="746" alt="Screenshot from 2026-01-14 09-21-07" src="https://github.com/user-attachments/assets/1315af29-b801-4158-be4d-c1f52c8e726d" />

### 4) Back to the Wild
Once the loop closure is confirmed and MELC determines that sufficient constraints have been added, the yellow target disappears and the robot is released back into free exploration.
MELC resets the distance counter and continues monitoring the trajectory, waiting for the next loop closure opportunity.

<img width="2470" height="746" alt="Screenshot from 2026-01-14 09-21-11" src="https://github.com/user-attachments/assets/34f39c3e-c21c-49c1-974c-90a7c84cd402" />

### Algorithm

1. **Path Tracking**: Records robot position in map frame using TF
2. **Distance Monitoring**: Tracks total distance traveled since last loop closure
3. **Target Selection**: When `loop_closure_distance` is reached, finds the best target from path history
4. **Loop Detection**: Monitors pose graph edges to confirm loop closure

### Target Selection Scoring

The red target area is chosen based on three factors:

```
score = age_score * 0.3 + distance_score * 0.3 + density_score * 0.4
```

| Factor | Weight | Description |
|--------|--------|-------------|
| **Age Score** | 30% | Older poses preferred (better for loop closure quality) |
| **Distance Score** | 30% | Closer targets preferred (easier to reach) |
| **Density Score** | 40% | Areas visited multiple times preferred (more edges) |

## Parameters

All parameters are configured in `config/params.yaml`:

```yaml
melc:
  ros__parameters:
    minimum_travel_distance: 0.5    # Path point recording interval (m)
    loop_search_max_distance: 3.0   # Target radius - red circle size (m)
    loop_search_dimension: 8.0      # Max search distance for targets (m)
    loop_chain_size: 10             # Min poses before suggesting loop
    min_edge_increase: 3            # Edge increase to confirm loop closure
    min_pose_age: 40                # Skip recent poses as targets
```

### Parameter Details

| Parameter | Default | Effect on Target Area |
|-----------|---------|----------------------|
| `minimum_travel_distance` | 0.5m | Path resolution. Lower = more points, smoother path |
| `loop_search_max_distance` | 3.0m | **Red circle radius**. Robot must enter this area |
| `loop_search_dimension` | 8.0m | Max distance to search for loop targets |
| `loop_chain_size` | 10 | Min path points before loop suggestion. `min_path_points = loop_chain_size * 2` |
| `min_edge_increase` | 3 | Required new edges to confirm loop closure |
| `min_pose_age` | 40 | Ignores last N path points. Prevents targeting recent poses |

### Derived Values

```python
loop_closure_distance = loop_search_dimension * 3  # Distance before suggesting loop
loop_closure_radius = loop_search_max_distance     # Red circle radius
min_path_points = loop_chain_size * 2              # Min points to start
```

**Example with defaults:**
- Loop suggestion after: `8.0 * 3 = 24m` traveled
- Red circle radius: `3.0m`
- Ignores last `40` path points as targets

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/BCKSELFDRIVEWORLD/make_easy_loop_closures.git
cd .. && colcon build --packages-select make_easy_loop_closures
source install/setup.bash
```

## Usage

### Standalone
```bash
ros2 launch make_easy_loop_closures melc.launch.py
```

### With SLAM Toolbox
```python
# In your launch file
from ament_index_python.packages import get_package_share_directory

melc_params = os.path.join(
    get_package_share_directory('make_easy_loop_closures'), 'config', 'params.yaml'
)

Node(
    package='make_easy_loop_closures',
    executable='loop_closure_node.py',
    name='melc',
    parameters=[melc_params, {'use_sim_time': True}]
)
```




## RViz Setup

Add MarkerArray display:
- Topic: `/melc/markers`

### Visualization

| Color | Meaning |
|-------|---------|
| **Green path** | Robot's traveled path |
| **Red cylinder** | Loop closure target - go here! |
| **Yellow cylinder** | Target reached, waiting for loop closure |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/melc/markers` | `MarkerArray` | Visualization markers |
| `/melc/status` | `String` | Status text |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `Odometry` | Triggers TF lookup |
| `/slam_toolbox/graph_visualization` | `MarkerArray` | Edge count monitoring |

## Requirements

- ROS2 Humble
- SLAM Toolbox
- TF2

## License

MIT - (c) 2026 BCK

GitHub: [BCKSELFDRIVEWORLD](https://github.com/BCKSELFDRIVEWORLD)
