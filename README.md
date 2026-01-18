# Make Easy Loop Closures (MELC)

A ROS2 package that guides users during SLAM mapping to achieve optimal loop closures. Works with SLAM Toolbox.

![Screencast from 01-13-2026 01-03-47 PM](https://github.com/user-attachments/assets/4832a9f4-99cb-4200-ba40-3580a8e1f912)



## How It Works

MELC monitors the SLAM Toolbox pose graph during mapping. When enough nodes are added without closing a loop, it shows a **red target area** indicating where you should return to create a loop closure.

## Step-by-step: How MELC Guides Loop Closures

### 1) Red target appears (loop suggestion)
When enough nodes are added to the SLAM graph without a loop closure, MELC selects the best target area (based on distance / density / distribution / feature scoring) and shows a red target zone.
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
MELC resets the node counter and continues monitoring the SLAM graph, waiting for the next loop closure opportunity.

<img width="2470" height="746" alt="Screenshot from 2026-01-14 09-21-11" src="https://github.com/user-attachments/assets/34f39c3e-c21c-49c1-974c-90a7c84cd402" />

### Algorithm

1. **SLAM Graph Integration**: Uses SLAM Toolbox's pose graph directly (no separate path tracking)
2. **Automatic Configuration**: Fetches `loop_search_maximum_distance` from SLAM Toolbox parameters
3. **K-D Tree Optimization**: O(n log n) spatial queries using scipy's K-D Tree (falls back to brute force if unavailable)
4. **Edge-Based Validation**: Monitors pose graph edges to confirm loop closure with proximity checks

### Target Selection Scoring

The red target area is chosen based on four factors:

```
score = distance_score * 0.2 + density_score * 0.3 + distribution_score * 0.2 + feature_score * 0.3
```

| Factor | Weight | Description |
|--------|--------|-------------|
| **Distance** | 20% | Closer targets preferred (easier to reach) |
| **Density** | 30% | Optimal node count in target area |
| **Distribution** | 20% | More filled sectors = better (6 sectors, 60° each) |
| **Feature** | 30% | Edge-rich areas preferred (more constraints) |

## Parameters

All parameters are configured in `config/params.yaml`:

```yaml
melc:
  ros__parameters:
    slam_toolbox_node: "slam_toolbox"  # SLAM Toolbox node name
    min_graph_nodes: 20                # Min nodes before searching
    nodes_between_loops: 50            # Nodes between loop suggestions
    min_edge_increase: 3               # Edge increase to confirm loop
    edge_proximity_threshold: 1.5      # Edge proximity check (m)
    min_area_density: -1               # -1 = auto, positive = manual
    max_area_density: -1               # -1 = auto, positive = manual
    density_k_min: 1.0                 # K coefficient for min density
    density_k_max: 11.0                # K coefficient for max density
    min_sectors: 4                     # Minimum filled sectors (out of 6)
```

### Parameter Details

| Parameter | Default | Description |
|-----------|---------|-------------|
| `slam_toolbox_node` | `"slam_toolbox"` | SLAM Toolbox node name (fetches `loop_search_maximum_distance` from it) |
| `min_graph_nodes` | 20 | Minimum graph nodes before searching for loop targets |
| `nodes_between_loops` | 50 | Number of nodes that must be added between loop suggestions |
| `min_edge_increase` | 3 | Required new edges to confirm loop closure |
| `edge_proximity_threshold` | 1.5m | How close edge endpoints must be to robot/target nodes |
| `min_area_density` | -1 | Minimum nodes in target area (-1 = auto-calculate) |
| `max_area_density` | -1 | Maximum nodes in target area (-1 = auto-calculate) |
| `density_k_min` | 1.0 | K coefficient for auto min density: `k * radius²` |
| `density_k_max` | 11.0 | K coefficient for auto max density: `k * radius²` |
| `min_sectors` | 4 | Minimum sectors with nodes (out of 6) for valid target |

### Auto-Calculated Density

When `min_area_density` and `max_area_density` are set to `-1`, density limits are auto-calculated:

```
min_density = density_k_min * radius²
max_density = density_k_max * radius²
```

**Example with radius=3.0m (from SLAM Toolbox):**
- `k_min=1.0` → min_density = 1.0 × 9 = 9 nodes
- `k_max=11.0` → max_density = 11.0 × 9 = 99 nodes

### Sector Distribution Check

Target area is divided into 6 sectors (60° each). Areas at map corners/edges are filtered out:

```
     Sector 1
       /\
      /  \
Sec 6      Sec 2
    /  ○  \
   /      \
Sec 5      Sec 3
      \/
   Sector 4
```

- `min_sectors: 6` = only accept center areas (strictest)
- `min_sectors: 4` = accept moderately distributed areas (default)
- `min_sectors: 3` = accept corner/edge areas (most permissive)

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
- Topic: `/loop_closure_assistant/markers`

### Visualization

| Color | Meaning |
|-------|---------|
| **Red cylinder** | Loop closure target - go here! |
| **Yellow cylinder** | Target reached, waiting for loop closure |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/loop_closure_assistant/markers` | `MarkerArray` | Visualization markers |
| `/loop_closure_assistant/status` | `String` | Status text |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `Odometry` | Triggers TF lookup |
| `/slam_toolbox/graph_visualization` | `MarkerArray` | Edge count monitoring |

## Requirements

- ROS2 Humble
- SLAM Toolbox
- TF2
- `python3-scipy` (optional, for K-D Tree optimization - falls back to brute force if unavailable)

## License

MIT - (c) 2026 BCK

GitHub: [BCKSELFDRIVEWORLD](https://github.com/BCKSELFDRIVEWORLD)
