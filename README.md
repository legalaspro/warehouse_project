# Warehouse Robot Project

ROS2 autonomous warehouse robot system for shelf transportation. The robot navigates to shelves, attaches to them, transports them to shipping areas, and returns to its starting position.

## Features

- **Autonomous Navigation** – Nav2-based path planning with recovery behaviors
- **SLAM Mapping** – Real-time map building using Google Cartographer
- **Shelf Detection** – Laser-based detection of reflective shelf legs
- **Shelf Manipulation** – Lift/lower mechanism for shelf attachment
- **Sim/Real Support** – Seamless switching between simulation and real robot

## Packages

| Package               | Description                                                            |
| --------------------- | ---------------------------------------------------------------------- |
| `nav2_apps`           | Main application: warehouse orchestrator + utility scripts for testing |
| `path_planner_server` | Nav2 controller, planner, behavior server, and BT navigator            |
| `localization_server` | AMCL-based localization with map server                                |
| `map_server`          | Static map serving (simulation and real-world maps)                    |
| `cartographer_slam`   | Google Cartographer SLAM for map creation                              |

## Requirements

- ROS2 Humble (or compatible)
- Nav2 stack
- Cartographer ROS
- Python 3

## Installation

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone <repository-url> warehouse_project

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1. Create a Map (SLAM)

```bash
# Simulation
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=true

# Real robot
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=false
```

### 2. Localization

```bash
# Simulation
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml

# Real robot
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml

# With keepout zones
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_sim.yaml
```

### 3. Path Planning

```bash
# Simulation
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=true

# Real robot
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=false
```

### 4. Run Full Warehouse Mission

Run in **three separate terminals**:

**Real Robot:**

```bash
# Terminal 1: Localization (with keepout zones)
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_real.yaml

# Terminal 2: Path Planner
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=false

# Terminal 3: Mission Script
ros2 run nav2_apps move_shelf_to_ship_real.py
```

**Simulation:**

```bash
# Terminal 1: Localization (with keepout zones)
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_sim.yaml

# Terminal 2: Path Planner
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=true

# Terminal 3: Mission Script
ros2 run nav2_apps move_shelf_to_ship.py
```

> **Note:** Keepout maps include costmap filter zones to prevent the robot from entering restricted areas.

## Warehouse Workflow

1. **Navigate to loading position** – Robot moves to shelf location
2. **Detect and approach shelf** – Uses laser scan to find reflective shelf legs
3. **Position under shelf** – Precise alignment using TF transforms
4. **Lift shelf** – Activates lift mechanism, updates robot footprint
5. **Transport to shipping** – Navigate with shelf attached
6. **Lower shelf** – Releases shelf at destination
7. **Return to init** – Robot returns to starting position

## Configuration

### Maps

Located in `map_server/config/`:

- `warehouse_map_sim.yaml` – Simulation environment
- `warehouse_map_real.yaml` – Real robot environment
- `*_keepout_*` variants include costmap filter zones

### Navigation Parameters

Located in `path_planner_server/config/`:

- `controller_*.yaml` – Velocity controller settings
- `planner_*.yaml` – Global path planner settings
- `recoveries_*.yaml` – Recovery behavior settings
- `bt_navigator_*.yaml` – Behavior tree configuration

### Localization

Located in `localization_server/config/`:

- `amcl_config_sim.yaml` – AMCL parameters for simulation
- `amcl_config_real.yaml` – AMCL parameters for real robot

## Scripts

| Script                       | Purpose                                            |
| ---------------------------- | -------------------------------------------------- |
| `warehouse_orchestrator.py`  | Core orchestrator with full ROS2 parameter support |
| `move_shelf_to_ship.py`      | Configured launcher for simulation                 |
| `move_shelf_to_ship_real.py` | Configured launcher for real robot                 |

### Orchestrator Parameters

The `warehouse_orchestrator.py` is highly configurable via ROS2 parameters. The launcher scripts (`move_shelf_to_ship*.py`) set environment-specific values:

| Parameter Group     | Examples                                                 |
| ------------------- | -------------------------------------------------------- |
| **Frame IDs**       | `odom_frame`, `base_footprint_frame`                     |
| **Waypoints**       | `init_position`, `loading_position`, `shipping_position` |
| **Motion Control**  | `kp_dist`, `kp_yaw`, `v_min`, `v_max`, `w_max`           |
| **Shelf Detection** | `leg_intensity_threshold`, `min_points_per_leg`          |
| **Footprint**       | `footprint_shelf_up`, `footprint_robot_radius`           |

### Utility Scripts (for testing on real robot)

| Script                   | Purpose                              |
| ------------------------ | ------------------------------------ |
| `approacher.py`          | Test shelf detection and positioning |
| `footprint_publisher.py` | Test dynamic footprint updates       |

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
