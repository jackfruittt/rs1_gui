# Horizon Hive – Group 7 (Robotics Studio 1)

This repository is part of the **Horizon Hive** project for *Robotics Studio 1*.  
The system aims to control and monitor a **drone swarm** that can help maintain and protect a forest environment.

---

## Project Overview

- **Core Goal:**  
  Develop a swarm of drones capable of monitoring a forest environment, detecting hazards, and supporting forest maintenance.

- **Mission Control GUI:**  
  This repository contains the **mission control interface** with:
  - Swarm monitoring (states, positions, telemetry)
  - Live video streams from drones via ROS2 camera topics
  - A live map visualisation with drone overlays
  - Incident reporting and control panels
  - Centralised ROS2 integration for real-time data

- **Technology:**  
  - **ROS 2** integration via centralized `RosHandler` class
  - GUI built with **Pygame** for responsive real-time display
  - Threaded architecture to keep GUI responsive while handling ROS2 communications
  - Modular design for easy extension to additional ROS2 topics

---

## Architecture

### Core Components

- **`ros_handler.py`**: Centralised ROS2 handler that manages all ROS topics in a single thread
  - Thread-safe data access for GUI components
  - Automatic topic discovery and subscription management (Done when GUI is launched)
  - Extensible design for additional ROS2 message types
  - Handles camera feeds, telemetry, and other sensor data
  - Optimised subscription management for performance

- **`camera_ros.py`**: Camera display component with configurable switching modes
  - Displays live camera feeds from ROS2 Image topics
  - **Three switching modes** for different use cases:
    - **Instant switching** (`instant_switching=True, preload_all=True`): Zero-delay switching, all cameras preloaded
    - **Selective preload** (`instant_switching=True, preload_all=False`): Fast switching with selective loading
    - **Subscribe-unsubscribe** (`instant_switching=False`): Minimal resource usage, slower switching
  - Support for multiple camera switching (press 'C' to cycle)
  - Automatic discovery of drone camera topics
  - Fallback to placeholder when no feed available

- **`ui.py`**: Main GUI application
  - Integrates all components with shared RosHandler instance
  - Responsive pygame-based interface
  - Proper cleanup and thread management

## Architecture Benefits
- **Responsive GUI**: ROS2 runs in separate thread, GUI never freezes
- **Centraliaed management**: Single point for all ROS2 communications
- **Configurable performance**: Multiple camera switching modes for different requirements
- **Easy extension**: Add new topics without modifying existing code
- **Thread safety**: All data access is properly synchronized
- **Optimised subscriptions**: Intelligent topic management reduces resource usage
- **Graceful degradation**: Works with or without ROS2 environment
- **Manual configuration**: Can bypass auto-discovery for faster startup

### ROS2 Integration

The system uses a **centralised threading model**:
1. Single `RosHandler` thread manages all ROS2 communications
2. GUI components access data through thread-safe methods
3. No ROS2 calls from the main GUI thread (keeps interface responsive)
4. Easy to extend for additional topics (telemetry, navigation, etc.)

### Camera Configuration

The camera system supports multiple switching modes for different performance requirements:

#### Switching Modes

1. **Instant Switching (Recommended)**
   ```python
   camera_component = CameraComponent(ros_handler, 
                                     instant_switching=True, 
                                     preload_all=True)
   ```
   - **Use case**: Real-time monitoring, user interaction
   - **Behavior**: All cameras preloaded, zero-delay switching
   - **Resource usage**: Higher (all streams active)

2. **Selective Preload**
   ```python
   camera_component = CameraComponent(ros_handler, 
                                     instant_switching=True, 
                                     preload_all=False)
   ```
   - **Use case**: Balanced performance and resource usage
   - **Behavior**: Fast switching with selective loading
   - **Resource usage**: Medium (selective streams)

3. **Subscribe-Unsubscribe**
   ```python
   camera_component = CameraComponent(ros_handler, 
                                     instant_switching=False)
   ```
   - **Use case**: Resource-constrained environments, minimal bandwidth
   - **Behavior**: Subscribe to one camera at a time
   - **Resource usage**: Minimal (single stream)

---

## Current State

- ✅ **ROS 2 integrated** via centralised RosHandler
- ✅ **Live camera feeds** from ROS2 topics working
- ✅ **Thread-safe architecture** implemented
- 🔄 **Under development**: Additional telemetry integration
- 🔄 **Under development**: Map integration with ROS2 positioning data

---

## Usage

### Prerequisites

1. **Install dependencies:**
   ```bash
   pip install pygame opencv-python==4.10.0.84 "numpy<2.0"
   
   # For ROS2 integration:
   pip install rclpy cv-bridge
   ```

2. **Ensure ROS2 environment is sourced** (if using real ROS2 topics):
   ```bash
   source /opt/ros/humble/setup.bash  # or your ROS2 distro
   ```

### Running the Application

1. **Run the main GUI:**
   ```bash
   python3 ui.py
   ```

### Controls

- **Camera switching**: Press `C` to cycle through available camera topics
- **Robot selection**: Click on robots in the left panel
- **Map interaction**: Click on the map to select drones
- **Popup dialogs**: Click buttons to interact with system features

# Adding New ROS2 Topic Types - Implementation Guide

To add support for new ROS2 topic types (like odometry, GPS, IMU, etc.), follow this systematic pattern used throughout this ROS handler.

## Step 1: Add ROS2 Message Type Imports

Add the necessary imports at the top of the file:

**Example for odometry:**
```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
```

**Example for GPS:**
```python
from sensor_msgs.msg import NavSatFix
```

**Example for IMU:**
```python
from sensor_msgs.msg import Imu
```

## Step 2: Add Data Storage in `__init__`

Add a dictionary to store the new data type in the `RosHandler.__init__` method:

```python
self.odometry_data = {}     # For nav_msgs/Odometry
self.gps_data = {}          # For sensor_msgs/NavSatFix  
self.imu_data = {}          # For sensor_msgs/Imu
```

## Step 3: Update Topic Discovery (Optional)

If using automatic topic discovery, update `_discover_topics()` method:

```python
elif 'nav_msgs/msg/Odometry' in topic_types:
    odometry_topics.append(topic_name)
elif 'sensor_msgs/msg/NavSatFix' in topic_types:
    gps_topics.append(topic_name)
```

## Step 4: Create Data-Specific Subscription Methods

Follow the same pattern as camera methods. Create these methods in `RosHandler`:

### A) Subscribe method:
```python
def subscribe_to_odometry_topic(self, topic_name: str) -> bool:
    if not self.node or not self.ros2_available:
        return False
    try:
        self.node.subscribe_to_odometry(topic_name)
        print(f"Subscribed to odometry topic: {topic_name}")
        return True
    except Exception as e:
        print(f"Failed to subscribe to {topic_name}: {e}")
        return False
```

### B) Unsubscribe method (optional for persistent data like odometry):
```python
def unsubscribe_from_odometry_topic(self, topic_name: str) -> bool:
    # Similar pattern to unsubscribe_from_camera_topic()
    pass
```

### C) Data retrieval method:
```python
def get_latest_odometry(self, topic_name: str) -> Optional[Dict[str, Any]]:
    with self.data_lock:
        return self.odometry_data.get(topic_name, {}).copy() if topic_name in self.odometry_data else None
```

### D) Convenience methods (optional but recommended):
```python
def get_drone_pose(self, drone_id: int) -> Optional[Dict[str, float]]:
    topic_name = f"/rs1_drone_{drone_id}/odom"
    odom_data = self.get_latest_odometry(topic_name)
    if not odom_data:
        return None
    return odom_data.get('pose', None)
```

### E) Available topics method:
```python
def get_available_odometry_topics(self) -> list:
    with self.data_lock:
        return self.available_odometry_topics.copy()
```

## Step 5: Create Message Handler Method

Create a message handler method that processes incoming ROS2 messages:

```python
def _handle_odometry_message(self, topic_name: str, msg):
    # Add message handler logic
```

## Step 6: Add RosNode Subscription Methods

In the `RosNode` class, add subscription management methods:

### A) Add subscription storage in `RosNode.__init__`:
```python
self.odometry_subscriptions = {}  # topic_name -> subscription object
```

### B) Add subscribe method:
```python
def subscribe_to_odometry(self, topic_name: str):
    # Add subscribe method logic
    )
    
    self.odometry_subscriptions[topic_name] = subscription
    self.get_logger().info(f'Subscribed to odometry topic: {topic_name}')
```

### C) Add unsubscribe method:
```python
def unsubscribe_from_odometry(self, topic_name: str):
    # Add unsubscribe logic
```

## Step 7: Update Status and Utility Methods

Update existing methods to include your new data type:

### A) Update `get_status_info()`:
```python
'odometry_topics_count': len(self.available_odometry_topics),
'active_odometry_subscriptions': len(self.odometry_data) if self.odometry_data else 0
```

### B) Update `is_topic_active()` to check your new data type:
```python
odom_info = self.get_latest_odometry(topic_name)
if odom_info:
    return time.time() - odom_info.get('timestamp', 0) < 2.0
```

## Step 8: Integration Example

**Expected topic naming convention:** `/rs1_drone_{drone_id}/odom`, `/rs1_drone_{drone_id}/gps`, etc.

**Usage in your application:**
```python
# Subscribe to odometry for all drones
for drone_id in range(1, drone_num):  # 3 drones
    ros_handler.subscribe_to_odometry_topic(f"/rs1_drone_{drone_id}/odom")

# Get pose data for map integration
pose = ros_handler.get_drone_pose(drone_id=1)
if pose:
    map_x, map_y = world_to_map_coords(pose['x'], pose['y'])
    # Update drone marker on map at (map_x, map_y)
```

## Key Principles

- **Follow established naming patterns** (`get_latest_X`, `subscribe_to_X_topic`, etc.)
- **Always use thread-safe data access** with `self.data_lock`
- **Include timestamp and header info** for data freshness checks
- **Provide both raw data access and convenience methods**
- **Handle exceptions gracefully** in message handlers
- **Use consistent error handling and logging**

## Why This Pattern?

The ROS handler follows a **data-type-specific pattern** rather than having generic methods. Each data type (camera, odometry, GPS, etc.) needs:

- Its own data dictionary (`camera_data`, `odometry_data`, etc.)
- Specific subscription methods (`subscribe_to_camera_topic`, `subscribe_to_odometry_topic`, etc.)
- A message handler (`_handle_camera_message`, `_handle_odometry_message`, etc.)
- Data retrieval methods (`get_latest_camera_image`, `get_latest_odometry`, etc.)

This pattern provides **type safety**, **clear interfaces**, and allows for **data-type-specific processing** (like converting quaternions to euler angles for odometry data).

---

## Configuration

### Camera Configuration Options

The camera system can be configured for different performance and resource requirements:

| Parameter | Default | Description | Use Case |
|-----------|---------|-------------|----------|
| `instant_switching` | `True` | Enable fast switching between cameras | Real-time monitoring |
| `preload_all` | `False` | Preload all camera streams for zero-delay switching | Maximum responsiveness |

### Common Configurations

```python
# High-performance setup (recommended for desktop)
CameraComponent(ros_handler, instant_switching=True, preload_all=True)

# Balanced setup (good for most applications)
CameraComponent(ros_handler, instant_switching=True, preload_all=False)

# Resource-efficient setup (for embedded/constrained systems)
CameraComponent(ros_handler, instant_switching=False)
```

### ROS Handler Options

```python
# Custom node name
ros_handler = RosHandler('custom_node_name')

# Pre-configure expected topics (skips auto-discovery)
ros_handler.set_expected_drone_topics(
    num_drones=3, 
    camera_types=['front', 'bottom', 'side']
)
```

---

## Repository Structure

```
.
├── ui.py                    # Main GUI application
├── ros_handler.py          # Centralised ROS2 handler
├── camera_ros.py           # Camera component using RosHandler
├── robots_panel.py         # Robot status and control panel
├── incidents_panel.py      # Incident management panel
├── drone_control_panel.py  # Drone control interface
├── map_panel.py           # Map visualisation component
├── telemetry_panel.py     # Telemetry display panel
├── popup.py               # Popup dialog system
├── constants.py           # GUI constants and colors
├── utils.py               # Utility functions
├── test_ros_handler.py    # ROS2 integration tests
├── example_extended_ros.py # Extension example
├── media/                 # Fonts, images, and assets
└── README.md
```

---

## Development Notes

- **Thread Safety**: All ROS2 data access uses proper locking mechanisms
- **Error Handling**: Graceful fallback when ROS2 is not available
- **Modular Design**: Each component can be developed independently
- **Performance**: Efficient data sharing between ROS2 thread and GUI
- **Extensibility**: Easy to add new ROS2 topics and GUI components

