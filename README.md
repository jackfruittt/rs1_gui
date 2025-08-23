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

### Adding New ROS2 Topics

The architecture makes it easy to add new ROS2 functionality:

1. **Extend the RosHandler** for new message types:
   ```python
   # Add custom callback for new topic type
   ros_handler.register_topic_callback(topic_name, your_callback_function)
   
   # Access data through thread-safe methods
   data = ros_handler.get_latest_data(topic_name)
   ```

2. **Create a GUI component** that uses the shared RosHandler:
   ```python
   class NewComponent:
       def __init__(self, ros_handler):
           self.ros_handler = ros_handler
           # Subscribe to your topics
   ```

4. **Access data thread-safely** through the RosHandler methods:
   ```python
   # Get latest camera image
   image_data = ros_handler.get_latest_camera_image(topic_name)
   
   # Check if topic is active
   is_active = ros_handler.is_topic_active(topic_name)
   
   # Get handler status
   status = ros_handler.get_status_info()
   ```

---

## Architecture Benefits

- **Responsive GUI**: ROS2 runs in separate thread, GUI never freezes
- **Centralized management**: Single point for all ROS2 communications
- **Configurable performance**: Multiple camera switching modes for different requirements
- **Easy extension**: Add new topics without modifying existing code
- **Thread safety**: All data access is properly synchronized
- **Optimized subscriptions**: Intelligent topic management reduces resource usage
- **Graceful degradation**: Works with or without ROS2 environment
- **Manual configuration**: Can bypass auto-discovery for faster startup

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

