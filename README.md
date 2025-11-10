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
  - **ROS 2** integration via centralised `RosHandler` class
  - GUI built with **Pygame** for responsive real-time display
  - Threaded architecture to keep GUI responsive while handling ROS2 communications
  - Modular design for easy extension to additional ROS2 topics

---

## Architecture

### Core Components

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
---

## Usage

### Prerequisites

1. Ensure Python 3.10 is installed.

2. **Install dependencies:**
   ```bash
   pip3 install -r requirements.txt
   
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

```

---


## Development Notes

- **Thread Safety**: All ROS2 data access uses proper locking mechanisms
- **Error Handling**: Graceful fallback when ROS2 is not available
- **Modular Design**: Each component can be developed independently
- **Performance**: Efficient data sharing between ROS2 thread and GUI
- **Extensibility**: Easy to add new ROS2 topics and GUI components

