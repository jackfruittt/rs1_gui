# Horizon Hive – Group 7 (Robotics Studio 1)

This repository is part of the **Horizon Hive** project for *Robotics Studio 1*.  
The system aims to control and monitor a **drone swarm** that can help maintain and protect a forest environment.

---

## Project Overview

- **Core Goal:**  
  Develop a swarm of drones capable of monitoring a forest environment, detecting hazards, and supporting forest maintenance.

- **Mission Control GUI:**  
  This repository contains the **mission control interface**. It is planned to provide:
  - Swarm monitoring (states, positions, telemetry)
  - Live video streams from drones
  - A live map visualization with drone overlays
  - Incident reporting and control panels

- **Technology:**  
  - Interconnected using **ROS 2**
  - GUI currently prototyped in **Pygame**
  - Planned ROS integration via a dedicated connection layer (likely a separate file/module)
  - Possible communication between GUI and ROS bridge via **shared resources** or **websocket**

---

## Current State

- The GUI is **incomplete and under active development**.
- Only one main file exists: [`ui.py`](./ui.py)  
  - Contains layout experiments and interactive GUI elements
  - Displays dummy telemetry, incident lists, and a prototype drone/map panel
- **ROS 2 is not yet integrated**. Current data is generated locally/randomized for prototyping.
- Known limitations:
  - ROS 2 cannot run inside the Pygame game loop directly
  - Plan: separate ROS bridge file will handle subscriptions/publishing, and share data with the GUI

---

## Planned Development

1. **Finalize GUI layout** in Pygame
2. **Introduce ROS 2 bridge file**  
   - Handle swarm data, telemetry, incidents
   - Interface with ROSbags during testing
   - Later connect to live ROS 2 environment
3. **Connect GUI and ROS bridge**  
   - Options: shared memory, socket, or websocket
4. **Feature additions**:
   - Multi-drone live video stream panels
   - Interactive mission planning on the map
   - Incident logging and response commands
   - Health and status monitoring for each platform

---

## Running the Prototype

1. Install dependencies:
   ```bash
   pip install pygame opencv-python numpy
   ```
2. Run the GUI prototype:

   `python ui.py`
3. The GUI currently displays:
   - A forest/map splash
   - Dummy incident and robot cards
   - Simple drone control panel

## Repository structure
```
.
├── ui.py   # Main GUI prototype (Pygame)
├── media/  # Fonts, images, and sample video assets
└── README.md
```

