#!/usr/bin/env python3
"""
RS1 Drone Swarm Control GUI - Main UI File (Refactored)
____________________________________________________________

This is the main file provides a user interface for the RS1 drone swarm control system.
It integrates components of pygame and ros2 together while managing the application's state.

The GUI provides the user with real-time monitoring and control of drones in the swarm.
These include key features such as:
    - Live camera feeds from drones,
    - Map visualisation with drone position and incident markers,
    - Incident detection panel to view reported incidents,  
    - Telemetry data display for individual drones,
    - Controller integration for manual drone piloting

Refer to project README for package installation, setup instructions, arcitecture overview, and usage guidelines.
____________________________________________________________


"""

import os
# Force a known domain (use the same everywhere)
os.environ.setdefault("ROS_DOMAIN_ID", "0")
print(f"[GUI] ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}")

import pygame
import cv2
import numpy as np
import sys
import random
import math
import csv
from dataclasses import dataclass


# Import our components
from constants import *
from utils import feather_image, load_waypoints_yaml
from ros_handler import RosHandler
from ros_handler import ros2_available
from camera_ros import CameraComponent  # Replaced VideoPlayer
from drones_panel import dronesPanel
from incidents_panel import IncidentsPanel
from drone_control_panel import DroneControlPanel
from map_panel import MapPanel
from incident_detail_panel import IncidentDetailPanel
from spawn_prompt import SpawnPromptPanel
from notification import NotificationUI
from fleet_control_panel import FleetActionsPanel

@dataclass
class KnownPoses:
    feature: str
    x: int
    y: int
    z: int

def load_known_poses():
    features = []
    with open('knownPoses.csv', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                feature = KnownPoses(
                feature=row['Feature'],
                x=int(row['X']),
                y=int(row['Y']),
                z=int(row['Z'])
                )
                features.append(feature)
            except (KeyError, ValueError) as e:
                print(f"Skipping invalid row: {row} ({e})")
    return features

class RS1GUI:
    """Main GUI application class""" 
    
    def __init__(self):
        """Initialise the GUI application"""
        pygame.init()
        self.screen = pygame.display.set_mode((1900, 960))
        pygame.display.set_caption("RS1 GUI")
        self.clock = pygame.time.Clock()

        self.simReady = False
        self.ros_available = ros2_available()   # was True
        self.camera_component = None
        self._incident_seen = {}          # key: (drone, id) -> index in self.incidents
        self._incident_cleared = set()    # optional: keys you've cleared in UI

        self.known_poses = load_known_poses()

        self.drones = []
        self.incidents = []

        self.lastIncidentCount = 0

        self.default_waypoints = load_waypoints_yaml("waypoints.yaml")

        # Controller Checking, 0 - no controller found, 1 - controller connected 
        self.controller_connected = 0 

        self.sim_uid = 1

        # Load fonts
        self.fonts = self._load_fonts()

        # camera (fully guarded)
        if self.camera_component:
            self.camera_component.update()
            self.camera_component.draw(self.screen)
            status_text = self.camera_component.get_status_info()
        else:
            status_text = "Cameras: not initialised yet"

        # sync drones from odom once per frame
        if self.ros_available and self.simReady:
            self._sync_drones_from_odom()

        # UI state
        self.selected_incident = -1
        self.selected_drone = -1
        self.running = True
        
        # Load assets
        self.forest_splash = pygame.image.load("media/images/forest_splash.jpg").convert()
        self.splash_rect = self.forest_splash.get_rect(center=(640, 360))
        self.map_top_view = pygame.image.load("media/images/bird_view.png").convert_alpha()
        self.drone_icon = pygame.image.load("media/images/drone.png").convert_alpha()
        self.drone_icon = pygame.transform.scale(self.drone_icon, (35, 35))
        
        # Create feathered splash
        self.feathered_splash = feather_image(
            self.forest_splash, 150, 150, 
            feather_top=False, feather_right=True, feather_bottom=True
        )
        
        # Initialise centralized ROS handler
        self.ros_handler = RosHandler('rs1_gui_main')
        # Initialise camera component as None initially
        self.camera_component = None

        # Initialise panels
        self.spawn_panel = SpawnPromptPanel(self, self.fonts, ros2_available())
        self.drones_panel = dronesPanel(self, self.fonts)
        self.incidents_panel = IncidentsPanel(self.fonts)
        self.map_panel = MapPanel(self, self.map_top_view, self.drone_icon)
        self.incident_detail_panel = IncidentDetailPanel(self.fonts)
        self.drone_control_panel = DroneControlPanel(self, self.fonts)
        self.notification_ui = NotificationUI(self.fonts)
        self.fleet_control_panel = FleetActionsPanel(self, self.fonts)

        # Initialise camera component with instant switching (no delay when changing cameras)
        self.camera_component = CameraComponent(
            self.ros_handler, 
            display_rect=(1240, 460, 640, 360),
            instant_switching=True,  # Use preload method for comparison
            preload_all=True        # Preload ALL cameras for zero-delay switching
        )
    
        # To store button state and handling button data
        self.buttons = []
        self.last_button_states = {}
        self.last_drone_id = 0

        # Fade state
        self.show_left_fade = False
        self.left_fade_start = 0
        self.left_fadeIn = True

    def apply_odometry_to_robot(self, robot: dict, odom: dict) -> None:
        """
        Apply odometry data to the given robot dictionary.
        
        Args:
            - robot (dict): The robot data dictionary to update.
            - odom (dict): The odometry data containing position and orientation.
        """
        x, y, z = odom["position"]
        yaw_deg = (math.degrees(odom["rpy"][2]) + 360.0) % 360.0
        robot["gps"] = f"{x:.1f}, {y:.1f}"
        robot["altitude"] = f"{z:.1f}m"
        robot["yaw"] = round(yaw_deg, 1)

    def _update_drones_from_odometry(self):
        """
        Update the drones list with the latest odometry data from ROS2.
        This function fetches odometry information for each drone and applies it
        to the corresponding robot dictionary in the drones list.

        """
        for i, robot in enumerate(self.drones):
            topic = f"/rs1_drone_{i+1}/odom"
            info = self.ros_handler.get_latest_odometry(topic)
            if info and self.ros_handler.is_odom_active(topic):
                self.apply_odometry_to_robot(robot, info)

    def _sync_incidents_from_ros(self):
        """
        Original _sync_incidents_from_ros function - Merge latest ROS incidents into the UI model.
        Queries available incident topics, fetches the most recent incident per topic, and updates
        the local incidents list while respecting locally cleared items and maintaining a key→index map.
        Also trims the list to a max size to keep updates efficient.
        Args:
            - None
        Returns:
            - None
        Side Effects:
            - Reads topics from self.incident_topics (or discovers them via ros_handler if available).
            - Updates/extends self.incidents with newest incident data.
            - Maintains self._incident_seen {(drone,id) → index} and honors self._incident_cleared.
            - Trims incident history to MAX_INC and rebuilds index bookkeeping after trimming.
        """

        topics = getattr(self, "incident_topics", None) or \
                (self.ros_handler.get_available_incident_topics() if self.ros_available else [])

        for t in topics:
            inc = self.ros_handler.get_latest_incident(t)  # last incident per topic
            if not inc:
                continue
            key = (inc["drone"], inc["id"])
            if key in self._incident_cleared:
                continue  # don't re-show locally cleared items
            if key in self._incident_seen:
                idx = self._incident_seen[key]
                # Update existing item in place (keeps selection indices stF_syuable)
                self.incidents[idx].update(inc)
            else:
                self._incident_seen[key] = len(self.incidents)
                self.incidents.append(inc)

        # (Optional) keep the list small and roughly newest-first without re-sorting every frame
        MAX_INC = 200
        if len(self.incidents) > MAX_INC:
            # simple trim from the front; adjust bookkeeping
            drop = len(self.incidents) - MAX_INC
            dropped = set()
            for i in range(drop):
                old = self.incidents[i]
                dropped.add((old["drone"], old["id"]))
            self.incidents = self.incidents[drop:]
            # rebuild index map
            self._incident_seen.clear()
            for i, it in enumerate(self.incidents):
                self._incident_seen[(it["drone"], it["id"])] = i
            # also clean cleared set
            self._incident_cleared.difference_update(dropped)
    
    def _load_fonts(self):
        """This function loads all required fonts for the GUI"""
        try:
            fonts = {
                'font': pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 32),
                'small_font': pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 20),
                'large_font': pygame.font.Font("media/fonts/TurretRoad-ExtraBold.otf", 46),
                'state_font': pygame.font.Font("media/fonts/boston.ttf", 38),
                'inter_large': pygame.font.Font("media/fonts/Inter-Regular.ttf", 48),
                'inter_medium': pygame.font.Font("media/fonts/Inter-Regular.ttf", 32),
                'inter_small': pygame.font.Font("media/fonts/Inter-Regular.ttf", 18),
                'inter_smaller': pygame.font.Font("media/fonts/Inter-Regular.ttf", 16),
                'inter_bold_large': pygame.font.Font("media/fonts/Inter-Bold.ttf", 48),
                'inter_bold_medium': pygame.font.Font("media/fonts/Inter-Bold.ttf", 32),
            }
        except Exception as e:
            print(f"Error loading fonts: {e}")
            # Fallback to default font
            default_font = pygame.font.Font(None, 24)
            fonts = {key: default_font for key in [
                'font', 'small_font', 'large_font', 'state_font', 'inter_large',
                'inter_medium', 'inter_small', 'inter_smaller', 'inter_bold_large', 
                'inter_bold_medium'
            ]}
        
        return fonts

    def generate_random_waypoints(self):
        waypoints = []
        for _ in range(random.randint(3,7)):
            wx = round(random.uniform(-world_size[0]/2, world_size[0]/2), 3)
            wy = round(random.uniform(-world_size[1]/2, world_size[1]/2), 3)
            wz = -999
            waypoints.append([wx, wy, wz])
        
        return waypoints
    
    def generate_drone(self):
        """
        This function generates a drone odom simuation purposes incase ROS2 isnt available after spawn prompt.

        Returns:
            - dict: A dictionary representing the drone with keys:
                - "battery": str - Battery percentage
                - "gps": str - GPS coordinates as "x, y"
                - "altitude": str - Altitude in meters
                - "state": str - Current state of the drone
                - "setPose": str - Current set pose
                - "nearPose": str - Nearest known pose  

        """
        states = ["Scouting", "Idle", "Piloting", "Responding", "Offline"]
        knownPoses = ["Home", "Parking", "Shed", "Gate", "Table", "-"]
        state = random.choice(states)
        gps = f"{round(random.uniform(-world_size[0]/2, world_size[0]/2), 3)}, {round(random.uniform(-world_size[0]/2, world_size[0]/2), 3)}"
        alt = f"{round(random.uniform(0, 70), 3)}m"
        yaw = round(random.uniform(0, 360), 1)
        battery = f"{round(random.uniform(5,100))}"
        setPose = "-"
        nearPose = random.choice(knownPoses)

        drone = {
            "battery": battery,
            "gps": gps,
            "altitude": alt,
            "state": state,
            "setPose": setPose,
            "nearPose": nearPose,
            "yaw": yaw,
            "waypoints": []
        }
        print(f"New drone generated: {drone}")
        return drone
    
    def generate_random_incident(self):
        """
        This function generate a random incident for simulation purposes incase ROS2 isnt available after spawn prompt.

        Returns:
            - dict: A dictionary representing the incident with keys:
                - "title": str - The title of the incident
                - "time": str - The time the incident was reported
                - "severity": int - The severity level of the incident
                - "drone": int - The ID of the drone that reported the incident
                - "drone_coords": tuple - The GPS coordinates of the reporting drone
        
        """
        titles = ["Fire", "Stranded Person", "Debris", "Other"]
        title = random.choice(titles)
        severity = random.randint(1, 3)
        drone = random.randint(1, 2)
        drone_coords = (round(random.uniform(-world_size[0]/2, world_size[0]/2), 3), round(random.uniform(-world_size[0]/2, world_size[0]/2), 3))
        
        incident = {
            "id": self.sim_uid,
            "title": title,
            "time": "2025-08-01 12:00",
            "severity": severity,
            "drone": drone,
            "drone_coords": drone_coords
        }
        print(f"New incident generated: {incident}")
        self.sim_uid += 1
        return incident
    
    def draw_base_ui(self):
        """Original draw_base_ui function with ROS2 camera integration"""
        self.screen.fill(DARK_GREEN)

        # Update and draw camera feed if available
        if self.camera_component:
            self.camera_component.update()
            self.camera_component.draw(self.screen)
            status_text = self.camera_component.get_status_info()
        else:
            status_text = "Cameras: not initialised yet"

        if self.ros_available:
            self.update_controller_buttons()

        if self.ros_available and self.simReady:
            self._sync_drones_from_odom()
            self._sync_incidents_from_ros() 


        if self.ros_available and self.simReady:
            for d in self.drones:
                topic = f"/{d['ns']}/odom"
                # replace with the actual getter you have; names I've seen in your file include
                # get_latest_odometry(...) or get_odometry_data(...). Use whichever exists.
                od = self.ros_handler.get_latest_odometry(topic) if hasattr(self.ros_handler, 'get_latest_odometry') else None
                if od:
                    x, y, z = od["position"]
                    yaw = od["rpy"][2]
                    d['gps']      = f"{x:.3f}, {y:.3f}"
                    d['altitude'] = f"{z:.2f}m"
                    d['yaw']      = (math.degrees(yaw) + 360.0) % 360.0

        if self.lastIncidentCount != len(self.incidents) and self.ros_available:
            self.lastIncidentCount = len(self.incidents)
            self.notification_ui.pushNotification("New Incident!", f"{self.incidents[-1]['title']}", bar_color=severity_colors[self.incidents[-1]["severity"] - 1])

        # Map
        self.map_panel.draw_map(self.drones, self.incidents, self.screen, self.selected_incident)
        
        # drones list
        # Update odometry
        self._update_drones_from_odometry()
        self.drones_panel.render_drones_list(self.drones, self.screen)
        
        # Right panel - either incidents or drone control
        if self.selected_drone < 0 and self.selected_incident < 0:
            self.incidents_panel.draw_incidents_panel(self.incidents, self.screen)
        elif self.selected_drone >= 0:
            self.drone_control_panel.drone_ui(self.screen)
        elif self.selected_incident >= 0:
            self.incident_detail_panel.draw_incident_detail(self.incidents[self.selected_incident], self.screen)

        frame_rate = self.fonts['small_font'].render(
            f"{int(self.clock.get_fps())}", True, WHITE
        )

        self.screen.blit(frame_rate, (340, 20))
        
        # Controls info
        controls_text = self.fonts['small_font'].render(
            "Camera: C=Switch", 
            True, (150, 150, 150)
        )
        self.screen.blit(controls_text, (1250, 790))

        self.fleet_control_panel.render(self.screen)

        self.notification_ui.drawNotifications(self.screen)

        # allDrones_bg = pygame.Rect(20, 780, 300, 150)
        # pygame.draw.rect(self.screen, BLACK, allDrones_bg)


    
    def handle_events(self):
        """
        This function handles all user input events such as keyboard and mouse events.
        
        It processes events like window close, keyboard shortcuts, mouse clicks on various UI components.
        Depending on the event, it updates the application state accordingly.

        Keyboard Shortcuts:
            - ESC: Exit the application
            - 1-6: Select corresponding drone or switch camera view if Shift is held
            - Shift + Number: Switch camera to that drone
            - Number keys (1-6): Select drone in the UI
            - C : Switch camera view (when camera component is available)

        Mouse Clicks:
        The mouse click handling is context-sensitive based on current panel state.
            - Click on the buttons/icons in the map panel to select incidents
            - Click on incident cards in the incidents panel to view details
            - Click on drone card to open drone control panel
        
        Notes:
        When the simulation is not ready, only the spawn panel processes keyboard events.
        Controller connection/disconnection via ROS2 through the RosHandler interface is handled in the drone control panel.
        
        EDIT THIS DEPENDING ON CHANGES:
        When a drone is selected, it can be controlled manually (PILOT) or set back to auto-pilot (SCOUT) via a service call.
            
        """
        for event in pygame.event.get():

            # Window close
            if event.type == pygame.QUIT:
                self.running = False
                return

            # -------------------------
            # Keyboard events only here
            # -------------------------
            elif event.type == pygame.KEYDOWN:
                # global hotkeys
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    return

                # feed the spawn panel (digit entry etc.)
                self.spawn_panel.handle_key(event)

                # number keys 1..6
                if pygame.K_1 <= event.key <= pygame.K_6:
                    if self.simReady:
                        # Shift+number -> switch camera to that drone
                        if (pygame.key.get_mods() & pygame.KMOD_SHIFT) and self.camera_component:
                            drone_id = event.key - pygame.K_0
                            self.camera_component.switch_to_drone_camera(drone_id, "front")
                        else:
                            # select drone in the UI
                            drone_index = event.key - pygame.K_1
                            if 0 <= drone_index < len(self.drones):
                                self.selected_drone = drone_index

                # camera key handling
                if self.simReady and self.camera_component:
                    self.camera_component.handle_keypress(event.key)

            # -------------------------
            # Mouse events
            # -------------------------
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos

                if not self.simReady:
                    self.spawn_panel.buttonLogic(mx, my)
                    continue

                self.map_panel.processClickInMap(self, mx, my)

                self.fleet_control_panel.buttonLogic(mx, my)

                if self.selected_incident < 0:
                    self.drones_panel.buttonLogic(self, mx, my)
                    if self.selected_drone < 0:
                        self.incidents_panel.selectIncidentButtons(self, mx, my)
                    else:
                        # drone control buttons
                        self.drone_control_panel.buttonLogic(mx, my)
                else:
                    self.incident_detail_panel.handle_click(self,(mx, my))


            elif event.type == pygame.MOUSEBUTTONUP:
                self.incidents_panel.stop_scrolling()


    def run(self):
        """
        Main application loop
        This function runs the main loop of the GUI application.
        It handles events, updates the UI, and manages the application state at each iteration.

        It exits when the self.running flag is set to False, either by user action or programmatically.

        """
        print("Starting RS1 Drone Control GUI...")
        while self.running:
            self.handle_events()
            if not self.simReady:
                self.spawn_panel.draw_prompt(self.screen)
            else:
                self.draw_base_ui()
            pygame.display.flip()
            self.clock.tick(75)

        # Clean, simple shutdown
        self.spawn_panel.killSim()
        self.cleanup()

    def cleanup(self):
        """Cleanup resources on exit"""

        print("Starting GUI cleanup...")
        try:
            self.ros_handler.cleanup()
        except Exception as e:
            print(f"ROS cleanup error: {e}")
        if self.camera_component:
            try:
                self.camera_component.cleanup()
            except Exception as e:
                print(f"Camera cleanup error: {e}")
        pygame.quit()
        print("GUI cleanup complete.")

    # Function to handle camera switching using the teensyjoy controller
    # Looked at how other topics were being subscribed to and handled in
    #  ui.py, ros_handler.py and spawn_prompt.py
    def update_controller_buttons(self):
        """
        Update_controller_buttons function - Poll and process ROS2 button input states.
        Retrieves the latest comma-separated button states from available button topics, detects
        rising edges (0→1) for each button, and triggers associated UI actions (e.g., camera topic
        switching). Debounces by comparing against previously seen states per topic.
        Args:
            - None
        Returns:
            - None
        Side Effects:
            - Reads available button topics via ros_handler.
            - Updates self.last_button_states with the latest per-topic button arrays.
            - Invokes self.camera_component.switch_to_previous_topic() when button index 5 rises.
            - Invokes self.camera_component.switch_to_next_topic() when button index 7 rises.
            - Builds a transient list of button state summaries (new_list) for potential use.
        """

        if not self.ros_available:
            return
        # Technically only "one" button topic (string type topic)
        button_topics = self.ros_handler.get_available_button_topics()
        new_list = []

        # Loop redundant i think cause only one button topic anyways
        for topic in button_topics:
            button = self.ros_handler.get_latest_button(topic)

            # Check if there is data
            if button:
                data = button["data"]
                button_states = [int(x.strip()) for x in data.split(',')]

                # Compare with previous states for debounce
                prev_states = self.last_button_states.get(topic, [0]*len(button_states))

                # Detect rising edges by comparing previous and current button states 
                for i, (prev, curr) in enumerate(zip(prev_states, button_states)):
                    if prev == 0 and curr == 1:
                        # Switch only to front camera views
                        if i == 5:
                            self.camera_component.switch_to_previous_front_topic()

                        elif i == 7:
                            self.camera_component.switch_to_next_front_topic()

                        # Switch to the front camera of the next drone
                        elif i == 8:
                            if self.selected_drone == -1:
                                self.selected_drone = 0
                            self.selected_drone = self.selected_drone + 1

                            if self.selected_drone >= len(self.drones):
                                self.selected_drone = 0

                            if self.ros_available:
                                self.ros_handler.publish_current_drone_id(self.selected_drone + 1)
                            
                            # +1 extra cause camera indexing 
                            self.camera_component.switch_to_drone_camera(self.selected_drone + 1, "front")

                # Save states for next cycle
                self.last_button_states[topic] = button_states


    def _sync_drones_from_odom(self):
        """
        Original _sync_drones_from_odom function - Refresh drone list from ROS2 odometry topics.
        Discovers available odometry topics, reads latest poses, computes yaw from quaternion,
        and rebuilds the UI's drone list while preserving per-namespace fields (e.g., waypoints,
        battery). Seeds waypoints from defaults if none exist for a discovered drone.
        Args:
            - None
        Returns:
            - None
        Side Effects:
            - Reads topics and odometry messages via self.ros_handler.
            - Updates self.drones with a freshly built list of drone dicts:
              {'ns','gps','altitude','state','setPose','nearPose','yaw','battery','waypoints'}.
        """

        if not self.ros_available:
            return

        # Cache previous drones by namespace to preserve fields like waypoints
        prev_by_ns = {d.get('ns'): d for d in self.drones if d.get('ns')}

        odom_topics = self.ros_handler.get_available_odometry_topics()
        new_list = []

        for topic in odom_topics:
            odom = self.ros_handler.get_latest_odometry(topic)
            if not odom:
                continue

            parts = topic.split('/')
            if len(parts) < 3:  # '/rs1_drone_X/odom'
                continue
            ns = parts[1]

            # Extract pose
            p = odom["position"]
            q = odom["orientation"]
            yaw = math.atan2(
                2.0 * (q[3] * q[2] + q[0] * q[1]),
                1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
            )

            # preserve existing waypoints (or seed from defaults if none exist)
            prev = prev_by_ns.get(ns)
            waypoints = (prev.get('waypoints') if prev else [])
            if not waypoints:
                # Try to seed from default_waypoints using the numeric suffix in ns
                try:
                    idx = int(ns.rsplit('_', 1)[1]) - 1
                    if 0 <= idx < len(self.default_waypoints):
                        import copy
                        waypoints = copy.deepcopy(self.default_waypoints[idx])
                except Exception:
                    pass

            new_list.append({
                'ns': ns,
                'gps': f"{p[0]:.3f}, {p[1]:.3f}",
                'altitude': f"{p[2]:.1f}m",
                'state': 'Active',
                'setPose': '-',
                'nearPose': '-',
                'yaw': (math.degrees(yaw) + 360.0) % 360.0,
                'battery': (prev.get('battery') if prev else 100),
                'waypoints': waypoints,  # keep/seed waypoints
            })

        self.drones = new_list

def main():
    """
    Main entry point for the RS1 GUI application
    This function initialises and runs the RS1GUI application.
    """
    try:
        app = RS1GUI()
        app.run()
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sys.exit()


if __name__ == "__main__":
    main()
