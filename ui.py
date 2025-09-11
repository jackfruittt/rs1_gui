#!/usr/bin/env python3
"""
RS1 Drone Swarm Control GUI - Main UI File (Refactored)
This is the main file that integrates all components together.
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


# Import our components
from constants import *
from utils import feather_image
from ros_handler import RosHandler
from ros_handler import ros2_available
from camera_ros import CameraComponent  # Replaced VideoPlayer
from drones_panel import dronesPanel
from incidents_panel import IncidentsPanel
from drone_control_panel import DroneControlPanel
from map_panel import MapPanel
from incident_detail_panel import IncidentDetailPanel
from spawn_prompt import SpawnPromptPanel

class RS1GUI:
    """Main GUI application class""" 
    
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1900, 860))
        pygame.display.set_caption("RS1 GUI")
        self.clock = pygame.time.Clock()

        self.simReady = False
        self.ros_available = ros2_available()   # was True
        self.drones = []
        self.camera_component = None
        
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
        
        # Initialize centralized ROS handler
        self.ros_handler = RosHandler('rs1_gui_main')
        # Initialize camera component as None initially
        self.camera_component = None

        # Initialise panels
        self.spawn_panel = SpawnPromptPanel(self, self.fonts, ros2_available())
        self.drones_panel = dronesPanel(self, self.fonts)
        self.incidents_panel = IncidentsPanel(self.fonts)
        self.drone_control_panel = DroneControlPanel(self, self.fonts)
        self.map_panel = MapPanel(self.map_top_view, self.drone_icon)
        self.incident_detail_panel = IncidentDetailPanel(self.fonts)

        # # Initialise camera component with instant switching (no delay when changing cameras)
        # self.camera_component = CameraComponent(
        #     self.ros_handler, 
        #     display_rect=(1240, 460, 640, 360),
        #     instant_switching=True,  # Use preload method for comparison
        #     preload_all=True        # Preload ALL cameras for zero-delay switching
        # )

        # self.odometry_topics = self.ros_handler.get_available_odometry_topics()
        # print(f"UI sees {len(self.odometry_topics)} odom topics: {self.odometry_topics}")
        # self.odom_topics = self.ros_handler.get_available_odometry_topics()
        # print(f"UI sees {len(self.odom_topics)} odom topics: {self.odom_topics}")
        # for t in self.odom_topics:
        #     self.ros_handler.subscribe_to_odometry_topic(t)
    
        
        # Initialise data 
        # self.drones = self._generate_drones()
        self.drones = []
        self.incidents = []

        # To store button state and handling button data
        self.buttons = []
        self.last_button_states = {}
        self.last_drone_id = 0

        # if ros2_available:
        #     for _ in range(4):
        #         self.drones.append(self.generate_drone())

        #     for _ in range(random.randint(3,10)):
        #         self.incidents.append(self.generate_random_incident())
        
        # Fade state
        self.show_left_fade = False
        self.left_fade_start = 0
        self.left_fadeIn = True

    def apply_odometry_to_robot(self, robot: dict, odom: dict) -> None:
            x, y, z = odom["position"]
            yaw_deg = (math.degrees(odom["rpy"][2]) + 360.0) % 360.0
            robot["gps"] = f"{x:.1f}, {y:.1f}"
            robot["altitude"] = f"{z:.1f}m"
            robot["yaw"] = round(yaw_deg, 1)

    def _update_drones_from_odometry(self):
        for i, robot in enumerate(self.drones):
            topic = f"/rs1_drone_{i+1}/odom"
            info = self.ros_handler.get_latest_odometry(topic)
            if info and self.ros_handler.is_odom_active(topic):
                self.apply_odometry_to_robot(robot, info)
    
    def _load_fonts(self):
        """Load all fonts"""
        try:
            fonts = {
                'font': pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 32),
                'small_font': pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 20),
                'large_font': pygame.font.Font("media/fonts/TurretRoad-ExtraBold.otf", 48),
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
    
    def generate_drone(self):
        """Generate a drone odom simuation purposes"""
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
            "yaw": yaw
        }
        print(f"New drone generated: {drone}")
        return drone
    
    def generate_random_incident(self):
        """Generate a random incident for simulation purposes"""
        titles = ["Fire", "Stranded Person", "Debris", "Other"]
        title = random.choice(titles)
        severity = random.randint(1, 3)
        drone = random.randint(1, 2)
        drone_coords = (round(random.uniform(-world_size[0]/2, world_size[0]/2), 3), round(random.uniform(-world_size[0]/2, world_size[0]/2), 3))
        
        incident = {
            "title": title,
            "time": "2025-08-01 12:00",
            "severity": severity,
            "drone": drone,
            "drone_coords": drone_coords
        }
        print(f"New incident generated: {incident}")
        return incident
    
    def draw_base_ui(self):
        """Original draw_base_ui function with ROS2 camera integration"""
        self.screen.fill(DARK_GRAY)

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
            self._sync_drones_from_odom()   # <-- add this


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

        # Camera status info (in bottom right)
        if self.camera_component:
            status_text = self.camera_component.get_status_info()

        debug_text = self.fonts['small_font'].render(
            f"RS1 {int(self.clock.get_fps())} | {status_text}", True, WHITE
        )
        debug_text = self.fonts['small_font'].render(
            f"RS1 {int(self.clock.get_fps())} | {status_text}", 
            True, WHITE
        )

        self.screen.blit(debug_text, (700, 760))
        
        # Controls info
        controls_text = self.fonts['small_font'].render(
            "Camera: C=Switch", 
            True, (150, 150, 150)
        )
        self.screen.blit(controls_text, (1250, 790))
    
    def handle_events(self):
        """Handle all input events"""
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

                # map incident icons
                for rect, idx in self.map_panel.get_icon_buttons():
                    if rect.collidepoint((mx, my)):
                        print(f'Icon incident clicked: {self.incidents[idx]["title"]}')
                        self.selected_incident = idx
                        break

                if self.selected_incident < 0:
                    if self.selected_drone < 0:
                        # incident cards
                        for rect, idx in self.incidents_panel.get_card_rects():
                            if rect.collidepoint((mx, my)):
                                print(f"Incident clicked: #{idx+1} - {self.incidents[idx]['title']}")
                                self.selected_incident = idx
                                break
                        # scroll buttons
                        self.incidents_panel.handle_scroll_click((mx, my))
                    else:
                        # drone control buttons
                        for rect, label in self.drone_control_panel.get_button_rects():
                            if rect.collidepoint((mx, my)):
                                print(f"Drone button clicked: {label}")
                                if label == "Close":
                                    self.selected_drone = -1
                                break

                    # drone card clicks -> optional camera jump
                    for rect, idx in self.drones_panel.get_card_rects():
                        if rect.collidepoint((mx, my)):
                            print(f"drone card clicked: #{idx+1}")
                            self.selected_drone = idx
                            if self.camera_component:  # ✅ guard
                                self.camera_component.switch_to_drone_camera(idx + 1, "front")
                            break
                else:
                    action = self.incident_detail_panel.handle_click((mx, my))
                    if action == 'close':
                        self.selected_incident = -1
                    elif action == 'respond':
                        print('Respond to incident')
                    elif action == 'clear':
                        print(f'clearing {self.selected_incident}')
                        del self.incidents[self.selected_incident]
                        self.selected_incident = -1
                        print('clear incident')

            elif event.type == pygame.MOUSEBUTTONUP:
                self.incidents_panel.stop_scrolling()


    def run(self):
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
        if not self.ros_available:
            return
        # Technically only "one" button topic (string type topic)
        butt_topics = self.ros_handler.get_available_button_topics()
        new_list = []

        # Loop redundant i think cause only one button topic anyways
        for topic in butt_topics:
            butt = self.ros_handler.get_latest_button(topic)

            # Check if there is data
            if butt:
                data = butt["data"]
                button_states = [int(x.strip()) for x in data.split(',')]

                # Compare with previous states for debounce
                prev_states = self.last_button_states.get(topic, [0]*len(button_states))

                # Detect rising edges by comparing previous and current button states 
                for i, (prev, curr) in enumerate(zip(prev_states, button_states)):
                    if prev == 0 and curr == 1:
                        # Check if button id corresponds 
                        if i == 5:
                            self.camera_component.switch_to_previous_topic()
                        elif i == 7:
                            self.camera_component.switch_to_next_topic()
                        #elif i == 6: # Waypoint button
                            # Print out Waypoint Data in GUI??
         
                        # Doesnt Work atm - need to keep track of what camera view is currently active
                        # So it switches views based on drone selected via teensyjoy
                        # elif i == 8:
                        #     # Drones is index 1 
                        #     if self.last_drone_id + 1 < len(self.drones)+1:
                        #         self.last_drone_id += 1
                        #     else:
                        #         self.last_drone_id = 1
                        #     self.camera_component.switch_to_drone_camera(self.last_drone_id, "front")

                        # Can add other remote functionality here i.e. waypoint save, +/- vel, drone change notification

                # Save states for next cycle
                self.last_button_states[topic] = button_states

                # Not sure what this does but will need clarification
                new_list.append({
                    'topic': topic,
                    'button_states': button_states,
                    'pressed_buttons': [i for i, state in enumerate(button_states) if state == 1]
                })


    def _sync_drones_from_odom(self):
        if not self.ros_available:
            return

        # Get the list of available odometry topics
        odom_topics = self.ros_handler.get_available_odometry_topics()
        new_list = []

        for topic in odom_topics:
            # Fetch the latest odometry data for the topic
            odom = self.ros_handler.get_latest_odometry(topic)
            if odom:
                # Extract drone namespace from the topic name
                parts = topic.split('/')
                if len(parts) < 3:  # '/rs1_drone_X/odom'
                    continue
                ns = parts[1]

                # Extract position and orientation data
                p = odom["position"]
                q = odom["orientation"]
                yaw = math.atan2(
                    2.0 * (q[3] * q[2] + q[0] * q[1]),
                    1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
                )

                # Add the drone data to the list
                new_list.append({
                    'ns': ns,
                    'gps': f"{p[0]:.3f}, {p[1]:.3f}",
                    'altitude': f"{p[2]:.1f}m",
                    'state': 'Active',
                    'setPose': '-',
                    'nearPose': '-',
                    'yaw': (math.degrees(yaw) + 360.0) % 360.0,
                    'battery': 100
                })

        # Update the drones list
        self.drones = new_list

def main():
    """Main entry point"""
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
