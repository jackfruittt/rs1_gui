#!/usr/bin/env python3
"""
RS1 Drone Swarm Control GUI - Main UI File (Refactored)
This is the main file that integrates all components together.
"""

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
        
        # Load fonts
        self.fonts = self._load_fonts()

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

        # Initialise camera component with instant switching (no delay when changing cameras)
        self.camera_component = CameraComponent(
            self.ros_handler, 
            display_rect=(1240, 460, 640, 360),
            instant_switching=True,  # Use preload method for comparison
            preload_all=True        # Preload ALL cameras for zero-delay switching
        )

        odometry_topics = self.ros_handler.get_available_odometry_topics()
        print(f"UI sees {len(odometry_topics)} odom topics: {odometry_topics}")
        odom_topics = self.ros_handler.get_available_odometry_topics()
        print(f"UI sees {len(odom_topics)} odom topics: {odom_topics}")
        for t in odom_topics:
            self.ros_handler.subscribe_to_odometry_topic(t)
        
        # Initialise panels
        self.spawn_panel = SpawnPromptPanel(self, self.fonts, ros2_available)
        self.drones_panel = dronesPanel(self, self.fonts)
        self.incidents_panel = IncidentsPanel(self.fonts)
        self.drone_control_panel = DroneControlPanel(self, self.fonts)
        self.map_panel = MapPanel(self.map_top_view, self.drone_icon)
        self.incident_detail_panel = IncidentDetailPanel(self.fonts)
    
        
        # Initialise data 
        # self.drones = self._generate_drones()
        self.drones = []
        self.incidents = []

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
    
    def apply_odometry_to_robot(self, robot: dict, odom: dict) -> None:
        x, y, z = odom["position"]
        yaw_deg = (math.degrees(odom["rpy"][2]) + 360.0) % 360.0
        robot["gps"] = f"{x:.1f}, {y:.1f}"
        robot["altitude"] = f"{z:.1f}m"
        robot["yaw"] = round(yaw_deg, 1)

    def _update_robots_from_odometry(self):
        for i, robot in enumerate(self.robots):
            topic = f"/rs1_drone_{i+1}/odom"
            info = self.ros_handler.get_latest_odometry(topic)
            if info and self.ros_handler.is_odom_active(topic):
                self.apply_odometry_to_robot(robot, info)
        
    def draw_base_ui(self):
        """Original draw_base_ui function with ROS2 camera integration"""
        self.screen.fill(DARK_GRAY)

        # Update and draw camera feed
        self.camera_component.update()
        self.camera_component.draw(self.screen)

        # Update odometry
        self._update_robots_from_odometry()

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
        status_text = self.camera_component.get_status_info()
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

            self.spawn_panel.handle_key(event)

            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.KEYDOWN:
                # Handle camera switching keys
                self.camera_component.handle_keypress(event.key)
                
                # Handle drone selection with number keys
                if event.key >= pygame.K_1 and event.key <= pygame.K_6:
                    # If holding shift, switch to that drone's camera
                    if pygame.key.get_pressed()[pygame.K_LSHIFT]:
                        drone_id = event.key - pygame.K_0
                        self.camera_component.switch_to_drone_camera(drone_id, "front")
                    else:
                        # Otherwise, select that drone in the UI
                        drone_index = event.key - pygame.K_1
                        if drone_index < len(self.drones):
                            self.selected_drone = drone_index

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos

                # check if incident icon on map is clicked
                for rect, idx in self.map_panel.get_icon_buttons():
                    if rect.collidepoint((mx, my)):
                        print(f'Icon incident clicked: {self.incidents[idx]["title"]}')
                        self.selected_incident = idx
                        
                if self.selected_incident < 0:
                    if self.selected_drone < 0:
                        # Handle incident clicks
                        for rect, idx in self.incidents_panel.get_card_rects():
                            if rect.collidepoint((mx, my)):
                                print(f"Incident clicked: #{idx+1} - {self.incidents[idx]['title']}")
                                self.selected_incident = idx
                                break
                        
                        # Handle scroll buttons
                        self.incidents_panel.handle_scroll_click((mx, my))
                    else:
                        # Handle drone control buttons
                        for rect, label in self.drone_control_panel.get_button_rects():
                            if rect.collidepoint((mx, my)):
                                print(f"Drone button clicked: {label}")
                                if label == "Close":
                                    self.selected_drone = -1
                                break

                    # Handle drone card clicks
                    for rect, idx in self.drones_panel.get_card_rects():
                        if rect.collidepoint((mx, my)):
                            print(f"drone card clicked: #{idx+1}")
                            self.selected_drone = idx
                            # Auto-switch camera to selected drone
                            self.camera_component.switch_to_drone_camera(idx + 1, "front")
                            break
                else:
                    # selected incident details panel
                    match self.incident_detail_panel.handle_click((mx, my)):
                        case 'close':
                            self.selected_incident = -1
                        case 'respond':
                            print('Respond to incident')
                        case 'clear':
                            print(f'clearing {self.selected_incident}')
                            del self.incidents[self.selected_incident]
                            self.selected_incident = -1
                            print('clear incident')
                            

            if event.type == pygame.MOUSEBUTTONUP:
                self.incidents_panel.stop_scrolling()
    
    def run(self):
        """Main application loop"""
        print("Starting RS1 Drone Control GUI...")
        
        while self.running:
            # Handle events
            self.handle_events()
            
            # Draw everything
            if self.simReady is not True:
                self.spawn_panel.draw_prompt(self.screen)
            else:
                self.draw_base_ui()
            
            
            # Update display
            pygame.display.flip()
            self.clock.tick(75)
        
        # Cleanup
        self.spawn_panel.killSim()
        self.cleanup()
    
    def cleanup(self):
        """Clean up all components"""
        print("Starting GUI cleanup...")
        self.ros_handler.cleanup()
        self.camera_component.cleanup()
        pygame.quit()
        print("GUI cleanup complete.")


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
