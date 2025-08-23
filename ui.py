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

# Import our components
from constants import *
from utils import feather_image
from camera_ros import CameraComponent  # Replaced VideoPlayer
from robots_panel import RobotsPanel
from incidents_panel import IncidentsPanel
from drone_control_panel import DroneControlPanel
from map_panel import MapPanel
from telemetry_panel import TelemetryPanel
from popup import Popup

class RS1GUI:
    """Main GUI application class""" 
    
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1900, 860))
        pygame.display.set_caption("RS1 GUI")
        self.clock = pygame.time.Clock()
        
        # Load fonts
        self.fonts = self._load_fonts()
        
        # Load assets
        self.forest_splash = pygame.image.load("media/images/forest_splash.jpg").convert()
        self.splash_rect = self.forest_splash.get_rect(center=(640, 360))
        self.map_top_view = pygame.image.load("media/images/bird_view.png").convert()
        self.drone_icon = pygame.image.load("media/images/drone.png").convert_alpha()
        self.drone_icon = pygame.transform.scale(self.drone_icon, (35, 35))
        
        # Create feathered splash
        self.feathered_splash = feather_image(
            self.forest_splash, 150, 150, 
            feather_top=False, feather_right=True, feather_bottom=True
        )
        
        # Initialise camera component (replaces video player)
        self.camera_component = CameraComponent(display_rect=(1240, 460, 640, 360))
        
        # Initialise panels
        self.robots_panel = RobotsPanel(self.fonts)
        self.incidents_panel = IncidentsPanel(self.fonts)
        self.drone_control_panel = DroneControlPanel(self.fonts)
        self.map_panel = MapPanel(self.map_top_view, self.drone_icon)
        self.telemetry_panel = TelemetryPanel(self.fonts)
        self.popup = Popup(self.fonts)
        
        # Initialise data 
        self.robots = self._generate_robots()
        self.incidents = self._generate_incidents()
        
        # UI state
        self.selected_robot = -1
        self.running = True
        
        # Fade state
        self.show_left_fade = False
        self.left_fade_start = 0
        self.left_fadeIn = True
    
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
    
    def _generate_robots(self):
        """Generate robot data (from original ui.py)"""
        return [
            {
                "battery": "92%",
                "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
                "altitude": "102m",
                "state": "Offline",
                "setPose": "-",
                "nearPose": "-",
                "yaw": round(random.uniform(0, 360), 1)
            },
            {
                "battery": "64%",
                "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
                "altitude": "87m",
                "state": "Scouting",
                "setPose": "-",
                "nearPose": "Lake",
                "yaw": round(random.uniform(0, 360), 1)
            },
            {
                "battery": "78%",
                "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
                "altitude": "56m",
                "state": "Idle",
                "setPose": "-",
                "nearPose": "-",
                "yaw": round(random.uniform(0, 360), 1)
            },
            {
                "battery": "51%",
                "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
                "altitude": "73m",
                "state": "Piloting",
                "setPose": "-",
                "nearPose": "Parking",
                "yaw": round(random.uniform(0, 360), 1)
            },
            {
                "battery": "88%",
                "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
                "altitude": "64m",
                "state": "Responding",
                "setPose": "-",
                "nearPose": "-",
                "yaw": round(random.uniform(0, 360), 1)
            },
            {
                "battery": "33%",
                "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
                "altitude": "45m",
                "state": "Offline",
                "setPose": "-",
                "nearPose": "Base",
                "yaw": round(random.uniform(0, 360), 1)
            }
        ]
    
    def _generate_incidents(self):
        """Generate incidents data (from original ui.py)"""
        return [
            {
                "id": 1, "title": "Fire Detected", "time": "2025-08-01 13:20", "severity": 3,
                "Platform": 1, "robot_coords": (-0.005, 0.002), "global_coords": (-33.8701, 151.2101)
            },
            {
                "id": 2, "title": "Fungal Detected", "time": "2025-08-01 13:45", "severity": 1,
                "Platform": 2, "robot_coords": (0.003, -0.001), "global_coords": (-33.8670, 151.2070)
            },
            {
                "id": 3, "title": "Bear Detected", "time": "2025-08-01 14:05", "severity": 3,
                "Platform": 1, "robot_coords": (-0.002, 0.004), "global_coords": (-33.8690, 151.2088)
            },
            {
                "id": 4, "title": "Flood Warning", "time": "2025-08-01 14:30", "severity": 2,
                "Platform": 2, "robot_coords": (0.001, 0.003), "global_coords": (-33.8665, 151.2065)
            },
            {
                "id": 5, "title": "Unauthorized Entry", "time": "2025-08-01 15:00", "severity": 3,
                "Platform": 1, "robot_coords": (-0.004, -0.002), "global_coords": (-33.8685, 151.2095)
            },
            {
                "id": 6, "title": "Smoke Alert", "time": "2025-08-01 15:20", "severity": 2,
                "Platform": 2, "robot_coords": (0.002, 0.001), "global_coords": (-33.8678, 151.2079)
            },
            {
                "id": 7, "title": "Gas Leak Detected", "time": "2025-08-01 15:45", "severity": 3,
                "Platform": 1, "robot_coords": (-0.001, -0.003), "global_coords": (-33.8693, 151.2083)
            },
            {
                "id": 8, "title": "Tree Fallen", "time": "2025-08-01 16:10", "severity": 1,
                "Platform": 2, "robot_coords": (0.004, -0.004), "global_coords": (-33.8659, 151.2062)
            },
            {
                "id": 9, "title": "Animal Intrusion", "time": "2025-08-01 16:35", "severity": 2,
                "Platform": 1, "robot_coords": (-0.003, 0.001), "global_coords": (-33.8682, 151.2090)
            },
            {
                "id": 10, "title": "Power Outage", "time": "2025-08-01 17:00", "severity": 2,
                "Platform": 2, "robot_coords": (0.002, -0.002), "global_coords": (-33.8667, 151.2072)
            },
        ]
    
    def draw_base_ui(self):
        """Original draw_base_ui function with ROS2 camera integration"""
        self.screen.fill(DARK_GRAY)

        # Update and draw camera feed
        self.camera_component.update()
        self.camera_component.draw(self.screen)

        # Map
        self.map_panel.draw_map(self.robots, self.screen)
        
        # Robots list
        self.robots_panel.render_robots_list(self.robots, self.screen)
        
        # Right panel - either incidents or drone control
        if self.selected_robot < 0:
            self.incidents_panel.draw_incidents_panel(self.incidents, self.screen)
        else:
            self.drone_control_panel.drone_ui(self.screen)

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
        self.screen.blit(controls_text, (20, 760))
    
    def handle_events(self):
        """Handle all input events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.KEYDOWN:
                # Handle camera switching keys
                self.camera_component.handle_keypress(event.key)
                
                # Handle robot selection with number keys
                if event.key >= pygame.K_1 and event.key <= pygame.K_6:
                    # If holding shift, switch to that robot's camera
                    if pygame.key.get_pressed()[pygame.K_LSHIFT]:
                        drone_id = event.key - pygame.K_0
                        self.camera_component.switch_to_drone_camera(drone_id, "front")
                    else:
                        # Otherwise, select that robot in the UI
                        robot_index = event.key - pygame.K_1
                        if robot_index < len(self.robots):
                            self.selected_robot = robot_index

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos

                # Handle popup close
                if self.popup.handle_close_click((mx, my)):
                    continue

                if not self.popup.popup_visible:
                    if self.selected_robot < 0:
                        # Handle incident clicks
                        for rect, idx in self.incidents_panel.get_card_rects():
                            if rect.collidepoint((mx, my)):
                                print(f"Incident clicked: #{idx+1} - {self.incidents[idx]['title']}")
                                # Uncomment to enable popup:
                                # self.popup.show_popup(self.incidents[idx])
                                break
                        
                        # Handle scroll buttons
                        self.incidents_panel.handle_scroll_click((mx, my))
                    else:
                        # Handle drone control buttons
                        for rect, label in self.drone_control_panel.get_button_rects():
                            if rect.collidepoint((mx, my)):
                                print(f"Drone button clicked: {label}")
                                if label == "Close":
                                    self.selected_robot = -1
                                break

                    # Handle robot card clicks
                    for rect, idx in self.robots_panel.get_card_rects():
                        if rect.collidepoint((mx, my)):
                            print(f"Robot card clicked: #{idx+1}")
                            self.selected_robot = idx
                            # Auto-switch camera to selected robot
                            self.camera_component.switch_to_drone_camera(idx + 1, "front")
                            break

            if event.type == pygame.MOUSEBUTTONUP:
                self.incidents_panel.stop_scrolling()
    
    def run(self):
        """Main application loop"""
        print("Starting RS1 Drone Control GUI...")
        
        while self.running:
            # Handle events
            self.handle_events()
            
            # Draw everything
            self.draw_base_ui()
            
            # Handle popup animation and drawing
            self.popup.handle_popup_animation()
            self.popup.draw_popup(self.screen)
            
            # Update display
            pygame.display.flip()
            self.clock.tick(75)
        
        # Cleanup
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
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
