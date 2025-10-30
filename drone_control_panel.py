import pygame
from constants import *

import subprocess, os
from utils import spawn_cmd_safe


class DroneControlPanel:
    def __init__(self, app, fonts):
        """
        Initialises the drone control panel.
        
        Args:
            - app: The main application instance.
            - fonts: Dictionary of Pygame font objects.
        """
        self.app = app
        self.fonts = fonts
        self.button_rects = []

        self.panelState = -1 # -1 normal; 0 scout; 1 pilot; 2 send

    def create_back_button(self, surface):
        back_rect = pygame.Rect(20, PANEL_HEIGHT-40-20, BTN_WIDTH*2+20, 40)
        pygame.draw.rect(surface, (140, 30, 40), back_rect)
        pygame.draw.rect(surface, WHITE, back_rect, 2)

        self.button_rects.append((back_rect, "BACK"))
        label_surface = self.fonts['inter_medium'].render("BACK", True, WHITE)
        label_rect = label_surface.get_rect(center=back_rect.center)
        surface.blit(label_surface, label_rect)
        

    def drone_ui(self, screen):
        """
        Original drone_ui function - Draw Drone panel and handle click detection.
        
        Drone control panel in the GUI. Handles drawing the panel and buttons onto the screen.

        Args:
            - screen (pygame.Surface): The surface to draw the drone control panel onto.
        """
        self.button_rects = []  # Reset for click detection

        self.dronePic = pygame.image.load("media/images/drone_cp.png").convert_alpha()
        self.dronePic = pygame.transform.scale(self.dronePic, (200, 200))
        # Panel surface
        drone_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT))
        drone_panel.fill(BLACK)
        pygame.draw.rect(drone_panel, DARK_GREEN, (0, 0, PANEL_WIDTH, 18))

        drone_panel.blit(self.dronePic, (PANEL_WIDTH - 210, PANEL_HEIGHT - 200))

        # --- Title (top-right)
        title_surface = self.fonts['large_font'].render(f'Drone #{self.app.selected_drone + 1}', True, WHITE)
        title_rect = title_surface.get_rect(topright=(PANEL_WIDTH - 10, 19))
        drone_panel.blit(title_surface, title_rect)

        drone = self.app.drones[self.app.selected_drone]
        state = drone["state"]
        state_color = state_colors.get(state, WHITE)
        state_text = self.fonts['inter_bold_medium'].render(state, True, state_color)
        drone_panel.blit(state_text, (PANEL_WIDTH - state_text.get_width() - 10, 60))

        if state == "Offline":
            last_seen_text = self.fonts['inter_smaller'].render("last seen:", True, WHITE)
            drone_panel.blit(last_seen_text, (PANEL_WIDTH - last_seen_text.get_width() - 10, 90))

            # Location or Pose
        if drone.get("nearPose") and drone["nearPose"] != "-":
            loc_str = f"Near: {drone['nearPose']}"
        else:
            loc_str = f"{drone['gps']}, {drone['altitude']}"
        loc_text = self.fonts['inter_small'].render(loc_str, True, WHITE)
        drone_panel.blit(loc_text, (PANEL_WIDTH - loc_text.get_width() - 10, 106))

        # Battery
        batt_text = self.fonts['inter_smaller'].render(f"{drone['battery']}% battery", True, WHITE)
        drone_panel.blit(batt_text, (PANEL_WIDTH - batt_text.get_width() - 10, 128))

        match self.panelState:
            case 0:
                self.create_back_button(drone_panel)
            case 1:
                if self.app.controller_connected:
                    controller_status = self.fonts['inter_small'].render(f'Controller Connected: Yes', True, WHITE)
                elif not self.app.controller_connected:
                    controller_status = self.fonts['inter_small'].render(f'Controller Connected: No', True, WHITE)

                controller_rect = controller_status.get_rect(topright=(PANEL_WIDTH - 350, 50))
                drone_panel.blit(controller_status, controller_rect)
                self.create_back_button(drone_panel)
            case 2:
                self.create_back_button(drone_panel)
                cardRect = pygame.Rect(20, 20, BTN_WIDTH*2+20, 60)
                pygame.draw.rect(drone_panel, PINK, cardRect, 2)

                num_waypoints = len(self.app.map_panel.highlighted_waypoints)
                if num_waypoints < 1:
                    msg = self.fonts['inter_small'].render("Click mini map for custom waypoint", True, WHITE)
                    msg_rect = msg.get_rect(center=cardRect.center)
                    drone_panel.blit(msg, msg_rect)
                else:
                    msg = self.fonts['inter_bold_medium'].render(f"{num_waypoints} waypoints", True, WHITE)
                    drone_panel.blit(msg, (32, 31))

                    mini_button = pygame.Rect(348, 29, 45, 45)
                    pygame.draw.rect(drone_panel, DARK_GRAY, mini_button)
                    pygame.draw.rect(drone_panel, WHITE, mini_button, 2)

                    self.button_rects.append((mini_button, "GO"))
                    label_surface = self.fonts['inter_small'].render("GO", True, WHITE)
                    label_rect = label_surface.get_rect(center=mini_button.center)
                    drone_panel.blit(label_surface, label_rect)

                    mini_button = pygame.Rect(297, 29, 45, 45)
                    pygame.draw.rect(drone_panel, DARK_GRAY, mini_button)
                    pygame.draw.rect(drone_panel, WHITE, mini_button, 2)

                    self.button_rects.append((mini_button, "CLR"))
                    label_surface = self.fonts['inter_small'].render("CLR", True, WHITE)
                    label_rect = label_surface.get_rect(center=mini_button.center)
                    drone_panel.blit(label_surface, label_rect)

                    mini_button = pygame.Rect(245, 29, 45, 45)
                    pygame.draw.rect(drone_panel, DARK_GRAY, mini_button)
                    pygame.draw.rect(drone_panel, WHITE, mini_button, 2)

                    self.button_rects.append((mini_button, "DEF"))
                    label_surface = self.fonts['inter_small'].render("DEF", True, WHITE)
                    label_rect = label_surface.get_rect(center=mini_button.center)
                    drone_panel.blit(label_surface, label_rect)
                

            case _:

                # --- Layout regions
                left_w = int(PANEL_WIDTH * 2 / 3)     # buttons area (left 2/3)
                right_x = left_w                       # right blank area start
                right_w = PANEL_WIDTH - right_x        # right 1/3

                # --- Button grid parameters (auto layout)
                top_margin = 80
                left_margin = 20
                col0_x = left_margin
                col1_x = left_margin + BTN_WIDTH + BTN_SPACING_X
                row_gap = BTN_SPACING_Y

                labels = ["HOVER", "LAND", "PILOT", "SEND", "CLOSE"]  # 6 buttons

                # Generate rects row-major: two columns
                buttons = []
                for idx, label in enumerate(labels):
                    col = idx % 2
                    row = idx // 2
                    x = col0_x if col == 0 else col1_x
                    y = top_margin + row * (BTN_HEIGHT + row_gap)
                    # ensure we don't spill into right 1/3
                    if x + BTN_WIDTH > left_w - 20:
                        # clamp to keep inside left area
                        x = max(left_margin, left_w - 20 - BTN_WIDTH)
                    rect = pygame.Rect(x, y, BTN_WIDTH, BTN_HEIGHT)
                    buttons.append({"label": label, "rect": rect})

                    # absolute rect for click detection
                    # abs_rect = pygame.Rect(PANEL_X + x, PANEL_Y + y, BTN_WIDTH, BTN_HEIGHT)
                    self.button_rects.append((rect, label))

                # --- Draw buttons
                for btn in buttons:
                    r = btn["rect"]
                    if btn["label"].lower() == "close":
                        # soft red fill for close button
                        pygame.draw.rect(drone_panel, (140, 30, 40), r)      # fill
                        pygame.draw.rect(drone_panel, WHITE, r, 2)           # border
                    else:
                        pygame.draw.rect(drone_panel, WHITE, r, 2)

                    label_surface = self.fonts['inter_medium'].render(btn["label"], True, WHITE)
                    label_rect = label_surface.get_rect(center=r.center)
                    drone_panel.blit(label_surface, label_rect)

        # Blit panel to screen
        screen.blit(drone_panel, (PANEL_X, PANEL_Y))
    
    def gen_wp_msg(self, selectedDrone):
        waypoints = ""

        for i, wp in enumerate(self.app.drones[selectedDrone]["waypoints"]):
            if i > 0:
                waypoints += f",{wp[0]},{wp[1]},18.8"
            else:
                waypoints += f"{wp[0]},{wp[1]},18.8"

        sld = selectedDrone+1
        publish_command = f'ros2 topic pub --once /rs1_drone_{sld}/mission_assignment std_msgs/msg/String "data: \'ASSIGN,ROUTE,{waypoints}\'"'
        # print(publish_command)
        return publish_command


    def buttonLogic(self, gmx, gmy):
        mx = gmx - PANEL_X
        my = gmy - PANEL_Y
        for rect, label in self.button_rects:
            if rect.collidepoint((mx, my)):
                print(f"Drone button clicked: {label}")
                match label:
                    case "HOVER":
                        self.app.notification_ui.pushNotification("Hovering!", f"Drone set to Hover!")
                    case "LAND":
                        self.app.notification_ui.pushNotification("Landing!", f"Drone set to Land!")
                    case "SCOUT":
                        self.panelState = 0
                    case "PILOT":
                        self.panelState = 1
                        if self.app.ros_available:
                            if self.app.selected_drone >= 0:
                                self.app.ros_handler.publish_current_drone_id(self.app.selected_drone + 1)
                            success = self.app.ros_handler.call_teensy_connect(True)
                            if success:
                                print("Connected to teensy joystick")
                                self.app.controller_connected = 1
                            else:
                                print("Failed to connect to teensy joystick")
                    case "SEND":
                        self.app.map_panel.highlighted_waypoints = []
                        self.app.map_panel.highlighted_waypoints = self.app.drones[self.app.selected_drone]["waypoints"].copy()
                        self.panelState = 2
                    case "CLOSE":
                        self.app.map_panel.highlighted_waypoints = []
                        self.app.selected_drone = -1
                    case "BACK":
                        self.panelState = -1
                        if self.app.ros_available:
                            success = self.app.ros_handler.call_teensy_connect(False)
                            if success:
                                print("Disconnected from teensy joystick")
                                self.app.controller_connected = 0
                            else:
                                print("Failed to disconnect from teensy joystick")
                    case "GO":
                        self.app.notification_ui.pushNotification(f"Sent Drone {self.app.selected_drone+1}!", f"{len(self.app.map_panel.highlighted_waypoints)} Waypoints sent!", bar_color=PINK)
                        self.app.drones[self.app.selected_drone]["waypoints"] = self.app.map_panel.highlighted_waypoints.copy()
                        spawn_cmd_safe(self.gen_wp_msg(self.app.selected_drone))

                        # send blank trigger
                        sld = self.app.selected_drone+1
                        cmd = f'ros2 service call /rs1_drone_{sld}/start_mission std_srvs/srv/Trigger {{}}"'
                        spawn_cmd_safe(cmd)
                        self.panelState = -1
                    case "CLR":
                        self.app.map_panel.highlighted_waypoints = []
                    
                    case "DEF":
                        self.app.drones[self.app.selected_drone]["waypoints"] = self.app.default_waypoints[self.app.selected_drone].copy()
                        self.app.map_panel.highlighted_waypoints = self.app.drones[self.app.selected_drone]["waypoints"].copy()
                        self.app.notification_ui.pushNotification(f"Defaulting Drone {self.app.selected_drone+1}!", "Default waypoints sent!", bar_color=YELLOW)
                        sld = self.app.selected_drone+1
                        cmd = f'ros2 service call /rs1_drone_{sld}/start_mission std_srvs/srv/Trigger {{}}"'
                        spawn_cmd_safe(cmd)
                        self.panelState = -1
                break

