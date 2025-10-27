import pygame
from constants import state_colors, severity_colors, world_size, PINK
from utils import mapRange


class MapPanel:
    def __init__(self, map_top_view, drone_icon):
        self.map_top_view = map_top_view
        self.drone_icon = drone_icon
        self.mapImgSize = map_top_view.get_size()

        self.person_icon = pygame.image.load("media/images/person.png").convert_alpha()
        self.fire_icon = pygame.image.load("media/images/fire.png").convert_alpha()
        self.garbage_icon = pygame.image.load("media/images/garbage.png").convert_alpha()
        self.warning_icon = pygame.image.load("media/images/warning.png").convert_alpha()

        self.person_icon = pygame.transform.scale(self.person_icon, (30, 40))
        self.fire_icon = pygame.transform.scale(self.fire_icon, (30, 40))
        self.garbage_icon = pygame.transform.scale(self.garbage_icon, (30, 35))
        self.warning_icon = pygame.transform.scale(self.warning_icon, (30, 30))

        self.icon_buttons = []

        self.customWaypoints = []
    
    def draw_map(self, robots, incidents, screen, selected_incident):
        """Original draw_map function"""
        PANEL_WIDTH = 800
        PANEL_HEIGHT = 800

        SCREEN_X = 380
        SCREEN_Y = 20

        CUSTOM_WAYPOINT_RADIUS = 8
        CUSTOM_PATH_THICKNESS = 4

        self.icon_buttons = []

        map_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT)).convert_alpha()
        map_panel.fill((0, 0, 0,0))
        map_panel.blit(self.map_top_view, (0, 0))

        for i in range(len(self.customWaypoints)):
            wx, wy, wz = self.customWaypoints[i]
            imgX = mapRange(wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            imgY = mapRange(wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

            # draw node
            pygame.draw.circle(map_panel, PINK, (int(imgX), int(imgY)), CUSTOM_WAYPOINT_RADIUS)

            # connect to previous node
            if i > 0:
                prev_wx, prev_wy, prev_wz = self.customWaypoints[i-1]
                prev_imgX = mapRange(prev_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
                prev_imgY = mapRange(prev_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

                pygame.draw.line(map_panel, PINK,
                                (int(prev_imgX), int(prev_imgY)),
                                (int(imgX), int(imgY)),
                                CUSTOM_PATH_THICKNESS)

        # close the loop: connect last → first
        if len(self.customWaypoints) > 2:
            first_wx, first_wy, first_wz = self.customWaypoints[0]
            last_wx, last_wy, last_wz = self.customWaypoints[-1]

            first_imgX = mapRange(first_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            first_imgY = mapRange(first_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
            last_imgX = mapRange(last_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            last_imgY = mapRange(last_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

            pygame.draw.line(map_panel, PINK,
                            (int(last_imgX), int(last_imgY)),
                            (int(first_imgX), int(first_imgY)),
                            CUSTOM_PATH_THICKNESS)


        for i, inc in enumerate(incidents):
            colour = severity_colors[inc["severity"] - 1]
            gps_coords = incidents[i]["drone_coords"]
            gps_x = float(gps_coords[0])
            gps_y = float(gps_coords[1])
            title = incidents[i]["title"]
            imgX = mapRange(gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            imgY = mapRange(gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

            icon = None
            if 'Fire' in title:
                icon = self.fire_icon.copy()
            elif 'Person' in title:
                icon = self.person_icon.copy()
            elif 'Debris' in title:
                icon = self.garbage_icon.copy()
            else:
                icon = self.warning_icon.copy()
            
            
            pixel_array = pygame.PixelArray(icon)
            pixel_array.replace((0,0,0), colour)
            del pixel_array
            map_panel.blit(icon, (imgX, imgY))

            icon_button = pygame.Rect(imgX+SCREEN_X, imgY+SCREEN_Y, icon.get_width(), icon.get_height())
            self.icon_buttons.append((icon_button, i))


        for i in range(len(robots)):
            state = robots[i]["state"]
            
            state_color = state_colors.get(state, (255, 255, 255))

            # Parse GPS string to get coordinates
            gps_coords = robots[i]["gps"].split(', ')
            gps_x = float(gps_coords[1])
            gps_y = float(gps_coords[0])

            # print(f'Drone {i+1} {gps_x} {gps_y}')

            # Temporary Fix to align Drone Images on GUI Map to match Gazebo Positions with Ace Branch rs1_robot drone_spawner.py
            imgX = mapRange(-gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[1]) - 28
            imgY = mapRange(-gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1]) - 55
            
            icon_copy = self.drone_icon.copy()
            icon_copy = pygame.transform.rotate(icon_copy, robots[i]["yaw"])
            pixel_array = pygame.PixelArray(icon_copy)
            pixel_array.replace((255, 255, 255), state_color)  # Change white pixels to state color
            del pixel_array
            map_panel.blit(icon_copy, (imgX, imgY))

        screen.blit(map_panel, (SCREEN_X, SCREEN_Y))

    def get_icon_buttons(self):
        return self.icon_buttons

    def processClickInMap(self, ui, gmx, gmy):
        # map incident icons

        if gmx > 380 and gmx < 1180 and gmy > 20 and gmy < 820:

            if ui.selected_drone < 0:
                for rect, idx in self.icon_buttons:
                    if rect.collidepoint((gmx, gmy)):
                        print(f'Icon incident clicked: {ui.incidents[idx]["title"]}')
                        ui.selected_incident = idx
                        break
            else:

                if ui.drone_control_panel.panelState == 2:
                    lx = gmx - 380
                    ly = gmy - 20
                    lx = mapRange(lx, 0, self.mapImgSize[0], -(world_size[0]/2), (world_size[0]/2))
                    ly = mapRange(ly, 0, self.mapImgSize[1], -(world_size[1]/2), (world_size[1]/2))
                    self.customWaypoints.append([lx, ly, -999])
                    print(f'custom waypoint added at {lx}, {ly}')