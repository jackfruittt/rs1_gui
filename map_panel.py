import pygame
from constants import state_colors, severity_colors, world_size
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
    
    def draw_map(self, robots, incidents, screen, selected_incident):
        """Original draw_map function"""
        PANEL_WIDTH = 800
        PANEL_HEIGHT = 800

        SCREEN_X = 380
        SCREEN_Y = 20

        self.icon_buttons = []

        map_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT)).convert_alpha()
        map_panel.fill((0, 0, 0,0))
        map_panel.blit(self.map_top_view, (0, 0))

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
            gps_x = float(gps_coords[0])
            gps_y = float(gps_coords[1])

            # print(f'Drone {i+1} {gps_x} {gps_y}')

            imgX = mapRange(gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            imgY = mapRange(gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
            
            icon_copy = self.drone_icon.copy()
            icon_copy = pygame.transform.rotate(icon_copy, robots[i]["yaw"])
            pixel_array = pygame.PixelArray(icon_copy)
            pixel_array.replace((255, 255, 255), state_color)  # Change white pixels to state color
            del pixel_array
            map_panel.blit(icon_copy, (imgX, imgY))

        screen.blit(map_panel, (SCREEN_X, SCREEN_Y))

    def get_icon_buttons(self):
        return self.icon_buttons