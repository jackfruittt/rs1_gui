import pygame
from constants import state_colors, world_size
from utils import mapRange


class MapPanel:
    def __init__(self, map_top_view, drone_icon):
        self.map_top_view = map_top_view
        self.drone_icon = drone_icon
        self.mapImgSize = map_top_view.get_size()
    
    def draw_map(self, robots, screen):
        """Original draw_map function"""
        PANEL_WIDTH = 800
        PANEL_HEIGHT = 800

        map_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT))
        map_panel.fill((0, 0, 0))
        map_panel.blit(self.map_top_view, (0, 0))

        for i in range(len(robots)):
            state = robots[i]["state"]
            
            state_color = state_colors.get(state, (255, 255, 255))
            print(f'Robot {i+1} + {state} {state_color} {robots[i]["yaw"]}')

            # Parse GPS string to get coordinates
            gps_coords = robots[i]["gps"].split(', ')
            gps_x = float(gps_coords[0])
            gps_y = float(gps_coords[1])

            imgX = mapRange(gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            imgY = mapRange(gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
            
            icon_copy = self.drone_icon.copy()
            icon_copy = pygame.transform.rotate(icon_copy, robots[i]["yaw"])
            pixel_array = pygame.PixelArray(icon_copy)
            pixel_array.replace((255, 255, 255), state_color)  # Change white pixels to state color
            del pixel_array
            map_panel.blit(icon_copy, (imgX, imgY))

        screen.blit(map_panel, (500, 20))