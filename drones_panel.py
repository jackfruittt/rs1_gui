import pygame
from constants import state_colors, WHITE, BLACK, LIGHT_GRAY

class dronesPanel:
    """ This class handles the Drones Panel UI and interactions. """
    def __init__(self, app, fonts):
        """
        Initializes the drones panel.
        
        Args:
            - app: The main application instance.
            - fonts: Dictionary of Pygame font objects.

        """

        self.fonts = fonts
        self.drone_card_rects = []
        self.app = app
    
    def render_drones_list(self, drones, screen):
        """
        This function renders the list of drone cards onto the given Pygame screen surface based on the drones data from app.
        
        Args:
            - drones (list): List of drones present in the system.
            - screen (pygame.Surface): The surface to draw the drones panel onto.

        """

        self.drone_card_rects = []  # reset each frame

        panel_width, panel_height = 350, 900
        card_height = 90
        card_width = 300
        margin = 15

        panel_surface = pygame.Surface((panel_width, panel_height)).convert_alpha()
        panel_surface.fill((0, 0, 0, 0))  # transparent

        for i, drone in enumerate(drones):
            y = i * (card_height + margin)
            if y + card_height > panel_height:
                break

            # Card rect (for detecting clicks)
            card_rect = pygame.Rect(10, 10 + y, card_width, card_height)
            self.drone_card_rects.append((card_rect, i))

            # Draw card background and border
            pygame.draw.rect(panel_surface, BLACK, (0, y, card_width, card_height))
            pygame.draw.rect(panel_surface, WHITE, (0, y, card_width, card_height), 2)

            # drone ID
            id_text = self.fonts['inter_bold_large'].render(f"#{i+1}", True, WHITE)
            panel_surface.blit(id_text, (10, y + 10))

            # drone State (color-coded)
            state = drone["state"]
            state_color = state_colors.get(state, WHITE)
            state_text = self.fonts['inter_bold_medium'].render(state, True, state_color)
            panel_surface.blit(state_text, (90, y + 10))

            if state == "Offline":
                last_seen_text = self.fonts['inter_smaller'].render("last seen:", True, WHITE)
                panel_surface.blit(last_seen_text, (90 + state_text.get_width() + 5, y + 22))

            # Location or Pose
            if drone.get("nearPose") and drone["nearPose"] != "-":
                loc_str = f"Near: {drone['nearPose']}"
            else:
                loc_str = f"{drone['gps']}, {drone['altitude']}"
            loc_text = self.fonts['inter_small'].render(loc_str, True, WHITE)
            panel_surface.blit(loc_text, (90, y + 45))

            # Battery
            batt_text = self.fonts['inter_smaller'].render(f"{drone['battery']}% battery", True, WHITE)
            panel_surface.blit(batt_text, (90, y + 70))

            # Selection arrow for currently selected drone
            if self.app.selected_drone == i:
                arrow_text = self.fonts['inter_bold_large'].render("◀", True, WHITE)
                arrow_x = card_width - arrow_text.get_width() - 10
                arrow_y = y + (card_height - arrow_text.get_height()) // 2
                panel_surface.blit(arrow_text, (arrow_x, arrow_y))
        
        #scrolling buttons
        scroll_up_rel = pygame.Rect(panel_width - 50, 80, 40, 100)
        scroll_down_rel = pygame.Rect(panel_height - 50, 280, 40, 100)
        pygame.draw.rect(panel_surface, LIGHT_GRAY, scroll_up_rel)   # Up button
        pygame.draw.rect(panel_surface, LIGHT_GRAY, scroll_down_rel) # Down button # not showing

        screen.blit(panel_surface, (10, 10))
    
    def get_card_rects(self):
        """
        This function gets clickable card rectangles from the drone list

        Returns:
            - list: List of tuples containing card rectangles and their corresponding drone indices.
            
        """
        return self.drone_card_rects
