import pygame
from constants import state_colors, WHITE, BLACK


class RobotsPanel:
    def __init__(self, fonts):
        self.fonts = fonts
        self.robot_card_rects = []
    
    def render_robots_list(self, robots, screen):
        """Original render_robots_list function"""
        self.robot_card_rects = []  # reset each frame

        panel_width, panel_height = 350, 700
        card_height = 90
        margin = 15

        panel_surface = pygame.Surface((panel_width, panel_height)).convert_alpha()
        panel_surface.fill((0, 0, 0, 0))  # transparent

        for i, robot in enumerate(robots):
            y = i * (card_height + margin)
            if y + card_height > panel_height:
                break

            # Card rect (for detecting clicks)
            card_rect = pygame.Rect(10, 10 + y, panel_width, card_height)  # position matches blit later
            self.robot_card_rects.append((card_rect, i))  # store index with rect

            # Draw card background and border
            pygame.draw.rect(panel_surface, BLACK, (0, y, panel_width, card_height))
            pygame.draw.rect(panel_surface, WHITE, (0, y, panel_width, card_height), 2)

            # Robot ID
            id_text = self.fonts['inter_bold_large'].render(f"#{i+1}", True, WHITE)
            panel_surface.blit(id_text, (10, y + 10))

            # Robot State (color-coded)
            state = robot["state"]
            state_color = state_colors.get(state, WHITE)
            state_text = self.fonts['inter_bold_medium'].render(state, True, state_color)
            panel_surface.blit(state_text, (90, y + 10))

            if state == "Offline":
                last_seen_text = self.fonts['inter_smaller'].render("last seen:", True, WHITE)
                panel_surface.blit(last_seen_text, (90 + state_text.get_width() + 5, y + 22))

            # Location or Pose
            if robot.get("nearPose") and robot["nearPose"] != "-":
                loc_str = f"Near: {robot['nearPose']}"
            else:
                loc_str = f"{robot['gps']}, {robot['altitude']}"
            loc_text = self.fonts['inter_small'].render(loc_str, True, WHITE)
            panel_surface.blit(loc_text, (90, y + 45))

            # Battery
            batt_text = self.fonts['inter_smaller'].render(f"{robot['battery']} battery", True, WHITE)
            panel_surface.blit(batt_text, (90, y + 70))

        screen.blit(panel_surface, (10, 10))
    
    def get_card_rects(self):
        """Get clickable card rectangles"""
        return self.robot_card_rects