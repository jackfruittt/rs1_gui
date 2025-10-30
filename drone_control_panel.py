import pygame
from constants import *


class DroneControlPanel:
    """ This class handles the Drone Control Panel UI and interactions. """
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
        
    
    def drone_ui(self, screen):
        """
        Original drone_ui function - Draw Drone panel and handle click detection.
        This function handles drawing the panel specific for drone control and its buttons onto the screen.

        Args:
            - screen (pygame.Surface): The surface to draw the drone control panel onto.

        """
        self.button_rects = []  # Reset for click detection

        # Button definitions
        buttons = [
            {"label": "HOVER", "x": 20, "y": 80},
            {"label": "LAND",  "x": 20 + BTN_WIDTH + BTN_SPACING_X, "y": 80},
            {"label": "SCOUT", "x": 20, "y": 80 + BTN_HEIGHT + BTN_SPACING_Y},
            {"label": "PILOT", "x": 20 + BTN_WIDTH + BTN_SPACING_X, "y": 80 + BTN_HEIGHT + BTN_SPACING_Y},
            {"label": "Close", "x": 20, "y": 180 + (BTN_HEIGHT + BTN_SPACING_Y)}
            # {"label": "Close", "x": (PANEL_WIDTH - BTN_WIDTH) // 2, "y": 80 + (BTN_HEIGHT + BTN_SPACING_Y) * 2}
        ]

        # Precompute button rects relative to panel
        for btn in buttons:
            btn["rect"] = pygame.Rect(btn["x"], btn["y"], BTN_WIDTH, BTN_HEIGHT)
            # Store absolute rect for click detection
            abs_rect = pygame.Rect(PANEL_X + btn["x"], PANEL_Y + btn["y"], BTN_WIDTH, BTN_HEIGHT)
            self.button_rects.append((abs_rect, btn["label"]))

        # Draw panel surface
        drone_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT))
        drone_panel.fill(BLACK)

        # Title
        title_surface = self.fonts['large_font'].render(f'Drone #{self.app.selected_drone + 1}', True, WHITE)
        title_rect = title_surface.get_rect(topright=(PANEL_WIDTH - 10, 10))
        drone_panel.blit(title_surface, title_rect)

        # Draw buttons
        for btn in buttons:
            pygame.draw.rect(drone_panel, WHITE, btn["rect"], 2)
            font_to_use = self.fonts['inter_medium']
            label_surface = font_to_use.render(btn["label"], True, WHITE)
            label_rect = label_surface.get_rect(center=btn["rect"].center)
            drone_panel.blit(label_surface, label_rect)
        
        if self.app.controller_connected:
            controller_status = self.fonts['inter_medium'].render(f'Controller Connected: Yes', True, WHITE)
        elif not self.app.controller_connected:
            controller_status = self.fonts['inter_medium'].render(f'Controller Connected: No', True, WHITE)

        controller_rect = controller_status.get_rect(topright=(PANEL_WIDTH - 10, PANEL_HEIGHT - 35))
        drone_panel.blit(controller_status, controller_rect)

        # Blit panel to screen
        screen.blit(drone_panel, (PANEL_X, PANEL_Y))
    
    def get_button_rects(self):
        """Get button rectangles for click detection"""
        return self.button_rects