import pygame
from constants import *


class TelemetryPanel:
    def __init__(self, fonts):
        self.fonts = fonts
    
    def draw_telemetry_panel(self, robots, selected_robot, screen):
        """
        Original draw_telemetry_panel function
        Draws the telemetry panel for the selected robot onto the given Pygame screen surface.
        Args:
            - robots (list): List of robot telemetry data dictionaries.
            - selected_robot (int): Index of the currently selected robot.
            - screen (pygame.Surface): The surface to draw the telemetry panel onto.
        """
        # --- Create telemetry panel surface ---
        telemetry_panel = pygame.Surface((1280, 80))
        telemetry_panel.fill(GRAY)

        # --- Determine robot state color ---
        color = RED
        if robots[selected_robot]['state'] != "Offline":
            color = GREEN
        pygame.draw.rect(telemetry_panel, color, (610, 10, 60, 60))  # adjusted y to fit in panel (from 650 → 10)

        # --- Robot 1 Telemetry ---
        robot = robots[0]
        string = f"Batt: {robot['battery']}, GPS:{robot['gps']}, {robot['altitude']}"
        text = self.fonts['small_font'].render(string, True, WHITE)
        telemetry_panel.blit(text, (640 - self.fonts['small_font'].size(string)[0] - 55, 50))

        string = f"{robot['state']}"
        text = self.fonts['state_font'].render(string, True, WHITE)
        telemetry_panel.blit(text, (640 - self.fonts['state_font'].size(string)[0] - 55, 10))

        e_stop_col = DARK_GRAY

        if robot['state'] == "Offline":
            pygame.draw.rect(telemetry_panel, RED, (592, 10, 10, 60))
            string = "last seen"
            text = self.fonts['small_font'].render(string, True, WHITE)
            telemetry_panel.blit(text, (640 - self.fonts['small_font'].size(string)[0] - 60 - self.fonts['state_font'].size("Offline")[0], 30))
        else:
            pygame.draw.rect(telemetry_panel, GREEN, (592, 10, 10, 60))
            e_stop_col = RED

        pygame.draw.rect(telemetry_panel, e_stop_col, (20, 10, 160, 60))

        # --- Robot 2 Telemetry ---
        robot = robots[1]
        string = f"Batt: {robot['battery']}, GPS:{robot['gps']}, {robot['altitude']}"
        text = self.fonts['small_font'].render(string, True, WHITE)
        telemetry_panel.blit(text, (695, 50))

        if robot['state'] == "Offline":
            pygame.draw.rect(telemetry_panel, RED, (678, 10, 10, 60))
            text = self.fonts['small_font'].render("last seen", True, WHITE)
            telemetry_panel.blit(text, (840, 30))
            e_stop_col = DARK_GRAY
        else:
            pygame.draw.rect(telemetry_panel, GREEN, (678, 10, 10, 60))
            e_stop_col = RED

        telemetry_title = self.fonts['state_font'].render(f"{robot['state']}", True, WHITE)
        telemetry_panel.blit(telemetry_title, (695, 10))

        pygame.draw.rect(telemetry_panel, e_stop_col, (1100, 10, 160, 60))

        # --- Blit telemetry panel to screen ---
        screen.blit(telemetry_panel, (screen.get_width() // 2 - telemetry_panel.get_width() // 2, 640))