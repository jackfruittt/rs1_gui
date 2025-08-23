"""
Constants extracted from original ui.py
"""

import pygame

# Global settings
pretty = True  # True = feathered edges, False = solid edges

# Colors
WHITE = (255, 255, 255)
LIGHT_GRAY = (200, 200, 200)
GRAY = (50, 50, 50)
DARK_GRAY = (30, 30, 30)
BLACK = (0, 0, 0)
BLUE = (100, 170, 255)
RED = (255, 100, 100)
YELLOW = (255, 200, 100)
GREEN = (100, 255, 100)

# State colors mapping
state_colors = {
    "Offline":   LIGHT_GRAY,
    "Scouting":  BLUE,
    "Idle":      RED,
    "Piloting":  GREEN,
    "Responding": YELLOW
}

# Severity colors and messages
severity_colors = (GREEN, YELLOW, RED)
severity_msg = ("Safe", "Warning", "Critical")

# Animation settings
fade_transition_duration = 300
ANIM_DURATION = 200  # milliseconds

# Scrolling settings
scroll_speed = 5

# World and map settings
world_size = (800, 800)

# Panel dimensions 
PANEL_WIDTH = 440
PANEL_HEIGHT = 430
PANEL_X = 1460
PANEL_Y = 0

# Button dimensions 
BTN_WIDTH = 180
BTN_HEIGHT = 80
BTN_SPACING_X = 20
BTN_SPACING_Y = 20