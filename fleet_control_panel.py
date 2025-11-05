import pygame
from constants import BLACK, WHITE
from utils import spawn_cmd_safe

CLOSE_RED = (140, 30, 40)   # same red as the 'CLOSE' button in the exemplar panel

class FleetActionsPanel:
    def __init__(self, app, fonts, pos=(20, 780), size=(300, 150)):
        self.app = app
        self.fonts = fonts
        self.pos = pos
        self.size = size
        self.button_rects = []   # list of (pygame.Rect, label) in panel-local coords

        # Choose a readable medium font; fall back if key doesn't exist
        self._label_font = (
            (self.fonts.get('inter_medium') if isinstance(self.fonts, dict) else None)
            or pygame.font.SysFont(None, 20)
        )

    def _draw_button(self, surface, rect, label, fill=None, border_color=WHITE, border_w=2, label_color=WHITE):
        """Draw a button like in the exemplar: optional fill, white border, centered label."""
        if fill is not None:
            pygame.draw.rect(surface, fill, rect)
        pygame.draw.rect(surface, border_color, rect, border_w)

        label_surf = self._label_font.render(label, True, label_color)
        label_rect = label_surf.get_rect(center=rect.center)
        surface.blit(label_surf, label_rect)

    def render(self, screen):
        """Draw the panel and its two buttons to the screen at self.pos."""
        self.button_rects = []  # reset per-frame

        w, h = self.size
        panel = pygame.Surface((w, h))
        panel.fill(BLACK)

        # layout: ~10px border, two stacked buttons with ~10px gap
        margin = 10
        gap = 10
        inner_w = w - margin * 2
        inner_h = h - margin * 2
        btn_h = (inner_h - gap) // 2

        takeoff_rect = pygame.Rect(margin, margin, inner_w, btn_h)
        land_rect    = pygame.Rect(margin, margin + btn_h + gap, inner_w, btn_h)

        # TAKEOFF All: border-only (like regular buttons)
        self._draw_button(panel, takeoff_rect, "TAKEOFF All", fill=None, border_color=WHITE, border_w=2, label_color=WHITE)
        self.button_rects.append((takeoff_rect, "TAKEOFF All"))

        # LAND All: filled with CLOSE_RED (same shade as exemplar 'CLOSE' button), white border
        self._draw_button(panel, land_rect, "LAND All", fill=CLOSE_RED, border_color=WHITE, border_w=2, label_color=WHITE)
        self.button_rects.append((land_rect, "LAND All"))

        # blit to screen
        screen.blit(panel, self.pos)

    def buttonLogic(self, gmx, gmy):
        """Check if a global click (gmx,gmy) hit a button; print to terminal if so."""
        px, py = self.pos
        mx = gmx - px
        my = gmy - py
        for rect, label in self.button_rects:
            if rect.collidepoint((mx, my)):
                print(f"Fleet button clicked: {label}")
                if label == "TAKEOFF All":
                    spawn_cmd_safe("ros2 service call /rs1_drone_$/takeoff_drone std_srvs/srv/Trigger")
                elif label == "LAND All":
                    spawn_cmd_safe("ros2 service call /rs1_drone_$/land_drone std_srvs/srv/Trigger")
                break