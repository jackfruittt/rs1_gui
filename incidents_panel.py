
import pygame
from constants import *
from utils import feather_image

CARD_H = 80          # card height in pixels (as drawn)
STEP = 90            # vertical step between cards (80 card + 10 gap)
TOP_PAD = 80         # y-position where the first card's top sits
FEATHER_H = 30       # height of bottom feather overlay (used as bottom pad)

class IncidentsPanel:
    def __init__(self, fonts):
        self.fonts = fonts
        self.incident_y = TOP_PAD  # starting y position for incident list
        self.incident_card_rects = []
        self.incident_scroll_up_rect = None
        self.incident_scroll_down_rect = None
        self.scrolling = False
        self.scroll_direction = 1

        self.bottomFeather = None

        self.last_known_incident_count = 0
        self.cache_surface = None

    # ---- scroll clamping helpers -------------------------------------------
    def _content_height(self, n_items: int) -> int:
        """Exact vertical content height that we draw (including top pad & feather pad)."""
        if n_items <= 0:
            return TOP_PAD + FEATHER_H
        # TOP_PAD + (n-1)*STEP + CARD_H + FEATHER_H
        return TOP_PAD + (n_items - 1) * STEP + CARD_H + FEATHER_H

    def _clamp_scroll(self, n_items: int):
        """Clamp incident_y so first card can sit at TOP_PAD and the last card can be fully visible above feather."""
        content_h = self._content_height(n_items)
        # If everything fits in the panel, lock to TOP_PAD (no scrolling)
        if content_h <= PANEL_HEIGHT:
            self.incident_y = TOP_PAD
            return

        max_y = TOP_PAD  # top-most we allow (first card aligned to TOP_PAD)
        # When scrolled to the very bottom, last card's bottom y should be PANEL_HEIGHT - FEATHER_H
        # last_top = incident_y + (n-1)*STEP
        # last_bottom = last_top + CARD_H = incident_y + (n-1)*STEP + CARD_H
        # Set last_bottom == PANEL_HEIGHT - FEATHER_H  =>  incident_y == PANEL_HEIGHT - FEATHER_H - CARD_H - (n-1)*STEP
        min_y = PANEL_HEIGHT - FEATHER_H - CARD_H - (n_items - 1) * STEP

        if self.incident_y > max_y:
            self.incident_y = max_y
        if self.incident_y < min_y:
            self.incident_y = min_y

    # ------------------------------------------------------------------------
    def draw_incidents_panel(self, incidents, screen):
        """Original draw_incidents_panel function with proper scroll clamping."""
        # Surface
        incidents_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT), pygame.SRCALPHA)
        incidents_panel.fill((0, 0, 0, 0))  # Transparent background

        needs_redraw = (len(incidents) != self.last_known_incident_count or
                        self.last_known_incident_count is None or
                        self.scrolling is True)

        if needs_redraw:
            self.incident_card_rects = []
            self.last_known_incident_count = len(incidents)

            # Scroll state
            if self.scrolling:
                self.incident_y += self.scroll_direction * scroll_speed

            # Clamp scrolling to valid range
            self._clamp_scroll(len(incidents))

            # Incident entries
            for i, inc in enumerate(incidents):
                y = self.incident_y + i * STEP
                # Only process and add clicks if within visible area
                card_rect_abs = pygame.Rect(PANEL_X + 20, PANEL_Y + y, PANEL_WIDTH - 80, CARD_H)

                pygame.draw.rect(incidents_panel, LIGHT_GRAY, (20, y, PANEL_WIDTH - 80, CARD_H))
                pygame.draw.rect(incidents_panel, DARK_GRAY, (30, y + 10, 60, 60))  # Image placeholder
                pygame.draw.rect(incidents_panel, severity_colors[inc["severity"] - 1], (20, y + 75, PANEL_WIDTH - 80, 5))

                t1 = self.fonts['small_font'].render(f'{inc["title"]}', True, BLACK)
                t2 = self.fonts['small_font'].render(inc["time"], True, BLACK)
                incidents_panel.blit(t1, (100, y + 10))
                incidents_panel.blit(t2, (100, y + 40))
                if 0 <= y < PANEL_HEIGHT:
                    self.incident_card_rects.append((card_rect_abs, i))

            # Top bar
            pygame.draw.rect(incidents_panel, GRAY, (0, 0, PANEL_WIDTH, 77))
            pygame.draw.rect(incidents_panel, DARK_GREEN, (0, 0, PANEL_WIDTH, 20))

            incident_text = self.fonts['font'].render(f"Incidents ({len(incidents)})", True, WHITE)
            incidents_panel.blit(incident_text, (20, 30))

            # Bottom feathered fade (build once, reuse)
            if self.bottomFeather is None:
                self.bottomFeather = pygame.Surface((PANEL_WIDTH, FEATHER_H), pygame.SRCALPHA)
                self.bottomFeather.fill(DARK_GREEN)
                self.bottomFeather = feather_image(self.bottomFeather, 25, 25,
                                                   feather_top=True, feather_right=False,
                                                   feather_bottom=False, feather_left=False)

            incidents_panel.blit(self.bottomFeather, (0, PANEL_HEIGHT - FEATHER_H))

            # Scroll buttons
            scroll_up_rel = pygame.Rect(PANEL_WIDTH - 50, 80, 40, 100)
            scroll_down_rel = pygame.Rect(PANEL_WIDTH - 50, 280, 40, 100)
            pygame.draw.rect(incidents_panel, LIGHT_GRAY, scroll_up_rel)   # Up button
            pygame.draw.rect(incidents_panel, LIGHT_GRAY, scroll_down_rel) # Down button

            # Store absolute positions for detection
            self.incident_scroll_up_rect = pygame.Rect(PANEL_X + scroll_up_rel.x, PANEL_Y + scroll_up_rel.y,
                                                       scroll_up_rel.width, scroll_up_rel.height)
            self.incident_scroll_down_rect = pygame.Rect(PANEL_X + scroll_down_rel.x, PANEL_Y + scroll_down_rel.y,
                                                         scroll_down_rel.width, scroll_down_rel.height)

            # cache
            self.cache_surface = incidents_panel.copy()

        # Final blit to screen
        screen.blit(self.cache_surface, (PANEL_X, PANEL_Y))

    def handle_scroll_click(self, pos):
        """Handle scroll button clicks"""
        if self.incident_scroll_up_rect and self.incident_scroll_up_rect.collidepoint(pos):
            self.scrolling = True
            self.scroll_direction = 1
            return True
        elif self.incident_scroll_down_rect and self.incident_scroll_down_rect.collidepoint(pos):
            self.scrolling = True
            self.scroll_direction = -1
            return True
        return False

    def stop_scrolling(self):
        """Stop scrolling"""
        self.scrolling = False

    def get_card_rects(self):
        """Get incident card rectangles for click detection"""
        return self.incident_card_rects

    def selectIncidentButtons(self, ui, gmx, gmy):
        # incident cards
        for rect, idx in ui.incidents_panel.get_card_rects():
            if rect.collidepoint((gmx, gmy)):
                print(f"Incident clicked: #{idx+1} - {ui.incidents[idx]['title']}")
                ui.selected_incident = idx
                break
        # scroll buttons
        ui.incidents_panel.handle_scroll_click((gmx, gmy))
