import pygame
from constants import state_colors, WHITE, BLACK, LIGHT_GRAY, DARK_GREEN
from utils import feather_image

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
        self.app = app

        # geometry (kept consistent with your original)
        self.PANEL_X, self.PANEL_Y = 10, 10
        self.PANEL_W, self.PANEL_H = 350, 900
        self.card_height = 90
        self.card_width = 300
        self.margin = 15
        self.inner_pad_x = 10  # left padding inside the panel for cards

        # scrolling state (mirrors incidents panel behaviour)
        self.scroll_y = 0     # starting offset inside the panel
        self.scrolling = False
        self.scroll_direction = 1   # +1 up area, -1 down area
        self.SCROLL_SPEED = 8

        # click rects (absolute)
        self.drone_card_rects = []
        self.scroll_up_abs = None
        self.scroll_down_abs = None

        self.topFeather = None
        self.bottomFeather = None

        self.max_y = 0
        self.min_y = 0

    def _content_height(self, n):
        """Exact drawn height including top/bottom padding and gaps between cards."""
        if n <= 0:
            return 0
        return (2 * self.inner_pad_x) + (n * self.card_height) + ((n - 1) * self.margin)

    def _clamp_scroll(self, n):
        """Clamp so first card can fully reach the top pad and last card can fully reach the bottom pad."""
        content_h = self._content_height(n)
        self.max_y = 0  # fully at top (first card’s top == top pad)
        # when content is shorter than panel, don't allow negative (no scrolling needed)
        self.min_y = min(0, self.PANEL_H - content_h - 50)
        if self.scroll_y > self.max_y:
            self.scroll_y = self.max_y
        if self.scroll_y < self.min_y:
            self.scroll_y = self.min_y

    def render_drones_list(self, drones, screen):
        panel_surface = pygame.Surface((self.PANEL_W, self.PANEL_H), pygame.SRCALPHA)
        panel_surface.fill((0, 0, 0, 0))

        # apply scrolling motion if active
        if self.scrolling:
            self.scroll_y += self.scroll_direction * self.SCROLL_SPEED

        # clamp to real content height
        self._clamp_scroll(len(drones))

        self.drone_card_rects = []

        for i, drone in enumerate(drones):
            # unified Y used for BOTH culling and drawing
            card_y = self.inner_pad_x + self.scroll_y + i * (self.card_height + self.margin)

            # cull off-screen rows using the same y we draw with
            if (card_y + self.card_height) < 0:
                continue
            if card_y > self.PANEL_H - self.inner_pad_x:
                continue

            card_x = self.inner_pad_x
            card_rect_rel = pygame.Rect(card_x, card_y, self.card_width, self.card_height)
            card_rect_abs = pygame.Rect(self.PANEL_X + card_x, self.PANEL_Y + card_y,
                                        self.card_width, self.card_height)
            self.drone_card_rects.append((card_rect_abs, i))

            # draw card background + border (unchanged)
            pygame.draw.rect(panel_surface, BLACK, card_rect_rel)
            pygame.draw.rect(panel_surface, WHITE, card_rect_rel, 2)

            # Drone ID
            id_text = self.fonts['inter_bold_large'].render(f"#{i+1}", True, WHITE)
            panel_surface.blit(id_text, (card_x + 10, card_y + 10))

            # State (color-coded)
            state = drone["state"]
            state_color = state_colors.get(state, WHITE)
            state_text = self.fonts['inter_bold_medium'].render(state, True, state_color)
            panel_surface.blit(state_text, (card_x + 90, card_y + 10))

            if state == "Offline":
                last_seen_text = self.fonts['inter_smaller'].render("last seen:", True, WHITE)
                panel_surface.blit(last_seen_text, (card_x + 90 + state_text.get_width() + 5, card_y + 22))

            # Location or Pose
            if drone.get("nearPose") and drone["nearPose"] != "-":
                loc_str = f"Near: {drone['nearPose']}"
            else:
                loc_str = f"{drone['gps']}, {drone['altitude']}"
            loc_text = self.fonts['inter_small'].render(loc_str, True, WHITE)
            panel_surface.blit(loc_text, (card_x + 90, card_y + 45))

            # Battery
            batt_text = self.fonts['inter_smaller'].render(f"{drone['battery']}% battery", True, WHITE)
            panel_surface.blit(batt_text, (card_x + 90, card_y + 70))

            # Selection arrow for currently selected drone
            if self.app.selected_drone == i:
                arrow_text = self.fonts['inter_bold_large'].render("◀", True, WHITE)
                arrow_x = card_x + self.card_width - arrow_text.get_width() - 10
                arrow_y = card_y + (self.card_height - arrow_text.get_height()) // 2
                panel_surface.blit(arrow_text, (arrow_x, arrow_y))

        # scrolling bars (panel-relative)
        scroll_up_rel = pygame.Rect(self.PANEL_W - 35, 50, 40, 300)
        scroll_down_rel = pygame.Rect(self.PANEL_W - 35, self.PANEL_H - 350, 40, 300)
        pygame.draw.rect(panel_surface, LIGHT_GRAY, scroll_up_rel)   # Up area
        pygame.draw.rect(panel_surface, LIGHT_GRAY, scroll_down_rel) # Down area

        # store absolute for click detection
        self.scroll_up_abs = pygame.Rect(self.PANEL_X + scroll_up_rel.x, self.PANEL_Y + scroll_up_rel.y,
                                         scroll_up_rel.width, scroll_up_rel.height)
        self.scroll_down_abs = pygame.Rect(self.PANEL_X + scroll_down_rel.x, self.PANEL_Y + scroll_down_rel.y,
                                           scroll_down_rel.width, scroll_down_rel.height)
        
        if self.bottomFeather is None:
            self.bottomFeather = pygame.Surface((self.card_width, 50), pygame.SRCALPHA)
            self.bottomFeather.fill(DARK_GREEN)
            self.bottomFeather = feather_image(self.bottomFeather, 30, 30, feather_top=True, feather_right=False, feather_bottom=False, feather_left=False)

            self.topFeather = pygame.Surface((self.card_width, 50), pygame.SRCALPHA)
            self.topFeather.fill(DARK_GREEN)
            self.topFeather = feather_image(self.topFeather, 30, 30, feather_top=False, feather_right=False, feather_bottom=True, feather_left=False)
        
        if self.scroll_y < self.max_y:
            panel_surface.blit(self.topFeather, (self.PANEL_X, 0))
        else:
            panel_surface.blit(self.bottomFeather, (self.PANEL_X, self.PANEL_H - 95))
        # final blit
        screen.blit(panel_surface, (self.PANEL_X, self.PANEL_Y))

    def handle_scroll_click(self, pos):
        """Begin scrolling if user pressed inside an up/down area (mirrors incidents panel)."""
        if self.scroll_up_abs and self.scroll_up_abs.collidepoint(pos):
            self.scrolling = True
            self.scroll_direction = 1
            return True
        if self.scroll_down_abs and self.scroll_down_abs.collidepoint(pos):
            self.scrolling = True
            self.scroll_direction = -1
            return True
        return False

    def stop_scrolling(self):
        """Stop scrolling (call on mouse button up)."""
        self.scrolling = False

    def get_card_rects(self):
        """Get clickable card rectangles (absolute)."""
        return self.drone_card_rects

    def buttonLogic(self, ui, mx, my):
        """Click handling: first try scroll bars, else card selection."""
        if self.handle_scroll_click((mx, my)):
            return

        for rect, idx in self.drone_card_rects:
            if rect.collidepoint((mx, my)):
                print(f"drone card clicked: #{idx+1}")
                ui.selected_drone = idx
                ui.drone_control_panel.panelState = -1
                ui.map_panel.customWaypoints = ui.drones[ui.selected_drone]["waypoints"].copy()
                if ui.camera_component:  # ✅ guard
                    ui.camera_component.switch_to_drone_camera(idx + 1, "front")
                break
