
import pygame
from constants import *
from utils import feather_image


class IncidentDetailPanel:
    """    
    This class represents the Incident Detail view for a single incident.
    Intended to replace the incidents list panel when the user clicks on an incident card.

    Style and sizing follow the same conventions as IncidentsPanel.

    Exposes click handling for three controls:
      - Close (top-right 'X')
      - Respond (primary action button)
      - Clear (secondary action button)

    """

    def __init__(self, fonts):
        self.fonts = fonts
        # Absolute rects (screen space) for click detection
        self._close_rect_abs = None
        self._respond_rect_abs = None
        self._clear_rect_abs = None

        self.scenario_image_cache = {}
    
    def _get_scenario_image_surface(self, incident, camera_component, img_w, img_h):
        """
        Get the scenario image surface for the given incident.
        Uses caching to avoid redundant conversions (5 second freshness).
        Reuses camera_component's conversion method to avoid code duplication.
        
        Args:
            incident: Incident dictionary with 'drone' key
            camera_component: CameraComponent instance (has ros_handler and conversion method)
            img_w, img_h: Target width and height for the image
            
        Returns:
            pygame.Surface or None: Image surface if available, None otherwise
        """
        if not camera_component or not camera_component.ros_handler.ros2_available:
            return None
            
        # Construct topic name from drone ID
        drone_id = incident.get('drone', 1)
        topic_name = f"/rs1_drone_{drone_id}/scenario_img"
        
        # Check cache first
        import time
        cache_key = (topic_name, img_w, img_h)
        if cache_key in self.scenario_image_cache:
            cached_data = self.scenario_image_cache[cache_key]
            # Check if cache is still fresh (5 seconds)
            if time.time() - cached_data['timestamp'] < 5.0:
                return cached_data['surface']
        
        # Try to get image from ROS
        cv_image = camera_component.ros_handler.get_latest_scenario_image(topic_name)

        surface = camera_component._cv2_to_pygame_surface(cv_image)

        # Cache the result
        self.scenario_image_cache[cache_key] = {
            'surface': surface,
            'timestamp': time.time()
        }
        return surface
    
    def draw_incident_detail(self, incident, screen, camera_component=None):
        """
        Render the incident detail panel.
        The User is notified in the main screen of the GUI of the detected incident details if any.
        The included details of the incident can be seen below
        NOTE: Update this param incident if the csv structure changes
        Args:
            - param incident: dict with keys:
            id, title, time, severity, Platform, drone_coords, global_coords
            - param screen: pygame Surface (main screen)

            - param screen: pygame Surface (main screen)

        """

        # Root panel surface (transparent)
        panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT), pygame.SRCALPHA)
        panel.fill((0, 0, 0, 0))

        # ===== Top bars (match list panel) =====
        pygame.draw.rect(panel, GRAY, (0, 0, PANEL_WIDTH, 77))
        pygame.draw.rect(panel, DARK_GREEN, (0, 0, PANEL_WIDTH, 20))

        # Title
        title_text = self.fonts['font'].render("Incident Detail", True, WHITE)
        panel.blit(title_text, (20, 30))

        # Close button ('X') in top-right
        close_size = 50
        close_rel = pygame.Rect(PANEL_WIDTH - close_size - 12, 24, close_size, close_size)
        pygame.draw.rect(panel, LIGHT_GRAY, close_rel)
        x_text = self.fonts['small_font'].render("X", True, BLACK)
        # Center 'X' in the button
        panel.blit(x_text, (close_rel.x + (close_rel.w - x_text.get_width()) // 2,
                            close_rel.y + (close_rel.h - x_text.get_height()) // 2))

        # Store absolute rect for click detection
        self._close_rect_abs = pygame.Rect(PANEL_X + close_rel.x, PANEL_Y + close_rel.y,
                                           close_rel.w, close_rel.h)

        # ===== Content card =====
        content_margin = 20
        card_top = 90
        card_height = PANEL_HEIGHT - card_top - 90
        pygame.draw.rect(panel, LIGHT_GRAY, (content_margin, card_top,
                                             PANEL_WIDTH - 2 * content_margin, card_height))

        # Severity accent line (bottom of the content card)
        sev_color = severity_colors[max(0, min(incidents_severity_index(incident), len(severity_colors)-1))]
        pygame.draw.rect(panel, sev_color, (content_margin, card_top + card_height - 6,
                                            PANEL_WIDTH - 2 * content_margin, 6))

        # ===== Image placeholder =====
        # Keep proportions and spacing similar to list panel; slightly larger for detail
        img_w, img_h = 220, 160
        img_x = content_margin + 10
        img_y = card_top + 10
        
        # Draw placeholder background
        pygame.draw.rect(panel, DARK_GRAY, (img_x, img_y, img_w, img_h))
        
        # Try to get and display scenario image
        scenario_surface = None
        if camera_component is not None:
            scenario_surface = self._get_scenario_image_surface(incident, camera_component, img_w, img_h)
        
        if scenario_surface:
            # Display the actual scenario image
            panel.blit(scenario_surface, (img_x, img_y))
        else:
            # Display placeholder text if no image available
            font = self.fonts['small_font']
            no_img_text = font.render("No Image", True, LIGHT_GRAY)
            text_rect = no_img_text.get_rect(center=(img_x + img_w//2, img_y + img_h//2))
            panel.blit(no_img_text, text_rect)
        
        # Draw border around image area
        pygame.draw.rect(panel, (80, 80, 80), (img_x, img_y, img_w, img_h), 2)


        # ===== Textual details =====
        # Left column: image; right column: metadata
        meta_x = img_x + img_w + 20
        meta_y = img_y
        line_gap = 10

        def line(text, font_key='small_font', color=BLACK):
            surf = self.fonts[font_key].render(text, True, color)
            nonlocal meta_y
            panel.blit(surf, (meta_x, meta_y))
            meta_y += surf.get_height() + line_gap

        # Header with id and title
        hdr = f"{incident.get('title', 'Unknown')}"
        line(hdr, font_key='font')

        # Time & severity
        line(f"Time: {incident.get('time', '—')}")
        line(f"Severity: {incident.get('severity', '—')}")

        # Platform / coordinates
        line(f"Drone: {incident.get('drone', '—')}")
        drone_coords = incident.get('drone_coords', ('—', '—'))
        line(f"Drone coords: {drone_coords}")
        
        # ===== Action buttons (bottom area) =====
        btn_w, btn_h = 140, 46
        space = 16
        btn_y = PANEL_HEIGHT - btn_h - 20

        # Primary: Respond (right-aligned group)
        respond_rel = pygame.Rect(PANEL_WIDTH - content_margin - btn_w, btn_y, btn_w, btn_h)
        clear_rel = pygame.Rect(respond_rel.x - btn_w - space, btn_y, btn_w, btn_h)

        # Buttons visual
        for r, label in ((clear_rel, "Clear"), (respond_rel, "Respond")):
            pygame.draw.rect(panel, LIGHT_GRAY, r)
            txt = self.fonts['small_font'].render(label, True, BLACK)
            panel.blit(txt, (r.x + (r.w - txt.get_width()) // 2,
                             r.y + (r.h - txt.get_height()) // 2))

        # Store absolute rects for click handling
        self._respond_rect_abs = pygame.Rect(PANEL_X + respond_rel.x, PANEL_Y + respond_rel.y,
                                             respond_rel.w, respond_rel.h)
        self._clear_rect_abs = pygame.Rect(PANEL_X + clear_rel.x, PANEL_Y + clear_rel.y,
                                           clear_rel.w, clear_rel.h)

        # Final blit
        screen.blit(panel, (PANEL_X, PANEL_Y))

    def handle_click(self, ui, pos):
        """
        Translate a mouse position (screen coords) into a semantic action.
        Args:
            - pos (tuple): (x, y) mouse position in screen coordinates.
        Returns:
            - str or None: Action string if a control was clicked, else None. str output depends on which control was clicked.

        """
        if self._close_rect_abs and self._close_rect_abs.collidepoint(pos):
            ui.selected_incident = -1
            return
        if self._respond_rect_abs and self._respond_rect_abs.collidepoint(pos):
            print('Respond to incident')
            return
        if self._clear_rect_abs and self._clear_rect_abs.collidepoint(pos):
            inc = ui.incidents[ui.selected_incident]
            key = (inc["drone"], inc["id"])
            ui._incident_cleared.add(key)       # remember it's cleared locally
            del ui.incidents[ui.selected_incident]
            # rebuild index map after deletion
            ui._incident_seen.clear()
            for i, it in enumerate(ui.incidents):
                ui._incident_seen[(it["drone"], it["id"])] = i
            ui.selected_incident = -1
            print('clear incident')
            ui.notification_ui.pushNotification("Cleared!", "Incident cleared!")
            return


def incidents_severity_index(incident):
    """
    Helper to convert incident['severity'] to index for severity_colors,
    clamping to valid range (0-based).
    Args:
        - incident (dict): Incident data dictionary.

    Returns:
        - int: Severity index (0-based).
    """
    sev = incident.get('severity', 1)
    try:
        sev = int(sev)
    except Exception:
        sev = 1
    # Map severity 1..N to index 0..N-1, clamp
    return max(0, sev - 1)
