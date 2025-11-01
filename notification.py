from constants import *
import pygame

class NotificationUI:
    def __init__(self, fonts, pos=(760, 740), size=(400, 100),
                 image_path="media/images/drone_cp.png", image_size=(80, 80),
                 bar_color=BLUE):
        """
        Original __init__ function - Initialize NotificationUI with visuals, layout, and state.
        Sets up fonts, position/size, loads and scales the preview image, defines colors and layout
        metrics, and initializes dynamic text/state and small cached text surfaces.
        Args:
            - fonts (dict): Mapping of font names to pygame.Font objects used for notification text.
            - pos (tuple[int, int], optional): Top-left screen position of the notification panel. Defaults to (760, 740).
            - size (tuple[int, int], optional): Width and height of the notification panel. Defaults to (400, 100).
            - image_path (str, optional): Path to the preview image displayed in the panel. Defaults to "media/images/drone_cp.png".
            - image_size (tuple[int, int] | None, optional): Target (w, h) for the preview image; None keeps original size. Defaults to (80, 80).
            - bar_color (tuple[int, int, int], optional): RGB color of the progress/alert bar. Defaults to BLUE.
        Side Effects:
            - Loads and optionally rescales the image from disk.
            - Initializes timing parameters for display and fade, and creates text caches.
        Returns:
            - None
        """

        self.fonts = fonts
        self.pos = pos
        self.size = size

        # visuals
        self.image = pygame.image.load(image_path).convert_alpha()
        if image_size:
            self.image = pygame.transform.smoothscale(self.image, image_size)
        self.image_size = image_size
        self.bar_color = bar_color

        # layout
        self.pad = 10
        self.bar_h = 8

        # dynamic state
        self.title = ""
        self.details = ""
        self.duration = 2000        # ms
        self.fade_duration = 500    # ms
        self._t0 = None             # push time (ms)

        # tiny text cache
        self._title_surf = None
        self._details_surf = None

    # --- helpers -------------------------------------------------------------
    def _now(self): return pygame.time.get_ticks()
    def _clamp01(self, x): return 0.0 if x < 0 else (1.0 if x > 1.0 else x)
    def is_active(self):
        if self._t0 is None: return False
        return (self._now() - self._t0) < (self.duration + self.fade_duration)

    # --- API -----------------------------------------------------------------
    def pushNotification(self, title, details, duration=2000, fade_duration=500, bar_color=None):
        """
        Original pushNotification function - Trigger a new on-screen notification.
        Updates the title, details, color, and timing parameters for a notification,
        then regenerates the cached text surfaces for display.
        Args:
            - title (str): Main title text of the notification.
            - details (str): Secondary description or message body text.
            - duration (int, optional): Total time (ms) the notification remains visible. Defaults to 2000.
            - fade_duration (int, optional): Time (ms) for fade-out animation after visibility period. Defaults to 500.
            - bar_color (tuple[int, int, int] | None, optional): RGB color of the notification’s bar; keeps previous color if None.
        Side Effects:
            - Updates self.title, self.details, self.duration, self.fade_duration, and optionally self.bar_color.
            - Records timestamp self._t0 for tracking visibility timing.
            - Renders new text surfaces for title and details into self._title_surf and self._details_surf.
        Returns:
            - None
        """

        self.title = title
        self.details = details
        self.duration = int(duration)
        self.fade_duration = int(fade_duration)
        if bar_color is not None:
            self.bar_color = bar_color
        self._t0 = self._now()

        # refresh tiny text cache
        self._title_surf = self.fonts['inter_medium'].render(self.title, True, WHITE)
        self._details_surf = self.fonts['inter_small'].render(self.details, True, LIGHT_GRAY)

    # --- draw ----------------------------------------------------------------
    def drawNotifications(self, screen):
        """
        Original drawNotifications function - Render active notification panel with progress and fade.
        If a notification is active, builds a temporary panel surface with image, title, details,
        and a left-to-right progress bar, applies a global fade alpha after the duration, and blits
        the result to the provided screen surface.
        Args:
            - screen (pygame.Surface): The target surface to draw the notification onto.
        Returns:
            - None
        Side Effects:
            - Draws the notification UI elements onto the given screen surface.
        """

        if not self.is_active():
            return

        x, y = self.pos
        W, H = self.size
        t = self._now() - self._t0

        # progress fraction: 1 -> 0 over duration
        if t <= self.duration and self.duration > 0:
            frac = 1.0 - (t / self.duration)
        else:
            frac = 0.0

        # fade alpha after progress completes
        if t <= self.duration:
            alpha = 255
        else:
            f = (t - self.duration) / max(1, self.fade_duration)
            alpha = int(255 * self._clamp01(1.0 - f))

        # draw to a temporary surface so we can apply one alpha to the whole thing
        panel = pygame.Surface((W, H), pygame.SRCALPHA)

        rect = pygame.Rect(0, 0, W, H)
        pygame.draw.rect(panel, BLACK, rect)
        pygame.draw.rect(panel, WHITE, rect, 2)

        # IMAGE
        panel.blit(self.image, (self.pad, self.pad))

        # TEXT
        text_x = self.pad + (self.image_size[0] if self.image_size else 0) + self.pad
        if self._title_surf:
            panel.blit(self._title_surf, (text_x, self.pad))
        if self._details_surf:
            panel.blit(self._details_surf, (text_x, self.pad + 34))

        # PROGRESS (LEFT → RIGHT)
        bar_margin = 2
        total = pygame.Rect(bar_margin, H - self.bar_h - bar_margin, W - 2 * bar_margin, self.bar_h)
        if frac > 0:
            filled_w = int(total.width * frac)
            pygame.draw.rect(panel, self.bar_color,
                             (total.left, total.top, filled_w, total.height))

        # apply fade to whole panel
        if alpha < 255:
            panel.set_alpha(alpha)

        # blit to screen
        screen.blit(panel, (x, y))
