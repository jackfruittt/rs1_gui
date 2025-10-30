
import pygame
from constants import state_colors, severity_colors, world_size, PINK
from utils import mapRange


class MapPanel:
    def __init__(self, app, map_top_view, drone_icon):
        self.app = app
        self.map_top_view = map_top_view
        self.drone_icon = drone_icon
        self.mapImgSize = map_top_view.get_size()

        # Load & scale incident icons
        self.person_icon = pygame.image.load("media/images/person.png").convert_alpha()
        self.fire_icon = pygame.image.load("media/images/fire.png").convert_alpha()
        self.garbage_icon = pygame.image.load("media/images/garbage.png").convert_alpha()
        self.warning_icon = pygame.image.load("media/images/warning.png").convert_alpha()

        self.person_icon = pygame.transform.scale(self.person_icon, (30, 40))
        self.fire_icon = pygame.transform.scale(self.fire_icon, (30, 40))
        self.garbage_icon = pygame.transform.scale(self.garbage_icon, (30, 35))
        self.warning_icon = pygame.transform.scale(self.warning_icon, (30, 30))

        self.icon_buttons = []
        self.highlighted_waypoints = []

        # Waypoint cycle state (triple overlap with T/3 staggering)
        self.waypoint_cycle_idx = 0
        self.waypoint_cycle_start = 0        # ms timestamp
        self.waypoint_cycle_duration = 1000   # ms per drone fade (full duration T)
        self.waypoint_initial_opacity = 128  # peak alpha (used for fade-in/out)
        self.waypoint_min_opacity = 0        # (ignored in in/out mode)

        # Cached waypoint layers: list of entries (per drone) -> {"surf": Surface, "pos": (x,y)}
        self.cachedWaypointSurfaces = []
        self._cachedWaypointSigs = []

        # One-time tinted icon caches
        self._incident_icon_cache = {}   # (kind, severity_idx) -> Surface
        self._drone_icon_tinted = {}     # state -> Surface

        # Reusable map panel surface
        self.map_panel_surface = pygame.Surface(self.mapImgSize, pygame.SRCALPHA).convert_alpha()

    # --------------------------------------------------------------------------------------
    # CACHING HELPERS
    # --------------------------------------------------------------------------------------
    def _build_wp_entry(self, waypoints):
        """Build a cropped waypoint surface and return {'surf': Surface, 'pos': (x,y)}."""
        if not waypoints:
            return None

        # Map to image coords once; compute bounds
        pts = []
        minx = miny = 10**9
        maxx = maxy = -10**9
        half_wx = world_size[0] / 2.0
        half_wy = world_size[1] / 2.0

        for wx, wy, _ in waypoints:
            x = mapRange(wx, -half_wx, half_wx, 0, self.mapImgSize[0])
            y = mapRange(wy, -half_wy, half_wy, 0, self.mapImgSize[1])
            xi, yi = int(x), int(y)
            pts.append((xi, yi))
            if xi < minx: minx = xi
            if yi < miny: miny = yi
            if xi > maxx: maxx = xi
            if yi > maxy: maxy = yi

        if not pts:
            return None

        PAD = 8  # radius/half-thickness padding
        tlx = max(0, minx - PAD)
        tly = max(0, miny - PAD)
        brx = min(self.mapImgSize[0], maxx + PAD)
        bry = min(self.mapImgSize[1], maxy + PAD)
        w = max(1, brx - tlx)
        h = max(1, bry - tly)

        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        R = 8
        TH = 4

        # Draw using local coords (crop offset)
        for i, (x, y) in enumerate(pts):
            lx, ly = x - tlx, y - tly
            pygame.draw.circle(surf, PINK, (lx, ly), R)
            if i > 0:
                px, py = pts[i - 1]
                pygame.draw.line(surf, PINK, (px - tlx, py - tly), (lx, ly), TH)

        # Close loop if more than 2 points
        if len(pts) > 2:
            (x0, y0) = pts[0]
            (xn, yn) = pts[-1]
            pygame.draw.line(surf, PINK, (xn - tlx, yn - tly), (x0 - tlx, y0 - tly), TH)

        return {"surf": surf, "pos": (tlx, tly)}

    def regenerate_cache_waypoint_surfaces(self):
        """Rebuild all waypoint surfaces from scratch."""
        self.cachedWaypointSurfaces = []
        self._cachedWaypointSigs = []
        for drone in getattr(self.app, "drones", []):
            wps = drone.get("waypoints", [])
            entry = self._build_wp_entry(wps) if wps else None
            self.cachedWaypointSurfaces.append(entry)
            sig = (len(wps), tuple((int(x*10), int(y*10)) for x, y, _ in wps))
            self._cachedWaypointSigs.append(sig)

    def _ensure_waypoint_cache(self):
        """Keep cached surfaces in sync with self.app.drones and their waypoint content."""
        drones = getattr(self.app, "drones", [])
        n = len(drones)

        # Resize arrays as needed
        if len(self.cachedWaypointSurfaces) != n:
            self.cachedWaypointSurfaces = [None] * n
        if len(self._cachedWaypointSigs) != n:
            self._cachedWaypointSigs = [None] * n

        # Update entries whose signature changed
        for i, d in enumerate(drones):
            wps = d.get("waypoints", [])
            sig = (len(wps), tuple((int(x*10), int(y*10)) for x, y, _ in wps))
            if self._cachedWaypointSigs[i] != sig:
                self.cachedWaypointSurfaces[i] = self._build_wp_entry(wps) if wps else None
                self._cachedWaypointSigs[i] = sig

    def _blit_cached_wp(self, map_panel, idx, alpha):
        """Blit a cached waypoint surface with the specified alpha (skip tiny alphas)."""
        if alpha <= 8:
            return
        if idx < 0 or idx >= len(self.cachedWaypointSurfaces):
            return
        entry = self.cachedWaypointSurfaces[idx]
        if not entry:
            return
        surf = entry["surf"]
        pos = entry["pos"]
        # Mutate alpha just-in-time; safe to call multiple times per frame
        surf.set_alpha(int(alpha))
        map_panel.blit(surf, pos)

    # --------------------------------------------------------------------------------------
    # ICON TINTING CACHES
    # --------------------------------------------------------------------------------------
    def _get_incident_icon(self, kind, severity_idx):
        key = (kind, severity_idx)
        if key in self._incident_icon_cache:
            return self._incident_icon_cache[key]

        base = {
            "Fire": self.fire_icon,
            "Person": self.person_icon,
            "Debris": self.garbage_icon,
            "Warn": self.warning_icon
        }[kind]

        colored = base.copy()
        px = pygame.PixelArray(colored)
        # Replace black with severity color (one-time)
        px.replace((0, 0, 0), severity_colors[severity_idx])
        del px
        self._incident_icon_cache[key] = colored
        return colored

    def _get_tinted_drone_icon(self, state):
        if state in self._drone_icon_tinted:
            return self._drone_icon_tinted[state]
        colored = self.drone_icon.copy()
        px = pygame.PixelArray(colored)
        # Replace white with state color (one-time)
        px.replace((255, 255, 255), state_colors.get(state, (255, 255, 255)))
        del px
        self._drone_icon_tinted[state] = colored
        return colored

    # --------------------------------------------------------------------------------------
    # (Legacy) draw_waypoints - kept for compatibility; cache builder uses a faster, cropped version
    # --------------------------------------------------------------------------------------
    def draw_waypoints(self, waypoints, opacity=255):
        CUSTOM_WAYPOINT_RADIUS = 8
        CUSTOM_PATH_THICKNESS = 4
        wp_surface = pygame.Surface((self.mapImgSize[0], self.mapImgSize[1]), pygame.SRCALPHA)

        for i in range(len(waypoints)):
            wx, wy, wz = waypoints[i]
            imgX = mapRange(wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            imgY = mapRange(wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

            # draw node
            pygame.draw.circle(wp_surface, PINK, (int(imgX), int(imgY)), CUSTOM_WAYPOINT_RADIUS)

            # connect to previous node
            if i > 0:
                prev_wx, prev_wy, prev_wz = waypoints[i-1]
                prev_imgX = mapRange(prev_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
                prev_imgY = mapRange(prev_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

                pygame.draw.line(wp_surface, PINK,
                                (int(prev_imgX), int(prev_imgY)),
                                (int(imgX), int(imgY)),
                                CUSTOM_PATH_THICKNESS)

        # close the loop: connect last → first
        if len(waypoints) > 2:
            first_wx, first_wy, first_wz = waypoints[0]
            last_wx, last_wy, last_wz = waypoints[-1]

            first_imgX = mapRange(first_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            first_imgY = mapRange(first_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
            last_imgX = mapRange(last_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            last_imgY = mapRange(last_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

            pygame.draw.line(wp_surface, PINK,
                            (int(last_imgX), int(last_imgY)),
                            (int(first_imgX), int(first_imgY)),
                            CUSTOM_PATH_THICKNESS)
        wp_surface.set_alpha(opacity)
        return wp_surface

    # --------------------------------------------------------------------------------------
    # MAIN DRAW
    # --------------------------------------------------------------------------------------
    def draw_map(self, robots, incidents, screen, selected_incident):
        """Draw map panel, waypoints, incidents, and robots."""
        PANEL_WIDTH, PANEL_HEIGHT = self.mapImgSize
        SCREEN_X = 380
        SCREEN_Y = 20

        self.icon_buttons = []

        # Reuse a single map panel surface
        map_panel = self.map_panel_surface
        map_panel.fill((0, 0, 0, 0))
        map_panel.blit(self.map_top_view, (0, 0))

        # Ensure waypoint cache is up to date (rebuild changed entries only)
        self._ensure_waypoint_cache()

        # ------------------------------------------------------------------
        # Triple-overlap waypoint rendering with T/3 staggering (no flash)
        # Opacity follows 0 -> max -> 0 triangular envelope over [0..1].
        # ------------------------------------------------------------------
        num_drones = len(self.app.drones) if hasattr(self.app, 'drones') else 0
        if num_drones > 0:
            now_ms = pygame.time.get_ticks()
            T = float(self.waypoint_cycle_duration)
            one_third = T / 3.0

            if self.waypoint_cycle_start == 0:
                self.waypoint_cycle_start = now_ms

            elapsed = now_ms - self.waypoint_cycle_start

            # When a full fade finishes, promote current -> next and
            # move the start forward by T/3 to preserve next's phase.
            if elapsed >= T:
                steps = int(elapsed // T)
                self.waypoint_cycle_idx = (self.waypoint_cycle_idx + steps) % num_drones
                self.waypoint_cycle_start += int(steps * one_third)
                elapsed = now_ms - self.waypoint_cycle_start

            def clamp01(x: float) -> float:
                return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

            # Fade mapping (0 -> max -> 0): triangular envelope across [0..1].
            # tri(frac) = 1 - |2*frac - 1|  -> 0 at 0 & 1, 1 at 0.5
            def fade_value(frac: float) -> int:
                f = clamp01(frac)
                tri = 1.0 - abs(2.0 * f - 1.0)
                return int(self.waypoint_initial_opacity * tri)

            cur_idx = self.waypoint_cycle_idx

            def draw_phase(idx_offset: int, delay_ms: float):
                e = elapsed - delay_ms
                if e < 0 or e > T:
                    return
                frac = e / T
                idx = (cur_idx + idx_offset) % num_drones
                self._blit_cached_wp(map_panel, idx, fade_value(frac))

            # Draw three overlapping phases (0, T/3, 2T/3)
            draw_phase(0, 0.0)
            draw_phase(1, one_third)
            draw_phase(2, 2.0 * one_third)

        # ------------------------------------------------------------------
        # Incidents (use pre-tinted icons; no per-frame recolor)
        # ------------------------------------------------------------------
        for i, inc in enumerate(incidents):
            severity_idx = max(0, min(len(severity_colors) - 1, inc["severity"] - 1))
            gps_coords = incidents[i]["drone_coords"]
            gps_x = float(gps_coords[0])
            gps_y = float(gps_coords[1])
            title = incidents[i]["title"]

            imgX = mapRange(gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
            imgY = mapRange(gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])

            kind = "Warn"
            if 'Fire' in title:
                kind = "Fire"
            elif 'Person' in title:
                kind = "Person"
            elif 'Debris' in title:
                kind = "Debris"

            icon = self._get_incident_icon(kind, severity_idx)
            map_panel.blit(icon, (imgX, imgY))

            icon_button = pygame.Rect(imgX+SCREEN_X, imgY+SCREEN_Y, icon.get_width(), icon.get_height())
            self.icon_buttons.append((icon_button, i))

        # ------------------------------------------------------------------
        # Robots (use pre-tinted icon per state; rotate per-frame)
        # ------------------------------------------------------------------
        for i in range(len(robots)):
            state = robots[i]["state"]

            # Parse GPS string to get coordinates
            gps_coords = robots[i]["gps"].split(', ')
            gps_x = float(gps_coords[1])
            gps_y = float(gps_coords[0])

            # Temporary Fix to align Drone Images on GUI Map to match Gazebo Positions
            imgX = mapRange(-gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[1]) - 28
            imgY = mapRange(-gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1]) - 55

            tinted = self._get_tinted_drone_icon(state)
            icon_rot = pygame.transform.rotate(tinted, robots[i]["yaw"])
            map_panel.blit(icon_rot, (imgX, imgY))

        # Blit the composed map panel to the main screen
        screen.blit(map_panel, (SCREEN_X, SCREEN_Y))

    def get_icon_buttons(self):
        return self.icon_buttons

    def processClickInMap(self, ui, gmx, gmy):
        # map incident icons
        if gmx > 380 and gmx < 1180 and gmy > 20 and gmy < 820:
            if ui.selected_drone < 0:
                for rect, idx in self.icon_buttons:
                    if rect.collidepoint((gmx, gmy)):
                        print(f'Icon incident clicked: {ui.incidents[idx]["title"]}')
                        ui.selected_incident = idx
                        break
            else:
                if ui.drone_control_panel.panelState == 2:
                    lx = gmx - 380
                    ly = gmy - 20
                    lx = mapRange(lx, 0, self.mapImgSize[0], -(world_size[0]/2), (world_size[0]/2))
                    ly = mapRange(ly, 0, self.mapImgSize[1], -(world_size[1]/2), (world_size[1]/2))
                    self.highlighted_waypoints.append([lx, ly, -999])
                    print(f'custom waypoint added at {lx}, {ly}')
