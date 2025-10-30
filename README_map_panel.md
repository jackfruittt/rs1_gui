# Map Panel Rendering (Waypoints, Fade, and Caching)

This README explains how `map_panel.py` renders drone waypoints with an **in–out fade** and **triple-overlap** scheduling, while keeping frame times low using cropped surface **caching** and **icon tint caches**. Each section cites exact line numbers from the file so you can jump straight to the code.

---

## 1) Key Parameters & State

The parameters that control the timing and opacity envelope live in the constructor:

```
  28:         # Waypoint cycle state (triple overlap with T/3 staggering)
  29:         self.waypoint_cycle_idx = 0
  30:         self.waypoint_cycle_start = 0        # ms timestamp
  31:         self.waypoint_cycle_duration = 1000   # ms per drone fade (full duration T)
  32:         self.waypoint_initial_opacity = 128  # peak alpha (used for fade-in/out)
  33:         self.waypoint_min_opacity = 0        # (ignored in in/out mode)
  34: 
  35:         # Cached waypoint layers: list of entries (per drone) -> {"surf": Surface, "pos": (x,y)}
  36:         self.cachedWaypointSurfaces = []
```

**What they do**
- `waypoint_cycle_duration` (`T`): Total time for one waypoint set’s fade cycle (in ms).
- `waypoint_initial_opacity`: **Peak** alpha of the triangular fade (0 → max → 0).
- Triple-overlap is achieved by staggering phases by `T/3` (see §2 & §3).

We also pre-allocate caches for performance and a reusable map panel surface:

```
  36:         self.cachedWaypointSurfaces = []
  37:         self._cachedWaypointSigs = []
  38: 
  39:         # One-time tinted icon caches
  40:         self._incident_icon_cache = {}   # (kind, severity_idx) -> Surface
  41:         self._drone_icon_tinted = {}     # state -> Surface
  42: 
  43:         # Reusable map panel surface
  44:         self.map_panel_surface = pygame.Surface(self.mapImgSize, pygame.SRCALPHA).convert_alpha()
  45: 
```

---

## 2) Fade Function (0 → max → 0)

Inside `draw_map()`, opacity follows a **triangular envelope**:
> tri(f) = 1 − |2f − 1|, where `f ∈ [0, 1]` is the normalized time within the cycle.

```
 267:             def clamp01(x: float) -> float:
 268:                 return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)
 269: 
 270:             # Fade mapping (0 -> max -> 0): triangular envelope across [0..1].
 271:             # tri(frac) = 1 - |2*frac - 1|  -> 0 at 0 & 1, 1 at 0.5
 272:             def fade_value(frac: float) -> int:
 273:                 f = clamp01(frac)
 274:                 tri = 1.0 - abs(2.0 * f - 1.0)
 275:                 return int(self.waypoint_initial_opacity * tri)
 276: 
 277:             cur_idx = self.waypoint_cycle_idx
```

**Math**
- `f = elapsed / T` (clamped to [0,1])
- `tri(f)` peaks at `f = 0.5` (alpha = `waypoint_initial_opacity`), and is 0 at `f = 0` and `f = 1`.

---

## 3) Overlap Scheduling (Three visible sets)

We render three overlapping phases with a **T/3** offset between them:
- Phase A: delay 0
- Phase B: delay `T/3`
- Phase C: delay `2T/3`

**Phase function**:

```
 279:             def draw_phase(idx_offset: int, delay_ms: float):
 280:                 e = elapsed - delay_ms
 281:                 if e < 0 or e > T:
 282:                     return
 283:                 frac = e / T
 284:                 idx = (cur_idx + idx_offset) % num_drones
 285:                 self._blit_cached_wp(map_panel, idx, fade_value(frac))
 286: 
 287:             # Draw three overlapping phases (0, T/3, 2T/3)
```

**Phase calls**:

```
 288:             draw_phase(0, 0.0)
 289:             draw_phase(1, one_third)
 290:             draw_phase(2, 2.0 * one_third)
```

**No‑flash handoff**  
When a full cycle completes, we **advance the index** and **shift the timebase** by `T/3`. That way, the “next” set doesn’t jump back to full opacity when it becomes “current” — its phase continues smoothly.

```
 259:             # When a full fade finishes, promote current -> next and
 260:             # move the start forward by T/3 to preserve next's phase.
 261:             if elapsed >= T:
 262:                 steps = int(elapsed // T)
 263:                 self.waypoint_cycle_idx = (self.waypoint_cycle_idx + steps) % num_drones
 264:                 self.waypoint_cycle_start += int(steps * one_third)
 265:                 elapsed = now_ms - self.waypoint_cycle_start
 266: 
```

---

## 4) Waypoint Caching (cropped surfaces)

Each drone’s waypoints are drawn **once** into a tightly‑cropped surface and cached, so render-time only needs a quick `set_alpha()` + `blit()` of a small texture.

**Builder:** computes the bounding box, crops, and draws locally into a surface.

```
  49:     def _build_wp_entry(self, waypoints):
  50:         """Build a cropped waypoint surface and return {'surf': Surface, 'pos': (x,y)}."""
  51:         if not waypoints:
  52:             return None
  53: 
  54:         # Map to image coords once; compute bounds
  55:         pts = []
  56:         minx = miny = 10**9
  57:         maxx = maxy = -10**9
  58:         half_wx = world_size[0] / 2.0
  59:         half_wy = world_size[1] / 2.0
  60: 
  61:         for wx, wy, _ in waypoints:
  62:             x = mapRange(wx, -half_wx, half_wx, 0, self.mapImgSize[0])
  63:             y = mapRange(wy, -half_wy, half_wy, 0, self.mapImgSize[1])
  64:             xi, yi = int(x), int(y)
  65:             pts.append((xi, yi))
  66:             if xi < minx: minx = xi
  67:             if yi < miny: miny = yi
  68:             if xi > maxx: maxx = xi
  69:             if yi > maxy: maxy = yi
  70: 
  71:         if not pts:
  72:             return None
  73: 
  74:         PAD = 8  # radius/half-thickness padding
  75:         tlx = max(0, minx - PAD)
  76:         tly = max(0, miny - PAD)
  77:         brx = min(self.mapImgSize[0], maxx + PAD)
  78:         bry = min(self.mapImgSize[1], maxy + PAD)
  79:         w = max(1, brx - tlx)
  80:         h = max(1, bry - tly)
  81: 
  82:         surf = pygame.Surface((w, h), pygame.SRCALPHA)
  83:         R = 8
  84:         TH = 4
  85: 
  86:         # Draw using local coords (crop offset)
  87:         for i, (x, y) in enumerate(pts):
  88:             lx, ly = x - tlx, y - tly
  89:             pygame.draw.circle(surf, PINK, (lx, ly), R)
  90:             if i > 0:
  91:                 px, py = pts[i - 1]
  92:                 pygame.draw.line(surf, PINK, (px - tlx, py - tly), (lx, ly), TH)
  93: 
  94:         # Close loop if more than 2 points
  95:         if len(pts) > 2:
  96:             (x0, y0) = pts[0]
  97:             (xn, yn) = pts[-1]
  98:             pygame.draw.line(surf, PINK, (xn - tlx, yn - tly), (x0 - tlx, y0 - tly), TH)
  99: 
 100:         return {"surf": surf, "pos": (tlx, tly)}
 101: 
```

**Cache maintenance:** ensures the cache size matches the number of drones and (re)builds entries only when waypoints change (via a lightweight signature).

```
 113:     def _ensure_waypoint_cache(self):
 114:         """Keep cached surfaces in sync with self.app.drones and their waypoint content."""
 115:         drones = getattr(self.app, "drones", [])
 116:         n = len(drones)
 117: 
 118:         # Resize arrays as needed
 119:         if len(self.cachedWaypointSurfaces) != n:
 120:             self.cachedWaypointSurfaces = [None] * n
 121:         if len(self._cachedWaypointSigs) != n:
 122:             self._cachedWaypointSigs = [None] * n
 123: 
 124:         # Update entries whose signature changed
 125:         for i, d in enumerate(drones):
 126:             wps = d.get("waypoints", [])
 127:             sig = (len(wps), tuple((int(x*10), int(y*10)) for x, y, _ in wps))
 128:             if self._cachedWaypointSigs[i] != sig:
 129:                 self.cachedWaypointSurfaces[i] = self._build_wp_entry(wps) if wps else None
 130:                 self._cachedWaypointSigs[i] = sig
 131: 
```

**Blitter:** applies per‑frame opacity and blits at the cached top‑left position. Very cheap.

```
 132:     def _blit_cached_wp(self, map_panel, idx, alpha):
 133:         """Blit a cached waypoint surface with the specified alpha (skip tiny alphas)."""
 134:         if alpha <= 8:
 135:             return
 136:         if idx < 0 or idx >= len(self.cachedWaypointSurfaces):
 137:             return
 138:         entry = self.cachedWaypointSurfaces[idx]
 139:         if not entry:
 140:             return
 141:         surf = entry["surf"]
 142:         pos = entry["pos"]
 143:         # Mutate alpha just-in-time; safe to call multiple times per frame
 144:         surf.set_alpha(int(alpha))
 145:         map_panel.blit(surf, pos)
 146: 
 147:     # --------------------------------------------------------------------------------------
 148:     # ICON TINTING CACHES
 149:     # --------------------------------------------------------------------------------------
```

**Where cache is refreshed each frame:**

```
 240: 
 241:         # Ensure waypoint cache is up to date (rebuild changed entries only)
 242:         self._ensure_waypoint_cache()
 243: 
 244:         # ------------------------------------------------------------------
```

---

## 5) Icon Tint Caches (one‑time recolor)

Per‑frame pixel recoloring is expensive. We recolor **once** and reuse:
- Incidents: black → severity color
- Drones: white → state color

**Incident icon cache:**

```
 150:     def _get_incident_icon(self, kind, severity_idx):
 151:         key = (kind, severity_idx)
 152:         if key in self._incident_icon_cache:
 153:             return self._incident_icon_cache[key]
 154: 
 155:         base = {
 156:             "Fire": self.fire_icon,
 157:             "Person": self.person_icon,
 158:             "Debris": self.garbage_icon,
 159:             "Warn": self.warning_icon
 160:         }[kind]
 161: 
 162:         colored = base.copy()
 163:         px = pygame.PixelArray(colored)
 164:         # Replace black with severity color (one-time)
 165:         px.replace((0, 0, 0), severity_colors[severity_idx])
 166:         del px
 167:         self._incident_icon_cache[key] = colored
 168:         return colored
 169: 
```

**Drone icon cache:**

```
 170:     def _get_tinted_drone_icon(self, state):
 171:         if state in self._drone_icon_tinted:
 172:             return self._drone_icon_tinted[state]
 173:         colored = self.drone_icon.copy()
 174:         px = pygame.PixelArray(colored)
 175:         # Replace white with state color (one-time)
 176:         px.replace((255, 255, 255), state_colors.get(state, (255, 255, 255)))
 177:         del px
 178:         self._drone_icon_tinted[state] = colored
 179:         return colored
 180: 
 181:     # --------------------------------------------------------------------------------------
 182:     # (Legacy) draw_waypoints - kept for compatibility; cache builder uses a faster, cropped version
 183:     # --------------------------------------------------------------------------------------
```

These are then used in `draw_map()` when blitting incidents and drones.

---

## 6) Coordinate Mapping

World coordinates (meters) are mapped to image coordinates (pixels) using `mapRange()`, consistently throughout the file. Here’s an example in the waypoint cache builder:

```
  58:         half_wx = world_size[0] / 2.0
  59:         half_wy = world_size[1] / 2.0
  60: 
  61:         for wx, wy, _ in waypoints:
  62:             x = mapRange(wx, -half_wx, half_wx, 0, self.mapImgSize[0])
  63:             y = mapRange(wy, -half_wy, half_wy, 0, self.mapImgSize[1])
  64:             xi, yi = int(x), int(y)
  65:             pts.append((xi, yi))
  66:             if xi < minx: minx = xi
```

And in the main draw loop for incidents/drones, the same mapping is applied to convert GPS/world space onto the 2D map image.

---

## 7) Main Draw Flow

High‑level sequence inside `draw_map()`:

1. Clear and blit the background into the reusable `map_panel_surface`.
2. Ensure waypoint cache is aligned with the current `app.drones` and refresh only changed entries.
3. Compute elapsed time & schedule triple‑overlap phases.
4. For each visible phase, compute normalized time `f`, map to alpha via triangular envelope, and `_blit_cached_wp(...)`.
5. Blit incidents (pre‑tinted icons) and drones (pre‑tinted + rotated).
6. Blit the composed map panel to the main screen.

See the full `draw_map()` body for context:

```
 228:     def draw_map(self, robots, incidents, screen, selected_incident):
 229:         """Draw map panel, waypoints, incidents, and robots."""
 230:         PANEL_WIDTH, PANEL_HEIGHT = self.mapImgSize
 231:         SCREEN_X = 380
 232:         SCREEN_Y = 20
 233: 
 234:         self.icon_buttons = []
 235: 
 236:         # Reuse a single map panel surface
 237:         map_panel = self.map_panel_surface
 238:         map_panel.fill((0, 0, 0, 0))
 239:         map_panel.blit(self.map_top_view, (0, 0))
 240: 
 241:         # Ensure waypoint cache is up to date (rebuild changed entries only)
 242:         self._ensure_waypoint_cache()
 243: 
 244:         # ------------------------------------------------------------------
 245:         # Triple-overlap waypoint rendering with T/3 staggering (no flash)
 246:         # Opacity follows 0 -> max -> 0 triangular envelope over [0..1].
 247:         # ------------------------------------------------------------------
 248:         num_drones = len(self.app.drones) if hasattr(self.app, 'drones') else 0
 249:         if num_drones > 0:
 250:             now_ms = pygame.time.get_ticks()
 251:             T = float(self.waypoint_cycle_duration)
 252:             one_third = T / 3.0
 253: 
 254:             if self.waypoint_cycle_start == 0:
 255:                 self.waypoint_cycle_start = now_ms
 256: 
 257:             elapsed = now_ms - self.waypoint_cycle_start
 258: 
 259:             # When a full fade finishes, promote current -> next and
 260:             # move the start forward by T/3 to preserve next's phase.
 261:             if elapsed >= T:
 262:                 steps = int(elapsed // T)
 263:                 self.waypoint_cycle_idx = (self.waypoint_cycle_idx + steps) % num_drones
 264:                 self.waypoint_cycle_start += int(steps * one_third)
 265:                 elapsed = now_ms - self.waypoint_cycle_start
 266: 
 267:             def clamp01(x: float) -> float:
 268:                 return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)
 269: 
 270:             # Fade mapping (0 -> max -> 0): triangular envelope across [0..1].
 271:             # tri(frac) = 1 - |2*frac - 1|  -> 0 at 0 & 1, 1 at 0.5
 272:             def fade_value(frac: float) -> int:
 273:                 f = clamp01(frac)
 274:                 tri = 1.0 - abs(2.0 * f - 1.0)
 275:                 return int(self.waypoint_initial_opacity * tri)
 276: 
 277:             cur_idx = self.waypoint_cycle_idx
 278: 
 279:             def draw_phase(idx_offset: int, delay_ms: float):
 280:                 e = elapsed - delay_ms
 281:                 if e < 0 or e > T:
 282:                     return
 283:                 frac = e / T
 284:                 idx = (cur_idx + idx_offset) % num_drones
 285:                 self._blit_cached_wp(map_panel, idx, fade_value(frac))
 286: 
 287:             # Draw three overlapping phases (0, T/3, 2T/3)
 288:             draw_phase(0, 0.0)
 289:             draw_phase(1, one_third)
 290:             draw_phase(2, 2.0 * one_third)
 291: 
 292:         # ------------------------------------------------------------------
 293:         # Incidents (use pre-tinted icons; no per-frame recolor)
 294:         # ------------------------------------------------------------------
 295:         for i, inc in enumerate(incidents):
 296:             severity_idx = max(0, min(len(severity_colors) - 1, inc["severity"] - 1))
 297:             gps_coords = incidents[i]["drone_coords"]
 298:             gps_x = float(gps_coords[0])
 299:             gps_y = float(gps_coords[1])
 300:             title = incidents[i]["title"]
 301: 
 302:             imgX = mapRange(gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
 303:             imgY = mapRange(gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
 304: 
 305:             kind = "Warn"
 306:             if 'Fire' in title:
 307:                 kind = "Fire"
 308:             elif 'Person' in title:
 309:                 kind = "Person"
 310:             elif 'Debris' in title:
 311:                 kind = "Debris"
 312: 
 313:             icon = self._get_incident_icon(kind, severity_idx)
 314:             map_panel.blit(icon, (imgX, imgY))
 315: 
 316:             icon_button = pygame.Rect(imgX+SCREEN_X, imgY+SCREEN_Y, icon.get_width(), icon.get_height())
 317:             self.icon_buttons.append((icon_button, i))
 318: 
 319:         # ------------------------------------------------------------------
 320:         # Robots (use pre-tinted icon per state; rotate per-frame)
 321:         # ------------------------------------------------------------------
 322:         for i in range(len(robots)):
 323:             state = robots[i]["state"]
 324: 
 325:             # Parse GPS string to get coordinates
 326:             gps_coords = robots[i]["gps"].split(', ')
 327:             gps_x = float(gps_coords[1])
 328:             gps_y = float(gps_coords[0])
 329: 
 330:             # Temporary Fix to align Drone Images on GUI Map to match Gazebo Positions
 331:             imgX = mapRange(-gps_x, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[1]) - 28
 332:             imgY = mapRange(-gps_y, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1]) - 55
 333: 
 334:             tinted = self._get_tinted_drone_icon(state)
 335:             icon_rot = pygame.transform.rotate(tinted, robots[i]["yaw"])
 336:             map_panel.blit(icon_rot, (imgX, imgY))
 337: 
 338:         # Blit the composed map panel to the main screen
 339:         screen.blit(map_panel, (SCREEN_X, SCREEN_Y))
 340: 
```

---

## 8) Tuning Knobs

- **Speed:** `waypoint_cycle_duration` (L28) — shorter is faster.
- **Peak brightness:** `waypoint_initial_opacity` (L31).
- **Overlap feel:** Keep T/3 staggering for three sets; for two sets, use T/2.
- **Skip tiny alphas:** `_blit_cached_wp` threshold (L134) can be increased from 8 → 16 to shave a bit more work.

---

## 9) Common Pitfalls

- **IndexError on cache:** Always call `_ensure_waypoint_cache()` before drawing (L242). Never index `cachedWaypointSurfaces` directly; use `_blit_cached_wp()`.
- **Alpha chain mistake:** `Surface.set_alpha()` returns `None`. Don’t do `.copy().set_alpha(...)`; set alpha on a Surface and then blit it.
- **Per‑frame recolor:** Avoid `PixelArray.replace(...)` during the draw loop; use the caches (L150–183).

---

## 10) Glossary

- **T (cycle duration):** `waypoint_cycle_duration` in ms (L30).
- **f (phase):** `elapsed / T`, clamped to [0,1] (L267).
- **triangular envelope:** `1 − |2f − 1|` (L274)—peaks in the middle, zero at ends.
- **Phase staggering:** delays of `0, T/3, 2T/3` (L288–L290).

---

## Appendix: Legacy Full‑Panel Waypoint Drawer

Kept for compatibility/reference; real‑time code uses the cropped cache builder instead.

```
 184:     def draw_waypoints(self, waypoints, opacity=255):
 185:         CUSTOM_WAYPOINT_RADIUS = 8
 186:         CUSTOM_PATH_THICKNESS = 4
 187:         wp_surface = pygame.Surface((self.mapImgSize[0], self.mapImgSize[1]), pygame.SRCALPHA)
 188: 
 189:         for i in range(len(waypoints)):
 190:             wx, wy, wz = waypoints[i]
 191:             imgX = mapRange(wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
 192:             imgY = mapRange(wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
 193: 
 194:             # draw node
 195:             pygame.draw.circle(wp_surface, PINK, (int(imgX), int(imgY)), CUSTOM_WAYPOINT_RADIUS)
 196: 
 197:             # connect to previous node
 198:             if i > 0:
 199:                 prev_wx, prev_wy, prev_wz = waypoints[i-1]
 200:                 prev_imgX = mapRange(prev_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
 201:                 prev_imgY = mapRange(prev_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
 202: 
 203:                 pygame.draw.line(wp_surface, PINK,
 204:                                 (int(prev_imgX), int(prev_imgY)),
 205:                                 (int(imgX), int(imgY)),
 206:                                 CUSTOM_PATH_THICKNESS)
 207: 
 208:         # close the loop: connect last → first
 209:         if len(waypoints) > 2:
 210:             first_wx, first_wy, first_wz = waypoints[0]
 211:             last_wx, last_wy, last_wz = waypoints[-1]
 212: 
 213:             first_imgX = mapRange(first_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
 214:             first_imgY = mapRange(first_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
 215:             last_imgX = mapRange(last_wx, -(world_size[0]/2), (world_size[0]/2), 0, self.mapImgSize[0])
 216:             last_imgY = mapRange(last_wy, -(world_size[1]/2), (world_size[1]/2), 0, self.mapImgSize[1])
 217: 
 218:             pygame.draw.line(wp_surface, PINK,
 219:                             (int(last_imgX), int(last_imgY)),
 220:                             (int(first_imgX), int(first_imgY)),
 221:                             CUSTOM_PATH_THICKNESS)
 222:         wp_surface.set_alpha(opacity)
 223:         return wp_surface
 224: 
 225:     # --------------------------------------------------------------------------------------
 226:     # MAIN DRAW
 227:     # --------------------------------------------------------------------------------------
```
