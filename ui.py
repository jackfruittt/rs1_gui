import pygame
import cv2
import numpy as np
import sys

pretty = True  # Set to True for feathered edges, False for solid edges

# Hard-coded telemetry data
import random

robots = [
    { # robot 1
        "battery": "92%",
        "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
        "altitude": "102m",
        "state": "Offline",
        "setPose": "-",
        "nearPose": "-",
        "yaw": round(random.uniform(0, 360), 1)
    },
    { # robot 2
        "battery": "64%",
        "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
        "altitude": "87m",
        "state": "Scouting",
        "setPose": "-",
        "nearPose": "Lake",
        "yaw": round(random.uniform(0, 360), 1)
    },
    { # robot 3
        "battery": "78%",
        "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
        "altitude": "56m",
        "state": "Idle",
        "setPose": "-",
        "nearPose": "-",
        "yaw": round(random.uniform(0, 360), 1)
    },
    { # robot 4
        "battery": "51%",
        "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
        "altitude": "73m",
        "state": "Piloting",
        "setPose": "-",
        "nearPose": "Parking",
        "yaw": round(random.uniform(0, 360), 1)
    },
    { # robot 5
        "battery": "88%",
        "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
        "altitude": "64m",
        "state": "Responding",
        "setPose": "-",
        "nearPose": "-",
        "yaw": round(random.uniform(0, 360), 1)
    },
    { # robot 6
        "battery": "33%",
        "gps": f"{round(random.uniform(-400, 400), 3)}, {round(random.uniform(-400, 400), 3)}",
        "altitude": "45m",
        "state": "Offline",
        "setPose": "-",
        "nearPose": "Base",
        "yaw": round(random.uniform(0, 360), 1)
    }
]



# Store clickable card rects globally or pass them around
robot_card_rects = []  

selected_robot = -1

def render_robots_list(robots):
    global robot_card_rects
    robot_card_rects = []  # reset each frame

    panel_width, panel_height = 350, 700
    card_height = 90
    margin = 15

    panel_surface = pygame.Surface((panel_width, panel_height)).convert_alpha()
    panel_surface.fill((0, 0, 0, 0))  # transparent

    for i, robot in enumerate(robots):
        y = i * (card_height + margin)
        if y + card_height > panel_height:
            break

        # Card rect (for detecting clicks)
        card_rect = pygame.Rect(10, 10 + y, panel_width, card_height)  # position matches blit later
        robot_card_rects.append((card_rect, i))  # store index with rect

        # Draw card background and border
        pygame.draw.rect(panel_surface, (0, 0, 0), (0, y, panel_width, card_height))
        pygame.draw.rect(panel_surface, (255, 255, 255), (0, y, panel_width, card_height), 2)

        # Robot ID
        id_text = inter_bold_large.render(f"#{i+1}", True, (255, 255, 255))
        panel_surface.blit(id_text, (10, y + 10))

        # Robot State (color-coded)
        state = robot["state"]
        state_color = state_colors.get(state, (255, 255, 255))
        state_text = inter_bold_medium.render(state, True, state_color)
        panel_surface.blit(state_text, (90, y + 10))

        if state == "Offline":
            last_seen_text = inter_smaller.render("last seen:", True, (255, 255, 255))
            panel_surface.blit(last_seen_text, (90 + state_text.get_width() + 5, y + 22))

        # Location or Pose
        if robot.get("nearPose") and robot["nearPose"] != "-":
            loc_str = f"Near: {robot['nearPose']}"
        else:
            loc_str = f"{robot['gps']}, {robot['altitude']}"
        loc_text = inter_small.render(loc_str, True, (255, 255, 255))
        panel_surface.blit(loc_text, (90, y + 45))

        # Battery
        batt_text = inter_smaller.render(f"{robot['battery']} battery", True, (255, 255, 255))
        panel_surface.blit(batt_text, (90, y + 70))

    screen.blit(panel_surface, (10, 10))


# --- Function to create the feathered image ---
def feather_image(surface, feather_size_x, feather_size_y,
                  feather_top=False, feather_right=True,
                  feather_bottom=True, feather_left=False):
    """
    Feathers selected edges of a Pygame surface, if global `pretty` is True.

    Args:
        surface (pygame.Surface): The image surface to feather.
        feather_size_x (int): The width of the feathering effect (used for left/right).
        feather_size_y (int): The height of the feathering effect (used for top/bottom).
        feather_top (bool): Feather the top edge.
        feather_right (bool): Feather the right edge.
        feather_bottom (bool): Feather the bottom edge.
        feather_left (bool): Feather the left edge.

    Returns:
        pygame.Surface: A new surface with the feathered effect (or original if pretty is False).
    """
    global pretty
    if not pretty:
        return surface

    feathered_surface = surface.copy().convert_alpha()
    width, height = feathered_surface.get_size()

    alpha_mask = pygame.Surface((width, height), pygame.SRCALPHA)
    alpha_mask.fill((255, 255, 255, 255))

    alpha_pixels_view = pygame.surfarray.pixels_alpha(alpha_mask)

    h_gradient = np.linspace(255, 0, feather_size_x) if feather_right or feather_left else None
    v_gradient = np.linspace(255, 0, feather_size_y) if feather_top or feather_bottom else None

    for x in range(width):
        for y in range(height):
            alpha = alpha_pixels_view[x, y]

            if feather_right and x >= width - feather_size_x:
                grad_index = x - (width - feather_size_x)
                alpha = min(alpha, h_gradient[grad_index])

            if feather_left and x < feather_size_x:
                grad_index = feather_size_x - 1 - x
                alpha = min(alpha, h_gradient[grad_index])

            if feather_bottom and y >= height - feather_size_y:
                grad_index = y - (height - feather_size_y)
                alpha = min(alpha, v_gradient[grad_index])

            if feather_top and y < feather_size_y:
                grad_index = feather_size_y - 1 - y
                alpha = min(alpha, v_gradient[grad_index])

            alpha_pixels_view[x, y] = int(alpha)

    del alpha_pixels_view
    feathered_surface.blit(alpha_mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

    return feathered_surface

def mapRange(value, inMin, inMax, outMin, outMax): # https://stackoverflow.com/a/68722109/9797303
    return outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))

scrolling = False
scoll_speed = 5
scroll_direction = 1

fade_transition_duration = 300
show_left_fade = False
left_fade_start = 0
left_fadeIn = True

pygame.init()
screen = pygame.display.set_mode((1900, 860))
pygame.display.set_caption("RS1 GUI")
clock = pygame.time.Clock()

# Fonts and colors
font = pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 32)
small_font = pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 20)
large_font = pygame.font.Font("media/fonts/TurretRoad-ExtraBold.otf", 48)
state_font = pygame.font.Font("media/fonts/boston.ttf", 38)

inter_large   = pygame.font.Font("media/fonts/Inter-Regular.ttf", 48)
inter_medium  = pygame.font.Font("media/fonts/Inter-Regular.ttf", 32)
inter_small   = pygame.font.Font("media/fonts/Inter-Regular.ttf", 18)
inter_smaller = pygame.font.Font("media/fonts/Inter-Regular.ttf", 16)
inter_bold_large  = pygame.font.Font("media/fonts/Inter-Bold.ttf", 48)
inter_bold_medium = pygame.font.Font("media/fonts/Inter-Bold.ttf", 32)

WHITE = (255, 255, 255)
LIGHT_GRAY = (200, 200, 200)
GRAY = (50, 50, 50)
DARK_GRAY = (30, 30, 30)
BLACK = (0, 0, 0)
BLUE = (100, 170, 255)
RED = (255, 100, 100)
YELLOW = (255, 200, 100)
GREEN = (100, 255, 100)

state_colors = {
    "Offline":   LIGHT_GRAY,
    "Scouting":  BLUE,
    "Idle":      RED,
    "Piloting":  GREEN,
    "Responding":YELLOW
}

severity_colors = (GREEN, YELLOW, RED)
severity_msg = ("Safe", "Warning", "Critical")

forest_splash = pygame.image.load("media/images/forest_splash.jpg").convert()
splash_rect = forest_splash.get_rect(center=(640, 360))

map_top_view = pygame.image.load("media/images/bird_view.png").convert()
mapImgSize = map_top_view.get_size()

feathered_splash = feather_image(forest_splash, 150, 150, feather_top=False, feather_right=True, feather_bottom=True)

#video
# 360p video
video_path = "media/images/snake.mp4"
cap = cv2.VideoCapture(video_path)

# Get video properties
fps = cap.get(cv2.CAP_PROP_FPS)
frame_delay = int(1000 / fps) if fps > 0 else 33
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

world_size = (800, 800)
drone_icon = pygame.image.load("media/images/drone.png").convert_alpha()
drone_icon = pygame.transform.scale(drone_icon, (35,35))


# Dummy incidents
incidents = [
    {
        "id": 1, "title": "Fire Detected", "time": "2025-08-01 13:20", "severity": 3,
        "Platform": 1, "robot_coords": (-0.005, 0.002), "global_coords": (-33.8701, 151.2101)
    },
    {
        "id": 2, "title": "Fungal Detected", "time": "2025-08-01 13:45", "severity": 1,
        "Platform": 2, "robot_coords": (0.003, -0.001), "global_coords": (-33.8670, 151.2070)
    },
    {
        "id": 3, "title": "Bear Detected", "time": "2025-08-01 14:05", "severity": 3,
        "Platform": 1, "robot_coords": (-0.002, 0.004), "global_coords": (-33.8690, 151.2088)
    },
    {
        "id": 4, "title": "Flood Warning", "time": "2025-08-01 14:30", "severity": 2,
        "Platform": 2, "robot_coords": (0.001, 0.003), "global_coords": (-33.8665, 151.2065)
    },
    {
        "id": 5, "title": "Unauthorized Entry", "time": "2025-08-01 15:00", "severity": 3,
        "Platform": 1, "robot_coords": (-0.004, -0.002), "global_coords": (-33.8685, 151.2095)
    },
    {
        "id": 6, "title": "Smoke Alert", "time": "2025-08-01 15:20", "severity": 2,
        "Platform": 2, "robot_coords": (0.002, 0.001), "global_coords": (-33.8678, 151.2079)
    },
    {
        "id": 7, "title": "Gas Leak Detected", "time": "2025-08-01 15:45", "severity": 3,
        "Platform": 1, "robot_coords": (-0.001, -0.003), "global_coords": (-33.8693, 151.2083)
    },
    {
        "id": 8, "title": "Tree Fallen", "time": "2025-08-01 16:10", "severity": 1,
        "Platform": 2, "robot_coords": (0.004, -0.004), "global_coords": (-33.8659, 151.2062)
    },
    {
        "id": 9, "title": "Animal Intrusion", "time": "2025-08-01 16:35", "severity": 2,
        "Platform": 1, "robot_coords": (-0.003, 0.001), "global_coords": (-33.8682, 151.2090)
    },
    {
        "id": 10, "title": "Power Outage", "time": "2025-08-01 17:00", "severity": 2,
        "Platform": 2, "robot_coords": (0.002, -0.002), "global_coords": (-33.8667, 151.2072)
    },
]

incident_y = 80 # starting y position for incident list

# Popup state
popup_visible = False
selected_incident = None
popup_scale = 0
animating_in = False
animating_out = False
anim_start_time = 0
ANIM_DURATION = 200  # milliseconds

last_good_frame_surface = None
def video_loop():
    global last_good_frame_surface

    ret, frame = cap.read()

    if not ret:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        # Don't read again immediately — we'll just reuse last frame
    else:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_surface = pygame.surfarray.make_surface(np.flipud(np.rot90(frame_rgb)))
        last_good_frame_surface = frame_surface

    # Always blit the last good frame if we have one
    if last_good_frame_surface is not None:
        # screen.blit(feather_image(last_good_frame_surface, 150, 150, feather_top=False, feather_right=True, feather_bottom=True), (20, 20))
        screen.blit(last_good_frame_surface, (20, 20))

def draw_left_fade():
    l_fade_surface = pygame.Surface((670, 395), pygame.SRCALPHA)
    l_fade_surface.fill((225, 255, 255, 108))  # Transparent background
    screen.blit(l_fade_surface, (0, 0))


def drone_ui():
    """Draw Drone panel and handle click detection."""

    # Panel dimensions & position
    PANEL_WIDTH = 440
    PANEL_HEIGHT = 430
    PANEL_X = 1460
    PANEL_Y = 0

    # Button dimensions
    BTN_WIDTH = 180
    BTN_HEIGHT = 80
    BTN_SPACING_X = 20
    BTN_SPACING_Y = 20

    # Button definitions
    buttons = [
        {"label": "HOVER", "x": 20, "y": 80},
        {"label": "LAND",  "x": 20 + BTN_WIDTH + BTN_SPACING_X, "y": 80},
        {"label": "SCOUT", "x": 20, "y": 80 + BTN_HEIGHT + BTN_SPACING_Y},
        {"label": "PILOT", "x": 20 + BTN_WIDTH + BTN_SPACING_X, "y": 80 + BTN_HEIGHT + BTN_SPACING_Y},
        {"label": "Close", "x": (PANEL_WIDTH - BTN_WIDTH) // 2, "y": 80 + (BTN_HEIGHT + BTN_SPACING_Y) * 2}
    ]

    # Precompute button rects relative to panel
    for btn in buttons:
        btn["rect"] = pygame.Rect(btn["x"], btn["y"], BTN_WIDTH, BTN_HEIGHT)

    # Draw panel surface
    drone_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT))
    drone_panel.fill(BLACK)

    # Title
    title_surface = large_font.render("Drone", True, WHITE)
    title_rect = title_surface.get_rect(topright=(PANEL_WIDTH - 10, 10))
    drone_panel.blit(title_surface, title_rect)

    # Draw buttons
    for btn in buttons:
        pygame.draw.rect(drone_panel, WHITE, btn["rect"], 2)
        font_to_use = inter_medium if btn["label"] != "Close" else inter_medium
        label_surface = font_to_use.render(btn["label"], True, WHITE)
        label_rect = label_surface.get_rect(center=btn["rect"].center)
        drone_panel.blit(label_surface, label_rect)

    # Blit panel to screen
    screen.blit(drone_panel, (PANEL_X, PANEL_Y))


def draw_incidents_panel():
    global incident_y, incident_card_rects, incident_scroll_up_rect, incident_scroll_down_rect
    PANEL_WIDTH = 440
    PANEL_HEIGHT = 430
    PANEL_X = 1460
    PANEL_Y = 0

    # Prepare list for click detection
    incident_card_rects = []

    # Surface
    incidents_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT), pygame.SRCALPHA)
    incidents_panel.fill((0, 0, 0, 0))  # Transparent background

    # Scroll state
    if scrolling:
        incident_y += scroll_direction * scoll_speed

    # Incident entries
    for i, inc in enumerate(incidents):
        y = incident_y + i * 90
        # Only process and add clicks if within visible area
        card_rect_abs = pygame.Rect(PANEL_X + 20, PANEL_Y + y, PANEL_WIDTH - 80, 80)

        pygame.draw.rect(incidents_panel, LIGHT_GRAY, (20, y, PANEL_WIDTH - 80, 80))
        pygame.draw.rect(incidents_panel, DARK_GRAY, (30, y + 10, 60, 60))  # Image placeholder
        pygame.draw.rect(incidents_panel, severity_colors[inc["severity"] - 1], (20, y + 75, PANEL_WIDTH - 80, 5))

        t1 = small_font.render(f'#{i+1} {inc["title"]}', True, BLACK)
        t2 = small_font.render(inc["time"], True, BLACK)
        incidents_panel.blit(t1, (100, y + 10))
        incidents_panel.blit(t2, (100, y + 40))
        if 0 <= y < PANEL_HEIGHT:
            incident_card_rects.append((card_rect_abs, i))


    # Top bar
    pygame.draw.rect(incidents_panel, GRAY, (0, 0, PANEL_WIDTH, 77))
    pygame.draw.rect(incidents_panel, DARK_GRAY, (0, 0, PANEL_WIDTH, 20))

    incident_text = font.render(f"Incidents ({len(incidents)})", True, WHITE)
    incidents_panel.blit(incident_text, (20, 30))

    # Bottom feathered fade
    bottom_rect = pygame.Surface((PANEL_WIDTH, 30), pygame.SRCALPHA)
    bottom_rect.fill(DARK_GRAY)
    bottom_feathered = feather_image(bottom_rect, 25, 25, feather_top=True, feather_right=False, feather_bottom=False, feather_left=False)
    incidents_panel.blit(bottom_feathered, (0, PANEL_HEIGHT-30))

    # Scroll buttons
    scroll_up_rel = pygame.Rect(PANEL_WIDTH - 50, 80, 40, 100)
    scroll_down_rel = pygame.Rect(PANEL_WIDTH - 50, 280, 40, 100)
    pygame.draw.rect(incidents_panel, LIGHT_GRAY, scroll_up_rel)   # Up button
    pygame.draw.rect(incidents_panel, LIGHT_GRAY, scroll_down_rel) # Down button

    # Store absolute positions for detection
    incident_scroll_up_rect = pygame.Rect(PANEL_X + scroll_up_rel.x, PANEL_Y + scroll_up_rel.y, scroll_up_rel.width, scroll_up_rel.height)
    incident_scroll_down_rect = pygame.Rect(PANEL_X + scroll_down_rel.x, PANEL_Y + scroll_down_rel.y, scroll_down_rel.width, scroll_down_rel.height)

    # Final blit to screen
    screen.blit(incidents_panel, (PANEL_X, PANEL_Y))


def draw_telemetry_panel():
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
    text = small_font.render(string, True, WHITE)
    telemetry_panel.blit(text, (640 - small_font.size(string)[0] - 55, 50))

    string = f"{robot['state']}"
    text = state_font.render(string, True, WHITE)
    telemetry_panel.blit(text, (640 - state_font.size(string)[0] - 55, 10))

    e_stop_col = DARK_GRAY

    if robot['state'] == "Offline":
        pygame.draw.rect(telemetry_panel, RED, (592, 10, 10, 60))
        string = "last seen"
        text = small_font.render(string, True, WHITE)
        telemetry_panel.blit(text, (640 - small_font.size(string)[0] - 60 - state_font.size("Offline")[0], 30))
    else:
        pygame.draw.rect(telemetry_panel, GREEN, (592, 10, 10, 60))
        e_stop_col = RED

    pygame.draw.rect(telemetry_panel, e_stop_col, (20, 10, 160, 60))


    # --- Robot 2 Telemetry ---
    robot = robots[1]
    string = f"Batt: {robot['battery']}, GPS:{robot['gps']}, {robot['altitude']}"
    text = small_font.render(string, True, WHITE)
    telemetry_panel.blit(text, (695, 50))

    if robot['state'] == "Offline":
        pygame.draw.rect(telemetry_panel, RED, (678, 10, 10, 60))
        text = small_font.render("last seen", True, WHITE)
        telemetry_panel.blit(text, (840, 30))
        e_stop_col = DARK_GRAY
    else:
        pygame.draw.rect(telemetry_panel, GREEN, (678, 10, 10, 60))
        e_stop_col = RED

    telemetry_title = state_font.render(f"{robot['state']}", True, WHITE)
    telemetry_panel.blit(telemetry_title, (695, 10))

    pygame.draw.rect(telemetry_panel, e_stop_col, (1100, 10, 160, 60))

    # --- Blit telemetry panel to screen ---
    screen.blit(telemetry_panel, (screen.get_width() // 2 - telemetry_panel.get_width() // 2, 640))

def draw_map():
    PANEL_WIDTH = 800
    PANEL_HEIGHT = 800

    map_panel = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT))
    map_panel.fill((0,0,0))
    map_panel.blit(map_top_view,(0,0))
    # map_panel.blit(drone_icon, (100,100))

    for i in range(len(robots)):
        state = robots[i]["state"]
        
        state_color = state_colors.get(state, (255, 255, 255))
        print(f'Robot {i+1} + {state} {state_color} {robots[i]["yaw"]}')

        imgX = mapRange(float(robots[i]["gps"].split(', ')[0]), -(world_size[0]/2), (world_size[0]/2), 0, mapImgSize[0])
        imgY = mapRange(float(robots[i]["gps"].split(', ')[1]), -(world_size[1]/2), (world_size[1]/2), 0, mapImgSize[1])
        icon_copy = drone_icon.copy()
        icon_copy = pygame.transform.rotate(icon_copy, robots[i]["yaw"])
        pixel_array = pygame.PixelArray(icon_copy)
        pixel_array.replace((255, 255, 255), state_color)  # Change white pixels to state color)
        del pixel_array
        map_panel.blit(icon_copy,(imgX, imgY));

    screen.blit(map_panel, (500,20))
    

def draw_base_ui():
    screen.fill(DARK_GRAY)

    # # Camera feed placeholder (top-left)
    # if robot_state != "Offline":
    #     pygame.draw.rect(screen, BLACK, (20, 20, 640, 360))
    #     cam_text = small_font.render("Camera Feed", True, WHITE)
    #     screen.blit(cam_text, (30, 30))
    # else:
    #     screen.blit(feathered_splash, (20, 20))

    # screen.blit(feathered_splash, (20, 20))

    # video_loop()

    # draw_telemetry_panel()

    draw_map()
    render_robots_list(robots)
    if selected_robot < 0:
        draw_incidents_panel()
    else:
        drone_ui()

    # pygame.draw.rect(screen, LIGHT_GRAY, (1455, 0, 440, 430))  # Incidents panel top bar
    # draw_left_fade()

    debug_text = small_font.render(f"RS1 {int(clock.get_fps())} {pygame.mouse.get_pos()[0]} {pygame.mouse.get_pos()[1]}", True, WHITE)
    screen.blit(debug_text, (800, 500))

def draw_popup():
    global popup_scale
    scale = popup_scale / 100
    if scale <= 0: return

    popup_width = int(600 * scale)
    popup_height = int(400 * scale)
    popup_x = 340 + (600 - popup_width) // 2
    popup_y = 160 + (400 - popup_height) // 2

    popup_surface = pygame.Surface((popup_width, popup_height))
    popup_surface.fill(WHITE)

    if selected_incident:
        title = font.render(selected_incident["title"], True, BLACK)
        time_text = small_font.render(selected_incident["time"], True, BLACK)
        popup_surface.blit(title, (20, 20))
        popup_surface.blit(time_text, (20, 60))

        # Close button
        pygame.draw.rect(popup_surface, GRAY, (popup_width - 110, popup_height - 50, 100, 40))
        close_text = small_font.render("Close", True, BLACK)
        popup_surface.blit(close_text, (popup_width - 85, popup_height - 40))

        # Image placeholder
        img_w, img_h = int(popup_width * 0.6), int(popup_height * 0.4)
        img_x = (popup_width - img_w) // 2
        img_y = (popup_height - img_h) // 2
        
        pygame.draw.rect(popup_surface, LIGHT_GRAY, (img_x, img_y, img_w, img_h))

    # popup_surface.set_alpha(popup_scale)  # Semi-transparent background
    # print(f"Popup scale: {popup_scale}, Size: {popup_width}x{popup_height}")
    screen.blit(popup_surface, (popup_x, popup_y))

def handle_popup_animation():
    global popup_scale, anim_start_time, animating_in, animating_out, popup_visible
    elapsed = pygame.time.get_ticks() - anim_start_time
    progress = min(1.0, elapsed / ANIM_DURATION)
    if animating_in:
        popup_scale = int(100 * progress)
        if progress >= 1:
            animating_in = False
            popup_visible = True
    elif animating_out:
        popup_scale = int(100 * (1 - progress))
        if progress >= 1:
            animating_out = False
            popup_visible = False
            popup_scale = 0

running = True
while running:
    screen.fill(DARK_GRAY)
    draw_base_ui()
    handle_popup_animation()
    draw_popup()
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos

            if popup_visible:
                # Handle close button
                popup_width = int(600 * popup_scale / 100)
                popup_height = int(400 * popup_scale / 100)
                popup_x = 340 + (600 - popup_width) // 2
                popup_y = 160 + (400 - popup_height) // 2
                rel_x = mx - popup_x
                rel_y = my - popup_y
                if popup_width - 110 < rel_x < popup_width - 10 and popup_height - 50 < rel_y < popup_height - 10:
                    animating_out = True
                    anim_start_time = pygame.time.get_ticks()
            else:
                if selected_robot < 0:
                    # Detect incident click
                    for rect, idx in incident_card_rects:
                        if rect.collidepoint((mx, my)):
                            print(f"Incident clicked: #{idx+1} - {incidents[idx]['title']}")
                            # selected_incident = incidents[idx]
                            break
                    
                    # Detect scroll buttons
                    if incident_scroll_up_rect.collidepoint((mx, my)):
                        scrolling = True
                        scroll_direction = 1

                    elif incident_scroll_down_rect.collidepoint((mx, my)):
                        scrolling = True
                        scroll_direction = -1

                # Detect robot card click
                for rect, idx in robot_card_rects:
                    if rect.collidepoint((mx, my)):
                        print(f"Robot card clicked: #{idx+1}")
                        # If you want to store the selected robot:
                        # selected_robot = robots[idx]
                        break

        if event.type == pygame.MOUSEBUTTONUP:
            scrolling = False

    clock.tick(75)
cap.release()
pygame.quit()
sys.exit()
