import pygame
import cv2
import numpy as np
import sys

# Hard-coded telemetry data
robots = [
    { # robot 1 - quadcopter
        "battery": "92%",
        "gps": "-33.8688, 151.2093",
        "altitude": "102m",
        "state": "Offline"
    },
    { # robot 2 - rover
        "battery": "64%",
        "gps": "-37.8136, 144.9631",
        "altitude": "87m",
        "state": "Online"
    }
]


pretty = True  # Set to True for feathered edges, False for solid edges

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

scrolling = False
scoll_speed = 5
scroll_direction = 1

pygame.init()
screen = pygame.display.set_mode((1280, 720))
pygame.display.set_caption("RS1 GUI")
clock = pygame.time.Clock()

# Fonts and colors
font = pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 32)
small_font = pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 20)
large_font = pygame.font.Font("media/fonts/TurretRoad-ExtraBold.otf", 48)
state_font = pygame.font.Font("media/fonts/boston.ttf", 38)
WHITE = (255, 255, 255)
LIGHT_GRAY = (200, 200, 200)
GRAY = (50, 50, 50)
DARK_GRAY = (30, 30, 30)
BLACK = (0, 0, 0)
BLUE = (100, 100, 255)
RED = (255, 100, 100)
YELLOW = (255, 255, 100)
GREEN = (100, 255, 100)

severity_colors = (GREEN, YELLOW, RED)

forest_splash = pygame.image.load("media/images/forest_splash.jpg").convert()
splash_rect = forest_splash.get_rect(center=(640, 360))

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
        screen.blit(last_good_frame_surface, (20, 20))

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

    video_loop()

    # Incident list (right)
    panelwidth = 580
    pygame.draw.rect(screen, GRAY, (680, 20, panelwidth, 490))

    if scrolling:
        global incident_y
        incident_y += scroll_direction * scoll_speed

    for i, inc in enumerate(incidents):
        y = incident_y + i * 90
        pygame.draw.rect(screen, LIGHT_GRAY, (700, y, 500, 80))
        pygame.draw.rect(screen, DARK_GRAY, (710, y + 10, 60, 60))  # Image placeholder
        pygame.draw.rect(screen, severity_colors[inc["severity"] - 1], (700, y + 75, 500, 5))
        t1 = small_font.render(f'#{i+1} {inc["title"]}', True, BLACK)
        t2 = small_font.render(inc["time"], True, BLACK)
        screen.blit(t1, (780, y + 10))
        screen.blit(t2, (780, y + 40))
    
    pygame.draw.rect(screen, RED, (680, 500, panelwidth, 250))
    bottom_rect = pygame.Surface((panelwidth, 30), pygame.SRCALPHA)
    bottom_rect.fill(DARK_GRAY)
    bottom_feathered = feather_image(bottom_rect, 25, 25, feather_top=True, feather_right=False, feather_bottom=False, feather_left=False)
    screen.blit(bottom_feathered, (680, 470))
    pygame.draw.rect(screen, DARK_GRAY, (680, 500, panelwidth, 250))


    pygame.draw.rect(screen, GRAY, (680, 0, panelwidth, 75))
    pygame.draw.rect(screen, DARK_GRAY, (680, 0, panelwidth, 20))
    # top_rect = pygame.Surface((panelwidth, 80), pygame.SRCALPHA)
    # top_rect.fill(GRAY)
    # top_feathered = feather_image(top_rect, 25, 25, feather_top=False, feather_right=False, feather_bottom=True, feather_left=False)
    # screen.blit(top_feathered, (680, 0))
    incident_text = font.render(f"Incidents ({len(incidents)})", True, WHITE)
    screen.blit(incident_text, (700, 30))

    # incident scroll buttons
    pygame.draw.rect(screen, LIGHT_GRAY, (1210, 80, 40, 100))
    pygame.draw.rect(screen, LIGHT_GRAY, (1210, 390, 40, 100))

    #telemery surface
    telemetry_surface = pygame.Surface((900, 80))
    telemetry_surface.fill(GRAY)
    screen.blit(telemetry_surface, (screen.get_width()//2 - telemetry_surface.get_width()//2, 640))
    pygame.draw.rect(screen, RED, (610, 650, 60, 60))

# --- Robot 1 Telemetry ---
    robot = robots[0]
    string = f"Batt: {robot['battery']}, GPS:{robot['gps']}, {robot['altitude']}"
    text = small_font.render(string, True, WHITE)
    screen.blit(text, (640 - small_font.size(string)[0] - 55, 690))

    string = f"{robot['state']}"
    text = state_font.render(string, True, WHITE)
    screen.blit(text, (640 - state_font.size(string)[0] - 55, 650))

    if robot['state'] == "Offline":
        pygame.draw.rect(screen, RED, (592, 650, 10, 60))
        string = "last seen"
        text = small_font.render(string, True, WHITE)
        screen.blit(text, (640 - small_font.size(string)[0] - 60 - state_font.size("Offline")[0], 670))
    else:
        pygame.draw.rect(screen, GREEN, (592, 650, 10, 60))


    # --- Robot 2 Telemetry ---
    robot = robots[1]
    string = f"Batt: {robot['battery']}, GPS:{robot['gps']}, {robot['altitude']}"
    text = small_font.render(string, True, WHITE)
    screen.blit(text, (695, 690))

    if robot['state'] == "Offline":
        pygame.draw.rect(screen, RED, (678, 650, 10, 60))
        text = small_font.render("last seen", True, WHITE)
        screen.blit(text, (840, 670))
    else:
        pygame.draw.rect(screen, GREEN, (678, 650, 10, 60))

    telemetry_title = state_font.render(f"{robot['state']}", True, WHITE)
    screen.blit(telemetry_title, (695, 650))

    debug_text = small_font.render(f"RS1 debug {pygame.mouse.get_pos()[0]} {pygame.mouse.get_pos()[1]}", True, WHITE)
    screen.blit(debug_text, (1000, 790))

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
                # Detect incident click
                for i, inc in enumerate(incidents):
                    y = incident_y + i * 90
                    if 700 < mx < 1200 and y < my < y + 80 and 80 < my and my < 470:
                        selected_incident = inc
                        animating_in = True
                        anim_start_time = pygame.time.get_ticks()
                        break
                
                # Detect scroll buttons
                if 1210 < mx < 1250 and 80 < my < 180:
                    scrolling = True
                    scroll_direction = 1
                    
                elif 1210 < mx < 1250 and 390 < my < 490:
                    scrolling = True
                    scroll_direction = -1
                    
                    

        if event.type == pygame.MOUSEBUTTONUP:
            scrolling = False

    clock.tick(75)
cap.release()
pygame.quit()
sys.exit()
