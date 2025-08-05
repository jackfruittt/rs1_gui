import pygame
import numpy as np
import sys

# Hard-coded telemetry data
battery = "92%"
gps = "-33.8688, 151.2093"
altitude = "102m"
robot_state = "Offline"

# --- Function to create the feathered image ---
def feather_image(surface, feather_size_x, feather_size_y):
    """
    Feathers the right and bottom edges of a Pygame surface.

    Args:
        surface (pygame.Surface): The image surface to feather.
        feather_size_x (int): The width of the feathering effect on the right edge.
        feather_size_y (int): The height of the feathering effect on the bottom edge.

    Returns:
        pygame.Surface: A new surface with the feathered effect.
    """
    # Create a copy of the surface to work on, with per-pixel alpha
    feathered_surface = surface.copy().convert_alpha()
    width, height = feathered_surface.get_size()

    # Create an alpha mask surface of the same size, initially fully opaque
    alpha_mask = pygame.Surface((width, height), pygame.SRCALPHA)
    alpha_mask.fill((255, 255, 255, 255))

    # Get a direct NumPy view of the alpha pixels. Modifying this array
    # will directly modify the alpha_mask surface, which is very efficient.
    alpha_pixels_view = pygame.surfarray.pixels_alpha(alpha_mask)

    # Generate horizontal gradient (for the right edge)
    h_gradient = np.linspace(255, 0, feather_size_x)
    
    # Generate vertical gradient (for the bottom edge)
    v_gradient = np.linspace(255, 0, feather_size_y)

    # Apply the gradients to the mask array view
    # The minimum of the two gradients is taken to blend the corner correctly
    for x in range(width):
        for y in range(height):
            # Start with the current alpha value (which is 255)
            alpha = alpha_pixels_view[x, y]
            
            # Check if pixel is within the horizontal feather zone
            if x >= width - feather_size_x:
                # Get the corresponding value from the horizontal gradient
                grad_index_h = x - (width - feather_size_x)
                alpha = min(alpha, h_gradient[grad_index_h])
            
            # Check if pixel is within the vertical feather zone
            if y >= height - feather_size_y:
                # Get the corresponding value from the vertical gradient
                grad_index_v = y - (height - feather_size_y)
                alpha = min(alpha, v_gradient[grad_index_v])
            
            # Apply the final calculated alpha to the pixel
            alpha_pixels_view[x, y] = int(alpha)

    # IMPORTANT: The surface is "locked" while the surfarray view exists.
    # We must delete the view to unlock it for further blitting.
    del alpha_pixels_view
    
    # Apply the alpha mask to the image copy.
    # BLEND_RGBA_MULT multiplies the alpha of the mask with the alpha of the image.
    feathered_surface.blit(alpha_mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    
    return feathered_surface


pygame.init()
screen = pygame.display.set_mode((1280, 720))
pygame.display.set_caption("Drone GUI")
clock = pygame.time.Clock()

# Fonts and colors
font = pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 32)
small_font = pygame.font.Font("media/fonts/TurretRoad-Bold.otf", 20)
large_font = pygame.font.Font("media/fonts/TurretRoad-ExtraBold.otf", 48)
state_font = pygame.font.Font("media/fonts/boston.ttf", 38)
WHITE = (255, 255, 255)
LIGHT_GRAY = (200, 200, 200)
GRAY = (150, 150, 150)
DARK_GRAY = (30, 30, 30)
BLACK = (0, 0, 0)
BLUE = (100, 100, 255)
RED = (255, 100, 100)
GREEN = (100, 255, 100)

forest_splash = pygame.image.load("media/images/forest_splash.jpg").convert()
splash_rect = forest_splash.get_rect(center=(640, 360))

feathered_splash = feather_image(forest_splash, 150, 150)

# Dummy incidents
incidents = [
    {"id": 1, "title": "Fire Detected", "time": "2025-08-01 13:20"},
    {"id": 2, "title": "Fungal Detected", "time": "2025-08-01 13:45"},
    {"id": 3, "title": "Bear Detected", "time": "2025-08-01 14:05"},
]

# Popup state
popup_visible = False
selected_incident = None
popup_scale = 0
animating_in = False
animating_out = False
anim_start_time = 0
ANIM_DURATION = 200  # milliseconds

def draw_base_ui():
    screen.fill(DARK_GRAY)

    # Camera feed placeholder (top-left)
    if robot_state != "Offline":
        pygame.draw.rect(screen, BLACK, (20, 20, 640, 360))
        cam_text = small_font.render("Camera Feed", True, WHITE)
        screen.blit(cam_text, (30, 30))
    else:
        screen.blit(feathered_splash, (20, 20))

    # Incident list (right)
    pygame.draw.rect(screen, GRAY, (680, 20, 580, 500))
    incident_text = font.render("Incident List", True, BLACK)
    screen.blit(incident_text, (700, 30))

    for i, inc in enumerate(incidents):
        y = 80 + i * 100
        pygame.draw.rect(screen, LIGHT_GRAY, (700, y, 540, 80))
        pygame.draw.rect(screen, DARK_GRAY, (710, y + 10, 60, 60))  # Image placeholder
        t1 = small_font.render(inc["title"], True, BLACK)
        t2 = small_font.render(inc["time"], True, BLACK)
        screen.blit(t1, (780, y + 10))
        screen.blit(t2, (780, y + 40))

    # Telemetry info (bottom)
    # pygame.draw.rect(screen, GRAY, (20, 400, 640, 300))
    lines = [
        f"Battery: {battery}",
        f"GPS: {gps}",
        f"Altitude: {altitude}"
    ]
    for i, line in enumerate(lines):
        text = small_font.render(line, True, WHITE)
        screen.blit(text, (20, 620 + i * 20))

    if robot_state == "Offline":
        pygame.draw.circle(screen, RED, (28, 598), 10)
    else:
        pygame.draw.circle(screen, GREEN, (28, 598), 10)
        
    telemetry_title = state_font.render(f"{robot_state}", True, WHITE)
    screen.blit(telemetry_title, (45, 580))

    debug_text = small_font.render(f"RS1 debug {pygame.mouse.get_pos()[0]} {pygame.mouse.get_pos()[1]}", True, WHITE)
    screen.blit(debug_text, (1000, 690))

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
                    y = 80 + i * 100
                    if 700 < mx < 1240 and y < my < y + 80:
                        selected_incident = inc
                        animating_in = True
                        anim_start_time = pygame.time.get_ticks()
                        break

    clock.tick(75)

pygame.quit()
sys.exit()
