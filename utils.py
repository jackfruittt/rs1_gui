import pygame
import numpy as np
from constants import pretty


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


def mapRange(value, inMin, inMax, outMin, outMax):
    """Map a value from one range to another"""
    return outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))


def draw_left_fade(screen):
    """Draw left fade overlay"""
    l_fade_surface = pygame.Surface((670, 395), pygame.SRCALPHA)
    l_fade_surface.fill((225, 255, 255, 108))  # Transparent background
    screen.blit(l_fade_surface, (0, 0))