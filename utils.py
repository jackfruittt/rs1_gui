import pygame
import numpy as np
from constants import pretty

import re
from typing import Any, Dict, Iterable, List, Tuple
import yaml

import subprocess, os
from subprocess import DEVNULL


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

def spawn_cmd_safe(cmd): # genAI function
    print(f"SENDING CMD: {cmd}")
    try:
        shell = isinstance(cmd, str)
        kwargs = dict(stdout=DEVNULL, stderr=DEVNULL, stdin=DEVNULL, shell=shell)
        # Detach so it won’t block/kill your process if it errors.
        if os.name == "posix":
            kwargs["start_new_session"] = True
        else:  # Windows
            kwargs["creationflags"] = (
                subprocess.CREATE_NEW_PROCESS_GROUP |
                subprocess.DETACHED_PROCESS |
                subprocess.CREATE_NO_WINDOW
            )
        subprocess.Popen(cmd, **kwargs)
    except (FileNotFoundError, OSError):
        pass

DroneArray = List[List[List[float]]]
_DRONE_KEY_RE = re.compile(r"^rs1_drone_(\d+)$")

def _parse_xyz(node: Any) -> List[float]:
    # [x, y, z]
    if isinstance(node, (list, tuple)) and len(node) >= 3:
        return [float(node[0]), float(node[1]), float(node[2])]

    # {x:..., y:..., z:...}  or  {position: [...]}
    if isinstance(node, dict):
        if all(k in node for k in ("x", "y", "z")):
            return [float(node["x"]), float(node["y"]), float(node["z"])]
        if "position" in node:
            return _parse_xyz(node["position"])

    # "x, y, z"
    if isinstance(node, str):
        parts = [p.strip() for p in node.split(",")]
        if len(parts) >= 3:
            return [float(parts[0]), float(parts[1]), float(parts[2])]

    raise ValueError(f"Unrecognized position format: {node!r}")

def _extract_positions(drone_node: Any) -> List[List[float]]:
    """
    Accept either:
      - a list of positions directly
      - a mapping with 'positions' or 'waypoints' containing the list
    """
    if isinstance(drone_node, list):
        positions = drone_node
    elif isinstance(drone_node, dict):
        if "positions" in drone_node:
            positions = drone_node["positions"]
        elif "waypoints" in drone_node:
            positions = drone_node["waypoints"]
        else:
            # fallback: try values as positions if they look like position-like nodes
            values = list(drone_node.values())
            if values and all(isinstance(v, (list, tuple, dict, str)) for v in values):
                positions = values
            else:
                raise ValueError("Drone entry has no 'positions'/'waypoints' and is not a list.")
    else:
        raise ValueError("Drone entry must be a list or mapping.")

    return [_parse_xyz(p) for p in positions]

def _iter_mappings(root: Any) -> Iterable[Dict[str, Any]]:
    """Yield all dict nodes in the YAML tree (including root) for scanning."""
    stack = [root]
    while stack:
        node = stack.pop()
        if isinstance(node, dict):
            yield node
            stack.extend(node.values())
        elif isinstance(node, list):
            stack.extend(node)

def _find_drone_block(root: Any) -> Dict[str, Any]:
    """
    Find and return the FIRST mapping that contains at least one key of the form rs1_drone_<N>.
    If none found, raise.
    """
    for mapping in _iter_mappings(root):
        keys = list(mapping.keys()) if isinstance(mapping, dict) else []
        if any(_DRONE_KEY_RE.match(str(k)) for k in keys):
            return mapping
    raise ValueError("No mapping with keys matching 'rs1_drone_<N>' was found in the YAML.")

def load_waypoints_yaml(filepath: str) -> DroneArray:
    """
    Load a YAML file and return a 3D array:
      result[drone_index][position_index] = [x, y, z]
    """
    with open(filepath, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f)

    # Find the mapping containing rs1_drone_<N> keys (supports wrapped structures)
    block = _find_drone_block(root)

    # Collect (drone_num, positions)
    entries: List[Tuple[int, List[List[float]]]] = []
    for key, val in block.items():
        m = _DRONE_KEY_RE.match(str(key))
        if not m:
            continue
        drone_num = int(m.group(1))
        positions = _extract_positions(val)
        entries.append((drone_num, positions))

    if not entries:
        return []

    entries.sort(key=lambda kv: kv[0])
    min_id, max_id = entries[0][0], entries[-1][0]

    # Re-base so smallest ID => index 0 (contiguous outer dimension)
    result: DroneArray = [[] for _ in range(max_id - min_id + 1)]
    for drone_num, positions in entries:
        result[drone_num - min_id] = positions
    return result