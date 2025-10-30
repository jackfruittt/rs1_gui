import pygame
import cv2
import numpy as np
import time
from typing import Optional
from ros_handler import RosHandler

# Camera component that displays ROS2 camera feeds using the centralised RosHandler
class CameraComponent:
    
    def __init__(self, ros_handler: RosHandler, display_rect=(20, 20, 640, 360), instant_switching=True, preload_all=False):
        self.display_rect = display_rect
        self.ros_handler = ros_handler
        self.instant_switching = instant_switching  # If True, use preload method; if False, use subscribe-unsubscribe
        self.preload_all = preload_all  # If True, preload ALL cameras for zero-delay switching
        
        # Available camera topics (fixed at startup)
        self.available_topics = []
        self.current_topic_index = 0
        self.current_topic = None
        self.subscribed_topics = set()  # Topics that are currently subscribed
        
        # Preload settings (only used in instant_switching mode)
        self.preload_count = 5  # Keep 5 cameras preloaded for fast switching (It is beyond what we need but the code is useful for even more advanced systems so i've set it to 5)
        
        # Placeholder image
        self.placeholder_surface = self._create_placeholder()
        self.last_frame = None
        
        # Initialise camera topics
        self._initialise_topics()
        
        # Setup subscriptions based on mode
        if self.instant_switching:
            if self.preload_all:
                self._setup_all_cameras_preload()
            else:
                self._setup_preload_switching()
        else:
            self._setup_single_subscription()
        
        # Set initial display topic
        if self.available_topics:
            self.current_topic = self.available_topics[0]
            self.current_topic_index = 0
    
    # Place holder when there is no camera feed
    def _create_placeholder(self):
        surface = pygame.Surface(self.display_rect[2:4])
        surface.fill((40, 40, 40))
        
        # Add text
        font = pygame.font.Font(None, 36)
        if not self.ros_handler.ros2_available:
            text = "ROS2 Not Available"
            subtext = "GUI in simulation mode"
        else:
            text = "No Camera Feed"
            subtext = "Waiting for topics..."
        
        text_surface = font.render(text, True, (200, 200, 200))
        text_rect = text_surface.get_rect(center=(surface.get_width()//2, surface.get_height()//2 - 20))
        surface.blit(text_surface, text_rect)
        
        # Subtext
        small_font = pygame.font.Font(None, 24)
        subtext_surface = small_font.render(subtext, True, (150, 150, 150))
        subtext_rect = subtext_surface.get_rect(center=(surface.get_width()//2, surface.get_height()//2 + 20))
        surface.blit(subtext_surface, subtext_rect)
        
        return surface
    
    # Initialise a list of camera topics 
    def _initialise_topics(self):
        # Wait for ROS handler to discover topics
        time.sleep(0.5)
        
        self.available_topics = self.ros_handler.get_available_camera_topics()
        
        # Filter for drone-specific camera topics
        drone_topics = [topic for topic in self.available_topics if 'rs1_drone' in topic]
        if drone_topics:
            self.available_topics = drone_topics
        
        print(f"Camera component initialised with {len(self.available_topics)} topics: {self.available_topics}")
    
    # Subscribe to all for instant switching
    def _setup_all_cameras_preload(self):
        for topic in self.available_topics:
            self.ros_handler.subscribe_to_camera_topic(topic)
            self.subscribed_topics.add(topic)
        
        print(f"All-cameras mode: subscribed to all {len(self.available_topics)} camera topics for instant switching")
    
    # Preload 5 (always be subscribed to 5 and unsubscribe the rest)
    def _setup_preload_switching(self):
        # Subscribe to first few cameras for instant switching
        topics_to_preload = self.available_topics[:self.preload_count]
        
        for topic in topics_to_preload:
            self.ros_handler.subscribe_to_camera_topic(topic)
            self.subscribed_topics.add(topic)
        
        print(f"Preload mode: subscribed to {len(topics_to_preload)} camera topics for fast switching")
    
    def _setup_single_subscription(self):
        # Only subscribe to the first camera initially
        if self.available_topics:
            first_topic = self.available_topics[0]
            self.ros_handler.subscribe_to_camera_topic(first_topic)
            self.subscribed_topics.add(first_topic)
            print(f"Single subscription mode: subscribed to {first_topic}")
    
    def switch_to_topic(self, topic_index: int) -> bool:
        # Switch to a different camera topic by index.  
        if not self.available_topics:
            return False
        
        if 0 <= topic_index < len(self.available_topics):
            new_topic = self.available_topics[topic_index]
            
            if self.instant_switching:
                if self.preload_all:
                    # All cameras mode - true instant switching
                    self.current_topic = new_topic
                    self.current_topic_index = topic_index
                    print(f"Instantly switched to: {new_topic}")
                else:
                    # Preload mode - ensure topic is loaded, but avoid unnecessary operations
                    if new_topic not in self.subscribed_topics:
                        self._ensure_topic_preloaded(new_topic)
                    
                    # Switch immediately for no delay
                    self.current_topic = new_topic
                    self.current_topic_index = topic_index
                    print(f"Fast switched to: {new_topic}")
                    
                    # Preload adjacent topics for next switch
                    self._preload_adjacent_topics()
            else:
                # Subscribe-unsubscribe mode - best for hardware limitations 
                self._switch_subscription(new_topic)
                self.current_topic = new_topic
                self.current_topic_index = topic_index
                print(f"Switched to: {new_topic}")
            
            return True
        
        return False
    
    def _ensure_topic_preloaded(self, topic_name: str):
        if topic_name not in self.subscribed_topics:
            # Only unsubscribe if at the limit and need space
            if len(self.subscribed_topics) >= self.preload_count:
                # Find the topic that's furthest from current position to unsubscribe
                current_idx = self.current_topic_index
                furthest_topic = None
                max_distance = 0
                
                for loaded_topic in list(self.subscribed_topics):
                    if loaded_topic != self.current_topic and loaded_topic != topic_name:
                        try:
                            loaded_idx = self.available_topics.index(loaded_topic)
                            distance = min(abs(loaded_idx - current_idx), 
                                         len(self.available_topics) - abs(loaded_idx - current_idx))
                            if distance > max_distance:
                                max_distance = distance
                                furthest_topic = loaded_topic
                        except ValueError:
                            # Topic not in list, safe to remove
                            furthest_topic = loaded_topic
                            break
                
                if furthest_topic:
                    self.ros_handler.unsubscribe_from_camera_topic(furthest_topic)
                    self.subscribed_topics.remove(furthest_topic)
                    print(f"Unloaded furthest topic: {furthest_topic}")
            
            # Subscribe to the new topic
            self.ros_handler.subscribe_to_camera_topic(topic_name)
            self.subscribed_topics.add(topic_name)
            print(f"Preloaded: {topic_name}")
        
        # Pre-preload adjacent topics for smoother switching
        self._preload_adjacent_topics()
    
    def _preload_adjacent_topics(self):
        if len(self.subscribed_topics) < self.preload_count:
            # Try to preload next and previous topics
            current_idx = self.current_topic_index
            
            # Preload next topic
            next_idx = (current_idx + 1) % len(self.available_topics)
            next_topic = self.available_topics[next_idx]
            if next_topic not in self.subscribed_topics and len(self.subscribed_topics) < self.preload_count:
                self.ros_handler.subscribe_to_camera_topic(next_topic)
                self.subscribed_topics.add(next_topic)
                print(f"Pre-preloaded next: {next_topic}")
            
            # Preload previous topic if we still have room
            if len(self.subscribed_topics) < self.preload_count:
                prev_idx = (current_idx - 1) % len(self.available_topics)
                prev_topic = self.available_topics[prev_idx]
                if prev_topic not in self.subscribed_topics:
                    self.ros_handler.subscribe_to_camera_topic(prev_topic)
                    self.subscribed_topics.add(prev_topic)
                    print(f"Pre-preloaded prev: {prev_topic}")
    
    def _switch_subscription(self, new_topic: str):
        # Unsubscribe from current topic
        if self.current_topic and self.current_topic in self.subscribed_topics:
            self.ros_handler.unsubscribe_from_camera_topic(self.current_topic)
            self.subscribed_topics.remove(self.current_topic)
        
        # Subscribe to new topic
        self.ros_handler.subscribe_to_camera_topic(new_topic)
        self.subscribed_topics.add(new_topic)
    
    def switch_to_next_topic(self) -> bool:
        if not self.available_topics:
            return False
        
        next_index = (self.current_topic_index + 1) % len(self.available_topics)
        return self.switch_to_topic(next_index)
     
    def switch_to_previous_topic(self) -> bool:
        if not self.available_topics:
            return False
        previous_index = (self.current_topic_index - 1) % len(self.available_topics)
        return self.switch_to_topic(previous_index)
    
    def switch_to_drone_camera(self, drone_id: int, camera_type: str = "front") -> bool:
        """
        Switch to a specific drone's camera.
        Args:
            drone_id: Drone number (1, 2, 3, etc.)
            camera_type: "front" or "bottom"
        """
        topic_name = f"/rs1_drone_{drone_id}/{camera_type}/image"
        
        if topic_name in self.available_topics:
            topic_index = self.available_topics.index(topic_name)
            return self.switch_to_topic(topic_index)
        else:
            print(f"Topic {topic_name} not available in {self.available_topics}")
            return False


    def cycle_drone_camera(self, current_drone_index: int, num_drones: int, direction: int):
        """
        Cycle through drone front cameras without returning/updating drone selection.
        Args:
            current_drone_index: Current drone index (0-based, -1 if none selected)
            num_drones: Total number of drones
            direction: 1 for next, -1 for previous
        """
        drone_id = current_drone_index
        
        if drone_id == -1:
            drone_id = 0
        else:
            drone_id = drone_id + direction
            
            # Wrap around
            if drone_id < 0:
                drone_id = num_drones - 1
            elif drone_id >= num_drones:
                drone_id = 0
        
        # Switch to front camera (drone_id is 1-based for topic names, so add 1)
        self.switch_to_drone_camera(drone_id + 1, "front")


    def get_current_topic(self) -> Optional[str]:
        return self.current_topic
    
    def _cv2_to_pygame_surface(self, cv_image):
        # Convert BGR to RGB (OpenCV uses BGR)
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Resize to fit display area
        display_width, display_height = self.display_rect[2:4]
        rgb_image = cv2.resize(rgb_image, (display_width, display_height))
        
        # Convert to Pygame surface
        rgb_image = np.transpose(rgb_image, (1, 0, 2))
        surface = pygame.surfarray.make_surface(rgb_image)
        
        return surface
    
    def update(self):
        # Update camera feed from RosHandler
        if not self.current_topic:
            return
        
        # Get latest image from RosHandler
        cv_image = self.ros_handler.get_latest_camera_image(self.current_topic)
        
        if cv_image is not None:
            self.last_frame = self._cv2_to_pygame_surface(cv_image)
    
    def draw(self, screen):
        # Draw the camera feed on screen
        x, y, w, h = self.display_rect
        
        if self.last_frame and self.ros_handler.is_topic_active(self.current_topic):
            screen.blit(self.last_frame, (x, y))
            
            # Draw topic info
            if self.current_topic:
                font = pygame.font.Font(None, 24)
                topic_text = font.render(self.current_topic, True, (255, 255, 255))
                # Add background for better readability
                text_rect = topic_text.get_rect()
                text_rect.x = x + 10
                text_rect.y = y + 10
                pygame.draw.rect(screen, (0, 0, 0), text_rect.inflate(10, 5))
                screen.blit(topic_text, (x + 15, y + 12))
        else:
            screen.blit(self.placeholder_surface, (x, y))
        
        # Draw border
        pygame.draw.rect(screen, (100, 100, 100), (x, y, w, h), 2)
    # Handle keyboard input for camera switching
    def handle_keypress(self, key):
        if key == pygame.K_c:  # Press 'C' to switch camera
            self.switch_to_next_topic()
    
    # Get status information for display
    def get_status_info(self) -> str:
        if not self.ros_handler.ros2_available:
            return "ROS2 Not Available"
        
        if not self.available_topics:
            return "No camera topics found"
        
        if self.current_topic:
            is_active = self.ros_handler.is_topic_active(self.current_topic)
            status = "LIVE" if is_active else "STALE"
            return f"Camera: {self.current_topic} ({self.current_topic_index + 1}/{len(self.available_topics)}) [{status}]"
        
        return "Searching for cameras..."
    
    # Clean up camera component
    def cleanup(self):
        # Unsubscribe from all subscribed topics
        for topic in list(self.subscribed_topics):
            self.ros_handler.unsubscribe_from_camera_topic(topic)
        self.subscribed_topics.clear()
        print("Camera component cleanup complete.")