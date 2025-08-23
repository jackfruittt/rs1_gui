import pygame
import cv2
import numpy as np
import threading
import queue
import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available. Camera component will show placeholder.")

# ROS2 node for receiving camera images
class CameraNode(Node):
    
    def __init__(self):
        super().__init__('rs1_camera_viewer') # ROS2 node name
        self.bridge = CvBridge() # Converts ROS images to OpenCV format
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.subscription = None # Holds the image subscription 
        self.current_topic = None
        
    def subscribe_to_topic(self, topic_name: str):
        # Subscribe to a camera topic
        if self.subscription:
            self.destroy_subscription(self.subscription)
        
        self.current_topic = topic_name
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {topic_name}')
    
    def image_callback(self, msg):
        # Callback for received images
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_lock:
                self.latest_image = cv_image.copy()
                
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
    
    def get_latest_image(self):
        # Get the latest image (thread-safe) 
        with self.image_lock:
            if self.latest_image is not None:
                return self.latest_image.copy()
        return None


# Camera component that can display ROS2 camera feeds
# Replaces the VideoPlayer component
class CameraComponent:
    
    def __init__(self, display_rect=(20, 20, 640, 360)):
        self.display_rect = display_rect
        self.camera_node = None
        self.ros_thread = None
        self.running = False
        self.ros2_available = ROS2_AVAILABLE
        
        # Available camera topics (will be auto-discovered)
        self.available_topics = []
        self.current_topic_index = 0
        self.topic_switch_time = 0
        
        # Placeholder image
        self.placeholder_surface = self._create_placeholder()
        self.last_frame = None
        
        if self.ros2_available:
            self._start_ros2_node()
            self._discover_topics()
    
    # Create a placeholder image when no camera feed is available
    def _create_placeholder(self):
        surface = pygame.Surface(self.display_rect[2:4])
        surface.fill((40, 40, 40))
        
        # Add text
        font = pygame.font.Font(None, 36)
        if not self.ros2_available:
            text = "ROS2 Not Available"
            subtext = "Install: pip install rclpy cv_bridge"
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
    
    # Start ROS2 node in separate thread so the rest of the pygame GUI doesn't freeze
    def _start_ros2_node(self):
        if not self.ros2_available:
            return
        
        self.running = True
        self.ros_thread = threading.Thread(target=self._ros_thread_worker, daemon=True)
        self.ros_thread.start()
        
        # Give ROS2 time to initialise
        time.sleep(0.5)
    
    # Worker thread for ROS2
    def _ros_thread_worker(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.camera_node = CameraNode()
            
            while self.running and rclpy.ok():
                rclpy.spin_once(self.camera_node, timeout_sec=0.1)
                
        except Exception as e:
            print(f"ROS2 camera thread error: {e}")
        finally:
            if self.camera_node:
                self.camera_node.destroy_node()
    
    # Discover available camera topics
    def _discover_topics(self):
        
        if not self.ros2_available or not self.camera_node:
            return
        
        # Wait a bit for node to initialise
        time.sleep(1.0)
        
        try:
            topic_list = self.camera_node.get_topic_names_and_types()
            camera_topics = []
            
            for topic_name, topic_types in topic_list:
                if 'sensor_msgs/msg/Image' in topic_types:
                    # Prioritise RS1 drone topics
                    if 'rs1_drone' in topic_name:
                        camera_topics.append(topic_name)
            
            # Common camera topics
            common_topics = ['/camera/image_raw', '/image_raw', '/usb_cam/image_raw']
            for topic in common_topics:
                for topic_name, topic_types in topic_list:
                    if topic_name == topic and 'sensor_msgs/msg/Image' in topic_types:
                        if topic not in camera_topics:
                            camera_topics.append(topic)
            
            self.available_topics = camera_topics
            print(f"Discovered camera topics: {camera_topics}")
            
            # Subscribe to first available topic
            if camera_topics:
                self.switch_to_topic(0)
                
        except Exception as e:
            print(f"Error discovering topics: {e}")
    
    # Switch to a different camera topic by index
    def switch_to_topic(self, topic_index: int):
        if not self.available_topics or not self.camera_node:
            return False
        
        if 0 <= topic_index < len(self.available_topics):
            topic_name = self.available_topics[topic_index]
            self.camera_node.subscribe_to_topic(topic_name)
            self.current_topic_index = topic_index
            print(f"Switched to camera topic: {topic_name}")
            return True
        
        return False
    # Switch to the next available camera topic
    def switch_to_next_topic(self):
        if not self.available_topics:
            return False
        
        next_index = (self.current_topic_index + 1) % len(self.available_topics)
        return self.switch_to_topic(next_index)
    
    def switch_to_drone_camera(self, drone_id: int, camera_type: str = "front"):
        """
        Switch to a specific drone's camera
        Args:
            drone_id: Drone number (1, 2, 3, etc.)
            camera_type: "front" or "bottom"
        """
        topic_name = f"/rs1_drone_{drone_id}/{camera_type}/image"
        
        if topic_name in self.available_topics:
            topic_index = self.available_topics.index(topic_name)
            return self.switch_to_topic(topic_index)
        else:
            print(f"Topic {topic_name} not available")
            return False
    
    # Get the currently subscribed topic name
    def get_current_topic(self):
        if self.available_topics and 0 <= self.current_topic_index < len(self.available_topics):
            return self.available_topics[self.current_topic_index]
        return None
    
    # Convert openCV image to Pygame surface for display
    def _cv2_to_pygame_surface(self, cv_image):
        
        # Convert BGR to RGB (OpenCV uses BGR8)
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Resize to fit display area
        display_width, display_height = self.display_rect[2:4]
        rgb_image = cv2.resize(rgb_image, (display_width, display_height))
        
        # Convert to Pygame surface
        rgb_image = np.transpose(rgb_image, (1, 0, 2))
        surface = pygame.surfarray.make_surface(rgb_image)
        
        return surface
    
    # Update camera feed 
    def update(self):
        if not self.ros2_available or not self.camera_node:
            return
        
        # Get latest image
        cv_image = self.camera_node.get_latest_image()
        
        if cv_image is not None:
            self.last_frame = self._cv2_to_pygame_surface(cv_image)
    
    # Draw the camera feed on screen
    def draw(self, screen):
        x, y, w, h = self.display_rect
        
        if self.last_frame:
            screen.blit(self.last_frame, (x, y))
            
            # Draw topic info
            if self.get_current_topic():
                font = pygame.font.Font(None, 24)
                topic_text = font.render(self.get_current_topic(), True, (0, 0, 0))
                screen.blit(topic_text, (x + 70, y + 10))
        else:
            screen.blit(self.placeholder_surface, (x, y))
        
        # Draw border
        pygame.draw.rect(screen, (100, 100, 100), (x, y, w, h), 2)

    # Handle keyboard input for camera switching    
    def handle_keypress(self, key):

        if key == pygame.K_c:  # Press 'C' to switch camera
            self.switch_to_next_topic()
    
    def get_status_info(self):
        if not self.ros2_available:
            return "ROS2 Not Available"
        
        if not self.available_topics:
            return "No camera topics found"
        
        current_topic = self.get_current_topic()
        if current_topic:
            return f"Camera: {current_topic} ({self.current_topic_index + 1}/{len(self.available_topics)})"
        
        return "Searching for cameras..."
    
    def cleanup(self):
        self.running = False
        
        if self.ros_thread:
            self.ros_thread.join(timeout=2)
        
        if self.camera_node:
            self.camera_node.destroy_node()
            
        print("Camera component cleanup complete.")