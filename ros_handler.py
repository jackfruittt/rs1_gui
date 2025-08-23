import threading
import time
import queue
from typing import Optional, Dict, Callable, Any

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available. ROS Handler will run in simulation mode.")


class RosHandler:
    """
    Centralized ROS2 handler that manages all ROS topics in a single thread.
    This allows the pygame GUI to remain responsive while handling ROS communications.
    """
    
    def __init__(self, node_name: str = 'rs1_gui_handler'):
        self.node_name = node_name
        self.node = None
        self.ros_thread = None
        self.running = False
        self.ros2_available = ROS2_AVAILABLE
        
        # Thread-safe data storage
        self.data_lock = threading.Lock()
        self.camera_data = {}  # Store latest camera images by topic
        self.telemetry_data = {}  # Store telemetry data
        self.topic_callbacks = {}  # Custom callbacks for topics
        
        # Camera-specific components
        self.bridge = CvBridge() if ROS2_AVAILABLE else None
        self.available_camera_topics = []
        
        # Remove periodic discovery - topics are fixed at startup
        self.topics_discovered = False
        
        if self.ros2_available:
            self._start_ros_node()
    
    def set_expected_drone_topics(self, num_drones: int, camera_types: list = None):
        """
        Set expected drone topics without discovery.
        Args:
            num_drones: Number of drones (1, 2, 3, etc.)
            camera_types: List of camera types per drone (default: ['front', 'bottom'])
        """
        if camera_types is None:
            camera_types = ['front', 'bottom']
        
        expected_topics = []
        for drone_id in range(1, num_drones + 1):
            for camera_type in camera_types:
                topic_name = f"/rs1_drone_{drone_id}/{camera_type}/image"
                expected_topics.append(topic_name)
        
        with self.data_lock:
            self.available_camera_topics = expected_topics
        
        self.topics_discovered = True  # Skip automatic discovery
        print(f"Set expected topics for {num_drones} drones: {expected_topics}")
    
    def _start_ros_node(self):
        """Start the ROS2 node in a separate thread."""
        if not self.ros2_available:
            return False
        
        self.running = True
        self.ros_thread = threading.Thread(target=self._ros_thread_worker, daemon=True)
        self.ros_thread.start()
        
        # Give ROS2 time to initialise
        time.sleep(0.5)
        return True
    
    def _ros_thread_worker(self):
        """Main worker thread for ROS2 operations."""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = RosNode(self.node_name, self)
            
            print(f"ROS Handler started with node: {self.node_name}")
            
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                
                # One-time topic discovery at startup
                if not self.topics_discovered:
                    self._discover_topics()
                    self.topics_discovered = True
                
        except Exception as e:
            print(f"ROS Handler thread error: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
            print("ROS Handler thread terminated")
    
    def _discover_topics(self):
        """Discover available ROS topics."""
        if not self.node:
            return
        
        try:
            topic_list = self.node.get_topic_names_and_types()
            camera_topics = []
            
            for topic_name, topic_types in topic_list:
                if 'sensor_msgs/msg/Image' in topic_types:
                    camera_topics.append(topic_name)
            
            # Update available camera topics
            with self.data_lock:
                self.available_camera_topics = camera_topics
            
            print(f"Discovered {len(camera_topics)} camera topics: {camera_topics}")
            
        except Exception as e:
            print(f"Error discovering topics: {e}")
    
    def subscribe_to_camera_topic(self, topic_name: str) -> bool:
        if not self.node or not self.ros2_available:
            return False
        
        try:
            self.node.subscribe_to_camera(topic_name)
            print(f"Subscribed to camera topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to subscribe to {topic_name}: {e}")
            return False
    
    def unsubscribe_from_camera_topic(self, topic_name: str) -> bool:
        if not self.node:
            return False
        
        try:
            self.node.unsubscribe_from_camera(topic_name)
            with self.data_lock:
                if topic_name in self.camera_data:
                    del self.camera_data[topic_name]
            print(f"Unsubscribed from camera topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to unsubscribe from {topic_name}: {e}")
            return False
    
    def get_latest_camera_image(self, topic_name: str):
        with self.data_lock:
            camera_info = self.camera_data.get(topic_name, {})
            image = camera_info.get('image', None)
            # Only copy when actually needed by the GUI
            return image.copy() if image is not None else None
    
    def get_camera_info(self, topic_name: str) -> Dict[str, Any]:
        with self.data_lock:
            return self.camera_data.get(topic_name, {})
    
    def get_available_camera_topics(self) -> list:
        with self.data_lock:
            return self.available_camera_topics.copy()
    
    def register_topic_callback(self, topic_name: str, callback: Callable):
        self.topic_callbacks[topic_name] = callback
    
    def _handle_camera_message(self, topic_name: str, msg):
        try:
            if self.bridge:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                with self.data_lock:
                    self.camera_data[topic_name] = {
                        'image': cv_image, 
                        'timestamp': time.time(),
                        'header': msg.header
                    }
                
                # Call custom callback if registered
                if topic_name in self.topic_callbacks:
                    self.topic_callbacks[topic_name](topic_name, cv_image)
                    
        except Exception as e:
            print(f"Error processing camera message from {topic_name}: {e}")
    
    def get_status_info(self) -> Dict[str, Any]:
        """Get status information about the ROS handler."""
        return {
            'ros2_available': self.ros2_available,
            'running': self.running,
            'node_name': self.node_name,
            'camera_topics_count': len(self.available_camera_topics),
            'active_camera_subscriptions': len(self.camera_data) if self.camera_data else 0
        }
    
    def is_topic_active(self, topic_name: str) -> bool:
        info = self.get_camera_info(topic_name)
        if not info:
            return False
        
        return time.time() - info.get('timestamp', 0) < 2.0
    
    # ============================================================================
    # EXAMPLE: Adding Drone Odometry Topics
    # ============================================================================
    """
    To add drone odometry support, follow this pattern:
    
    1. Add imports:
       from nav_msgs.msg import Odometry
    
    2. Add data storage in __init__:
       self.odometry_data = {}
    
    3. Add subscription methods (similar to camera methods):
       - subscribe_to_odometry(topic_name)
       - unsubscribe_from_odometry(topic_name)
       - get_latest_odometry(topic_name)
    
    4. Add message handler:
       - _handle_odometry_message(topic_name, msg)
    
    5. Update RosNode class with odometry subscription methods
    
    Expected topics: /rs1_drone_1/odom, /rs1_drone_2/odom, etc.
    """
    # ============================================================================

    def cleanup(self):
        print("Cleaning up ROS Handler...")
        self.running = False
        
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=2)
        
        if self.node:
            self.node.destroy_node()
            
        print("ROS Handler cleanup complete.")


class RosNode(Node):
    
    def __init__(self, node_name: str, handler: RosHandler):
        super().__init__(node_name)
        self.handler = handler
        self.camera_subscriptions = {}  # topic_name -> subscription object
    
    def subscribe_to_camera(self, topic_name: str):
        # Unsubscribe if already subscribed
        if topic_name in self.camera_subscriptions:
            self.unsubscribe_from_camera(topic_name)
        
        subscription = self.create_subscription(
            Image,
            topic_name,
            lambda msg, topic=topic_name: self.handler._handle_camera_message(topic, msg),
            10
        )
        
        self.camera_subscriptions[topic_name] = subscription
        self.get_logger().info(f'Subscribed to camera topic: {topic_name}')
    
    def unsubscribe_from_camera(self, topic_name: str):
        if topic_name in self.camera_subscriptions:
            self.destroy_subscription(self.camera_subscriptions[topic_name])
            del self.camera_subscriptions[topic_name]
            self.get_logger().info(f'Unsubscribed from camera topic: {topic_name}')