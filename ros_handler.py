import threading
import time
import queue
import math
from typing import Optional, Dict, Callable, Any
from io import StringIO
import csv


ROS2_AVAILABLE = False;

def ros2_available() -> bool:
    """Return if ROS2 imports succeeded on this system."""
    return ROS2_AVAILABLE

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Pose
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available. ROS Handler will run in simulation mode.")


class RosHandler:
    """
    Centralised ROS2 handler that manages all ROS topics in a single thread.
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
        self.scenario_img_data = {}  # Store latest scenario images by topic
        self.camera_data = {}       # Store latest camera images by topic
        self.telemetry_data = {}    # Store telemetry data
        self.odometry_data = {}     # Store odometry data
        self.topic_callbacks = {}   # Custom callbacks for topics

        # Button-specific components
        self.button_data = {} # Testing Remote Control
        self.available_button_topics = []      

        # For syncing GUI drone selection with controller
        self.current_gui_drone_id = 1
        
        # Camera-specific components
        self.bridge = CvBridge() if ROS2_AVAILABLE else None
        self.available_camera_topics = []
        
        # Odometry-specific components
        self.available_odometry_topics = []

        # Incident-specific components
        self.available_incident_topics = []
        self.available_resolved_incident_topics = []
        self.available_scenario_topics = []

        # Array of drone ids (modular, doesn't need to be 1,2,3 could be 2,5,12)
        self.discovered_drone_ids = []

        # Drone-specific components
        self.drone_amount = 0
        
        # Remove periodic discovery - topics are fixed at startup
        self.topics_discovered = False
        
        if self.ros2_available:
            self._start_ros_node()
    
    # We can probably delete this now.
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
        expected_incidents = []

        for drone_id in range(1, num_drones + 1):
            for camera_type in camera_types:
                topic_name = f"/rs1_drone_{drone_id}/{camera_type}/image"
                expected_topics.append(topic_name)
            expected_incidents.append(f"/rs1_drone_{drone_id}/incident")
        
        with self.data_lock:
            self.available_camera_topics = expected_topics
            self.available_incident_topics = expected_incidents
        
        # Should I remove this now?
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
                    # self._discover_topics()
                    self.discover_topics()
                    self.topics_discovered = True
                
        except Exception as e:
            print(f"ROS Handler thread error: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
            print("ROS Handler thread terminated")
    
    def discover_topics(self):
        """Discover available ROS topics."""
        if not self.node:
            return
        
        try:
            topic_list = self.node.get_topic_names_and_types()
            camera_topics, odometry_topics, available_controller_topic, button_topics, incident_topics, resolved_incident_topics, scenario_topics = [], [], [], [], [], [], []
            
            drone_ids = set()

            for topic_name, topic_types in topic_list:
                if 'sensor_msgs/msg/Image' in topic_types:
                    if topic_name.endswith('/image'):
                        camera_topics.append(topic_name)
                    elif topic_name.endswith('/scenario_img'):
                        scenario_topics.append(topic_name)
                if 'nav_msgs/msg/Odometry' in topic_types:
                    odometry_topics.append(topic_name)
                if '/rs1_teensyjoy/button_control' in topic_name: # Here I directly mapped the topic name as there are other String Type Topics
                    button_topics.append(topic_name)
                if topic_name.endswith('/incident') and 'std_msgs/msg/String' in topic_types:
                    incident_topics.append(topic_name)
                if topic_name.endswith('/resolved_incident') and 'std_msgs/msg/String' in topic_types:
                    resolved_incident_topics.append(topic_name)

                # Collect drone ids from any segment like rs1_drone_<id>
                parts = [p for p in topic_name.split('/') if p]
                for seg in parts:
                    if seg.startswith('rs1_drone_'):
                        suffix = seg[len('rs1_drone_'):]
                        if suffix.isdigit():
                            drone_ids.add(int(suffix))

                # Build proactive incident topics from discovered drone ids
                proactive_incidents = [f"/rs1_drone_{i}/incident" for i in sorted(drone_ids)]
                target_incident_topics = sorted(set(incident_topics) | set(proactive_incidents))
                
                proactive_resolved_incidents = [f"/rs1_drone_{i}/resolved_incident" for i in sorted(drone_ids)]
                target_resolved_incident_topics = sorted(set(resolved_incident_topics) | set(proactive_resolved_incidents))

            # Update available camera topics
            with self.data_lock:
                self.available_camera_topics = camera_topics
                self.available_odometry_topics = odometry_topics
                self.available_incident_topics = target_incident_topics
                self.available_resolved_incident_topics = target_resolved_incident_topics
                self.discovered_drone_ids = sorted(drone_ids)
                self.available_button_topics = button_topics
                self.available_scenario_topics = scenario_topics
            
            # Subscribe to all discovered scenario image topics
            for topic in self.available_scenario_topics:
                    self.subscribe_to_scenario_image_topic(topic)
                    print(f"Auto-subscribed to {len(self.available_scenario_topics)} scenario image topics")

            print(f"Discovered drone ids: {sorted(drone_ids)}")
            print(f"Discovered {len(camera_topics)} camera topics: {camera_topics}")
            print(f"Discovered {len(scenario_topics)} scenario topic: {scenario_topics}")
            print(f"Discovered {len(odometry_topics)} odometry topics: {odometry_topics}")
            print(f"Discovered {len(incident_topics)} incident topics: {incident_topics}")
            print(f"Discovered {len(resolved_incident_topics)} resolved incident topics: {resolved_incident_topics}")
            print(f"Discovered {len(button_topics)} button topics: {button_topics}")
            print(f"Proactive incident subscriptions (all): {target_incident_topics}")
            print(f"Proactive resolved incident subscriptions (all): {target_resolved_incident_topics}")
            
        except Exception as e:
            print(f"Error discovering topics: {e}")
    

    def subscribe_to_incident_topic(self, topic_name: str) -> bool:
        """
        Original subscribe_to_incident_topic function - Subscribe to a ROS2 incident topic.
        Attempts to connect the node to a specified ROS2 topic for receiving incident messages.
        Returns True if the subscription is successful; otherwise logs an error and returns False.
        Args:
            - topic_name (str): The full name of the ROS2 topic to subscribe to (e.g., "/rs1/incidents").
        Returns:
            - (bool): True if subscription succeeded, False if ROS2 is unavailable or an error occurred.
        Side Effects:
            - Prints subscription status or error messages to the console.
        """

        if not self.node or not self.ros2_available:
            return False
        try:
            self.node.subscribe_to_incident(topic_name)
            print(f"Subscribed to incident topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to subscribe to {topic_name}: {e}")
            return False

    def subscribe_to_resolved_incident_topic(self, topic_name: str) -> bool:
        """
        Subscribe to a ROS2 resolved incident topic.
        Attempts to connect the node to a specified ROS2 topic for receiving resolved incident messages.
        Returns True if the subscription is successful; otherwise logs an error and returns False.
        Args:
            - topic_name (str): The full name of the ROS2 topic to subscribe to (e.g., "/rs1_drone_1/resolved_incident").
        Returns:
            - (bool): True if subscription succeeded, False if ROS2 is unavailable or an error occurred.
        Side Effects:
            - Prints subscription status or error messages to the console.
        """

        if not self.node or not self.ros2_available:
            return False
        try:
            self.node.subscribe_to_resolved_incident(topic_name)
            print(f"Subscribed to resolved incident topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to subscribe to {topic_name}: {e}")
            return False


    # Controller related functions
    def call_teensy_connect(self, connect: bool):
        """
        Call_teensy_connect function - Trigger ROS2 service to connect or disconnect Teensy joystick.
        Sends a service call request via the ROS2 node to either establish or terminate a connection
        with the Teensy-based joystick controller.
        Args:
            - connect (bool): True to connect, False to disconnect.
        Returns:
            - (bool): True if the service call succeeded, False if ROS2 is unavailable or the call failed.
        Side Effects:
            - Prints an error message if ROS2 is not available.
        """

        # True - connect, False - disconnect
        if not self.node or not self.ros2_available:
            print("ROS2 not available, cannot call service")
            return False
        return self.node.call_connect_service(connect)

    def publish_current_drone_id(self, drone_id: int):
        """
        Publish_current_drone_id function - Publish the currently selected drone ID to ROS2.
        Updates the shared GUI drone ID state and publishes it via the ROS2 node to inform other
        components which drone is currently active or being controlled.
        Args:
            - drone_id (int): The numeric ID of the selected drone to publish.
        Returns:
            - (bool): True if successfully published, False if ROS2 is unavailable or an error occurred.
        Side Effects:
            - Updates self.current_gui_drone_id within a thread-safe lock.
            - Prints the publish result or error message to the console.
        """

        if not self.node or not self.ros2_available:
            return False
            
        try:
            with self.data_lock:
                self.current_gui_drone_id = drone_id
                
            self.node.publish_drone_id(drone_id)
            print(f"Published GUI drone ID: {drone_id}")
            return True
        except Exception as e:
            print(f"Failed to publish drone ID: {e}")
            return False

    def get_current_gui_drone_id(self) -> int:
        with self.data_lock:
            return self.current_gui_drone_id


    # Button related functions based off the camera and odometry ones
    # Some may be redundant
    def subscribe_to_button_topic(self, topic_name: str) -> bool:
        """
        Subscribe_to_button_topic function - Subscribe to a ROS2 topic for button input events.
        Attempts to connect the node to a specified ROS2 topic that publishes controller or GUI button states.
        Returns True if the subscription succeeds; otherwise logs the exception and returns False.
        Args:
            - topic_name (str): The full name of the ROS2 topic to subscribe to (e.g., "/rs1/button_events").
        Returns:
            - (bool): True if subscription succeeded, False if ROS2 is unavailable or an error occurred.
        Side Effects:
            - Prints subscription status or error details to the console.
        """

        if not self.node or not self.ros2_available:
            return False
        try:
            self.node.subscribe_to_button(topic_name)
            print(f"Subscribed to button topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to subscribe to {topic_name}: {e}")
            return False
        
    def get_available_button_topics(self) -> list:
        with self.data_lock:
            return self.available_button_topics.copy()

    def _handle_controller_message(self, topic_name: str, msg):
        try:
            with self.data_lock:
                self.controller_data[topic_name] = {
                    "axes": msg.axes,
                    "buttons": msg.buttons,
                    "timestamp": time.time()
                }
            if topic_name in self.topic_callbacks:
                self.topic_callbacks[topic_name](topic_name, msg.data)
        except Exception as e:
            print(f"Error processing controller message from {topic_name}: {e}")
    
    def get_latest_button(self, topic_name: str): 
        with self.data_lock:
            return self.button_data.get(topic_name, {})
        
    def get_available_incident_topics(self) -> list:
        with self.data_lock:
            return self.available_incident_topics.copy()
        
    def get_latest_incident(self, topic_name: str): #AI
        with self.data_lock:
            seq = getattr(self, "incident_data", {}).get(topic_name) 
            return dict(seq[-1]) if seq else None

    def get_available_resolved_incident_topics(self) -> list:
        with self.data_lock:
            return self.available_resolved_incident_topics.copy()
        
    def get_latest_resolved_incident(self, topic_name: str):
        with self.data_lock:
            seq = getattr(self, "resolved_incident_data", {}).get(topic_name) 
            return dict(seq[-1]) if seq else None

    
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
    
    def get_available_scenario_topics(self) -> list:
        with self.data_lock:
            return self.available_scenario_topics.copy()
    
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
    
    def _handle_controller_message(self, topic_name: str, msg):
        try:
            with self.data_lock:
                self.controller_data[topic_name] = {
                    "axes": msg.axes,
                    "buttons": msg.buttons,
                    "timestamp": time.time()
                }
            if topic_name in self.topic_callbacks:
                self.topic_callbacks[topic_name](topic_name, msg.data)
        except Exception as e:
            print(f"Error processing controller message from {topic_name}: {e}")
        
    
    # Button Callback function
    def _handle_button_message(self, topic_name: str, msg):
        try:
            with self.data_lock:
                self.button_data[topic_name] = {
                    "data": msg.data,
                    "timestamp": time.time()
                }
            if topic_name in self.topic_callbacks:
                self.topic_callbacks[topic_name](topic_name, msg.data)
        except Exception as e:
            print(f"Error processing button message from {topic_name}: {e}")
    

    def subscribe_to_odometry_topic(self, topic_name: str) -> bool:
        if not self.node or not self.ros2_available:
            return False

        try:
            self.node.subscribe_to_odometry(topic_name)
            print(f"Subscribed to odometry topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to subscribe to {topic_name}: {e}")
            return False

    def unsubscribe_from_odometry_topic(self, topic_name: str) -> bool:
        if not self.node:
            return False

        try:
            self.node.unsubscribe_from_odometry(topic_name)
            with self.data_lock:
                if topic_name in self.odometry_data:
                    del self.odometry_data[topic_name]
            print(f"Unsubscribed from odometry topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to unsubscribe from {topic_name}: {e}")
            return False
        
    def subscribe_to_scenario_image_topic(self, topic_name: str) -> bool:
        """
        Subscribe to a scenario image topic (e.g., /rs1_drone_1/scenario_img).
        
        Args:
            topic_name: Full topic name like "/rs1_drone_1/scenario_img"
        
        Returns:
            bool: True if subscription succeeded, False otherwise
        """
        if not self.node or not self.ros2_available:
            return False
        
        try:
            self.node.subscribe_to_scenario_image(topic_name)
            print(f"Subscribed to scenario image topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to subscribe to scenario image {topic_name}: {e}")
            return False

    def unsubscribe_from_scenario_image_topic(self, topic_name: str) -> bool:
        """
        Unsubscribe from a scenario image topic.
        
        Args:
            topic_name: Full topic name to unsubscribe from
        
        Returns:
            bool: True if unsubscription succeeded, False otherwise
        """
        if not self.node:
            return False
        
        try:
            self.node.unsubscribe_from_scenario_image(topic_name)
            with self.data_lock:
                if topic_name in self.scenario_img_data:
                    del self.scenario_img_data[topic_name]
            print(f"Unsubscribed from scenario image topic: {topic_name}")
            return True
        except Exception as e:
            print(f"Failed to unsubscribe from scenario image {topic_name}: {e}")
            return False

    def get_latest_scenario_image(self, topic_name: str):
        """
        Get the latest scenario image from a topic.
        
        Args:
            topic_name: Full topic name like "/rs1_drone_1/scenario_img"
        
        Returns:
            numpy.ndarray or None: OpenCV image (BGR format) if available, None otherwise
        """
        with self.data_lock:
            scenario_info = self.scenario_img_data.get(topic_name, {})
            image = scenario_info.get('image', None)
            # Return a copy to avoid threading issues
            return image.copy() if image is not None else None

    def is_scenario_image_active(self, topic_name: str) -> bool:
        """
        Check if a scenario image topic has recent data (within last 5 seconds).
        
        Args:
            topic_name: Full topic name to check
        
        Returns:
            bool: True if topic has fresh data, False otherwise
        """
        with self.data_lock:
            scenario_info = self.scenario_img_data.get(topic_name, {})
            if not scenario_info:
                return False
            timestamp = scenario_info.get('timestamp', 0)
            return time.time() - timestamp < 5.0

    def _handle_scenario_image_message(self, topic_name: str, msg):
        """
        Internal callback to handle incoming scenario image messages.
        Converts ROS Image message to OpenCV format and stores it.
        
        Args:
            topic_name: Topic the message came from
            msg: sensor_msgs/Image message
        """
        try:
            if self.bridge:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                with self.data_lock:
                    self.scenario_img_data[topic_name] = {
                        'image': cv_image,
                        'timestamp': time.time(),
                        'header': msg.header
                    }
                
                if topic_name in self.topic_callbacks:
                    self.topic_callbacks[topic_name](topic_name, cv_image)
                    
        except Exception as e:
            print(f"Error processing scenario image from {topic_name}: {e}")

    def get_latest_odometry(self, topic_name: str): # The parameters might be wrong
        with self.data_lock:
            return self.odometry_data.get(topic_name, {})

    def get_drone_pose(self, topic_name: str):# The parameters might be wrong
        with self.data_lock:
            info = self.odometry_data.get(topic_name)
            if not info:
                return None
            (x, y, z) = info["position"]
            (roll, pitch, yaw) = info["rpy"]
        return x, y, z, roll, pitch, yaw

    def get_available_odometry_topics(self) -> list:
        with self.data_lock:
            return self.available_odometry_topics.copy()
    # START AI suggested and wrote this, and I'm not sure if we need it, but I'll leave it here since it could be useful
    def get_drone_pose_by_id(self, drone_id: int):
        return self.get_drone_pose(f"/rs1_drone_{drone_id}/odom")

    def is_odom_active(self, topic_name: str) -> bool:
        with self.data_lock:
            ts = self.odometry_data.get(topic_name, {}).get("timestamp")
        return ts is not None and (time.time() - ts) < 2.0

    def get_latest_odometry(self, topic_name: str):
        with self.data_lock:
            data = self.odometry_data.get(topic_name)
            return data.copy() if data else None
    # END AI suggested and wrote this, and I'm not sure if we need it, but I'll leave it here since it could be useful

    def _handle_odometry_message(self, topic_name: str, msg):
        try:
            position = msg.pose.pose.position      # x, y, z
            orientation = msg.pose.pose.orientation   # quaternion x, y, z, w   
            ox, oy, oz, ow = orientation.x, orientation.y, orientation.z, orientation.w
            norm = math.sqrt(ox*ox + oy*oy + oz*oz + ow*ow) or 1.0 # Normalise to be robust
            x, y, z, w = ox/norm, oy/norm, oz/norm, ow/norm         
            # RPY conversion
            ##  CONFIRM MATH
            sinr_cosp = 2*(w*x + y*z); cosr_cosp = 1 - 2*(x*x + y*y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = 2*(w*y - z*x)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            siny_cosp = 2*(w*z + x*y); cosy_cosp = 1 - 2*(y*y + z*z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            now = time.time()

            with self.data_lock:
                self.odometry_data[topic_name] = {
                    "position": (position.x, position.y, position.z),
                    "orientation": (orientation.x, orientation.y, orientation.z, orientation.w),
                    "rpy": (roll, pitch, yaw),
                    'odometry_topics_count': len(self.odometry_data), # can be nice to have? idk, you can delete this @Marcus
                    "timestamp": now,         # for LIVE/STALE checks
                    "header": msg.header      # for frame_id, ROS stamp, etc.

                }

            # Optional: custom callbacks can receive parsed odom
            if topic_name in self.topic_callbacks:
                self.topic_callbacks[topic_name](topic_name, self.odometry_data[topic_name])


        except Exception as e:
            print(f"Error processing odometry message from {topic_name}: {e}")

    def _handle_incident_message(self, topic_name: str, msg):
        """
        Parse a single std_msgs/String incident message and append it to the per-topic store.

        Expected CSV (one line, no header):
        drone_id, incident_id, title, severity, iso_time, x, y, z, description
        """
        try:
            # Lazy-init store so __init__ doesn't need edits
            if not hasattr(self, "incident_data"):
                self.incident_data = {}  # topic_name -> [incident dict, ...]

            # std_msgs/String keeps payload in .data
            payload = msg.data

            # CSV -> dict (handles quoted commas in description)
            f = StringIO(payload)
            fieldnames = ["drone_id","incident_id","title","severity","iso_time","x","y","z","description"]
            reader = csv.DictReader(f, delimiter=",", fieldnames=fieldnames, skipinitialspace=True)
            row = next(reader, None)
            if row is None:
                print(f"INCIDENT PARSE FAIL: empty payload on topic {topic_name}")
                return

            # Required fields (drone_id numeric)
            try:
                drone_id = int((row.get("drone_id") or "").strip())
            except Exception:
                print(f"INCIDENT PARSE FAIL: bad drone_id. Row={row}")
                return

            incident_id = (row.get("incident_id") or "").strip() or f"{drone_id}-{int(time.time()*1000)}"
            title = (row.get("title") or "").strip() or "Unknown"

            # Severity -> int in [1..3]
            try:
                severity = int((row.get("severity") or "1").strip())
            except Exception:
                severity = 1
            severity = max(1, min(3, severity))

            iso_time = (row.get("iso_time") or "").strip()

            # Coords -> floats
            def _to_float(v):
                try:
                    return float((v or "").strip())
                except Exception:
                    return None

            x = _to_float(row.get("x"))
            y = _to_float(row.get("y"))
            z = _to_float(row.get("z"))
            if x is None or y is None or z is None:
                print(f"INCIDENT PARSE FAIL: bad coords. Row={row}")
                return

            desc = (row.get("description") or "").strip()

            incident = {
                "id": incident_id,
                "title": title,
                "time": iso_time,           # display string (ISO)
                "severity": severity,       # 1..3
                "drone": drone_id,          # int
                "drone_coords": (x, y),     # floats
                "altitude": z,              # float
                "description": desc,
                "status": "open",
                "last_update": time.time(),
            }

            # Append latest per topic (mirrors your odom storage style)
            with self.data_lock:
                self.incident_data.setdefault(topic_name, []).append(incident)

            # Optional: notify any custom callback registered for this topic
            cb = self.topic_callbacks.get(topic_name)
            if cb:
                cb(topic_name, incident)

        except Exception as e:
            print(f"Error processing INCIDENT message from {topic_name}: {e}")

    def _handle_resolved_incident_message(self, topic_name: str, msg):
        """
        Parse a single std_msgs/String resolved incident message and append it to the per-topic store.

        Expected CSV (one line, no header):
        drone_id, incident_id, title, severity, iso_time, x, y, z, status
        """
        try:
            # Lazy-init store
            if not hasattr(self, "resolved_incident_data"):
                self.resolved_incident_data = {}  # topic_name -> [resolved incident dict, ...]

            # std_msgs/String keeps payload in .data
            payload = msg.data

            # CSV -> dict
            f = StringIO(payload)
            fieldnames = ["drone_id","incident_id","title","severity","iso_time","x","y","z","status"]
            reader = csv.DictReader(f, delimiter=",", fieldnames=fieldnames, skipinitialspace=True)
            row = next(reader, None)
            if row is None:
                print(f"RESOLVED INCIDENT PARSE FAIL: empty payload on topic {topic_name}")
                return

            # Required fields
            try:
                drone_id = int((row.get("drone_id") or "").strip())
            except Exception:
                print(f"RESOLVED INCIDENT PARSE FAIL: bad drone_id. Row={row}")
                return

            incident_id = (row.get("incident_id") or "").strip() or f"{drone_id}-resolved-{int(time.time()*1000)}"
            title = (row.get("title") or "").strip() or "Unknown"

            # Severity -> int in [1..3]
            try:
                severity = int((row.get("severity") or "1").strip())
            except Exception:
                severity = 1
            severity = max(1, min(3, severity))

            iso_time = (row.get("iso_time") or "").strip()

            # Coords -> floats
            def _to_float(v):
                try:
                    return float((v or "").strip())
                except Exception:
                    return None

            x = _to_float(row.get("x"))
            y = _to_float(row.get("y"))
            z = _to_float(row.get("z"))
            if x is None or y is None or z is None:
                print(f"RESOLVED INCIDENT PARSE FAIL: bad coords. Row={row}")
                return

            status = (row.get("status") or "RESOLVED").strip()

            resolved_incident = {
                "id": incident_id,
                "title": title,
                "time": iso_time,
                "severity": severity,
                "drone": drone_id,
                "drone_coords": (x, y),
                "altitude": z,
                "status": status,
                "last_update": time.time(),
            }

            # Append latest per topic
            with self.data_lock:
                self.resolved_incident_data.setdefault(topic_name, []).append(resolved_incident)

            # Optional: notify any custom callback registered for this topic
            cb = self.topic_callbacks.get(topic_name)
            if cb:
                cb(topic_name, resolved_incident)

        except Exception as e:
            print(f"Error processing RESOLVED INCIDENT message from {topic_name}: {e}")


    def get_drone_id_topics_amount(self) -> int:
        topic_list = self.node.get_topic_names_and_types()
        print(f"[ROS Handler] Topics visible: {topic_list}")
        drone_ids = set()
        for topic, _ in topic_list:
            if topic.startswith("/rs1_drone_"):
                try:
                    # Extract the drone id after the prefix
                    # Assuming topic format is like "/rs1_drone_1/odom"
                    drone_id = int(topic[len("/rs1_drone_"):].split("/")[0])
                    drone_ids.add(drone_id)
                except Exception as e:
                    print(f"Error parsing drone id from topic {topic}: {e}")
        self.drone_amount = list(drone_ids)
        return len(self.drone_amount)

    def cleanup(self):
        print("Cleaning up ROS Handler...")
        self.running = False
        
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=2)
        
        if self.node:
            self.node.destroy_node()
            
        print("ROS Handler cleanup complete.")

if ROS2_AVAILABLE:

    class RosNode(Node):
        
        def __init__(self, node_name: str, handler: RosHandler):
            super().__init__(node_name)
            self.handler = handler
            self.camera_subscriptions = {}      # topic_name -> subscription object
            self.odometry_subscriptions = {}    # topic_name -> subscription object
            self.incident_subscriptions = {}    # topic_name -> incident object
            self.resolved_incident_subscriptions = {}  # topic_name -> resolved incident object
            self.button_subscriptions = {}      # For buttons
            self.controller_subscription = {}   # For controller

            self.controller_subscription = {}   # For controller

            # For connecting to the controller 
            self.teensy_connect_client = self.create_client(SetBool, '/rs1_teensyjoy/connect')

            self.drone_id_publisher = self.create_publisher(
                String,
                '/rs1_gui/current_drone_id',
                10
            )

        def publish_drone_id(self, drone_id: int):
            msg = String()
            msg.data = str(drone_id)
            self.drone_id_publisher.publish(msg)

        
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


        def call_connect_service(self, connect: bool) -> bool:
            """Call the teensy connect service."""
            # Check if service is available
            if not self.teensy_connect_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('Teensy connect service not available')
                return False
            
            # Create request
            request = SetBool.Request()
            request.data = connect
            
            try:
                # Call service asynchronously
                future = self.teensy_connect_client.call_async(request)
                
                # Wait for result (increase timeout if needed)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    try:
                        response = future.result()
                        if response.success:
                            self.get_logger().info(f'Teensy {"connected" if connect else "disconnected"}: {response.message}')
                            return True
                        else:
                            self.get_logger().warn(f'Teensy connect failed: {response.message}')
                            return False
                    except Exception as e:
                        self.get_logger().error(f'Failed to get service response: {e}')
                        return False
                else:
                    self.get_logger().warn('Service call timed out after 5 seconds')
                    return False
                    
            except Exception as e:
                self.get_logger().error(f'Service call failed: {str(e)}')
                return False

        
        def unsubscribe_from_camera(self, topic_name: str):
            if topic_name in self.camera_subscriptions:
                self.destroy_subscription(self.camera_subscriptions[topic_name])
                del self.camera_subscriptions[topic_name]
                self.get_logger().info(f'Unsubscribed from camera topic: {topic_name}')

        def subscribe_to_odometry(self, topic_name: str):
                if topic_name in self.odometry_subscriptions:
                    self.unsubscribe_from_odometry(topic_name)
                subscription = self.create_subscription (
                    Odometry,
                    topic_name,
                    lambda msg, topic=topic_name: self.handler._handle_odometry_message(topic, msg),
                    10
                )
                self.odometry_subscriptions[topic_name] = subscription
                self.get_logger().info(f'Subscribed to odometry topic: {topic_name}')

        def unsubscribe_from_odometry(self, topic_name: str):
            if topic_name in self.odometry_subscriptions:
                self.destroy_subscription(self.odometry_subscriptions[topic_name])
                del self.odometry_subscriptions[topic_name]
                self.get_logger().info(f'Unsubscribed from odometry topic: {topic_name}')

        def subscribe_to_incident(self, topic_name: str):
            if topic_name in self.incident_subscriptions:
                self.destroy_subscription(self.incident_subscriptions[topic_name])
                del self.incident_subscriptions[topic_name]
                self.get_logger().info(f'Unsubscribed from incident topic: {topic_name}')
            subscription = self.create_subscription(
                String,
                topic_name,
                lambda msg, topic=topic_name: self.handler._handle_incident_message(topic, msg),
                10
            )
        
        def subscribe_to_resolved_incident(self, topic_name: str):
            if topic_name in self.resolved_incident_subscriptions:
                self.destroy_subscription(self.resolved_incident_subscriptions[topic_name])
                del self.resolved_incident_subscriptions[topic_name]
                self.get_logger().info(f'Unsubscribed from resolved incident topic: {topic_name}')
            subscription = self.create_subscription(
                String,
                topic_name,
                lambda msg, topic=topic_name: self.handler._handle_resolved_incident_message(topic, msg),
                10
            )
            self.resolved_incident_subscriptions[topic_name] = subscription
            self.get_logger().info(f'Subscribed to resolved incident topic: {topic_name}')
  
        def unsubscribe_from_incident(self, topic_name: str):
            if topic_name in self.incident_subscription:
                self.destroy_subscription(self.incident_subscription[topic_name])
                del self.incident_subscription[topic_name]
                self.get_logger().info(f'Unsubscribed from incident topic: {topic_name}')


        def call_connect_service(self, connect: bool) -> bool:
            if not self.teensy_connect_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Teensy connect service not available')
                return False
            
            request = SetBool.Request()
            request.data = connect
            
            try:
                future = self.teensy_connect_client.call_async(request)
                # We're in the ROS thread, so we can wait briefly
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f'Teensy connect service: {response.message}')
                        return True
                    else:
                        self.get_logger().warn(f'Teensy connect failed: {response.message}')
                        return False
                else:
                    self.get_logger().warn('Service call timed out')
                    return False
                    
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return False
        
        # Finally found where the functions are lmfao
        def subscribe_to_button(self, topic_name: str):
            if topic_name in self.button_subscriptions:
                self.unsubscribe_from_button(topic_name)

            subscription = self.create_subscription(
                String, 
                topic_name,
                lambda msg, topic=topic_name: self.handler._handle_button_message(topic, msg),
                10
            )
            self.button_subscriptions[topic_name] = subscription
            self.get_logger().info(f'Subscribed to button topic: {topic_name}')

        # Not sure if needed but added as well
        def unsubscribe_from_button(self, topic_name: str):
            if topic_name in self.button_subscriptions:
                self.destroy_subscription(self.button_subscriptions[topic_name])
                del self.button_subscriptions[topic_name]
                self.get_logger().info(f'Unsubscribed from button topic: {topic_name}')

        def subscribe_to_scenario_image(self, topic_name: str):
            """
            Subscribe to a scenario image topic in the ROS node.
            
            Args:
                topic_name: Full topic name like "/rs1_drone_1/scenario_img"
            """
            # Unsubscribe if already subscribed
            if topic_name in self.camera_subscriptions:
                self.unsubscribe_from_scenario_image(topic_name)
            
            subscription = self.create_subscription(
                Image,
                topic_name,
                lambda msg, topic=topic_name: self.handler._handle_scenario_image_message(topic, msg),
                10
            )
            
            # Store in camera_subscriptions since they use the same Image type
            self.camera_subscriptions[topic_name] = subscription
            self.get_logger().info(f'Subscribed to scenario image topic: {topic_name}')

        def unsubscribe_from_scenario_image(self, topic_name: str):
            """
            Unsubscribe from a scenario image topic.
            
            Args:
                topic_name: Full topic name to unsubscribe from
            """
            if topic_name in self.camera_subscriptions:
                self.destroy_subscription(self.camera_subscriptions[topic_name])
                del self.camera_subscriptions[topic_name]
                self.get_logger().info(f'Unsubscribed from scenario image topic: {topic_name}')


else:
    class RosNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 not available; RosNode cannot be used in simulation.")