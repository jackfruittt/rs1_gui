import pygame, math, time, subprocess, threading, random, os, signal
from constants import *
from pathlib import Path
from camera_ros import CameraComponent

class SpawnPromptPanel:
    """
    Spawn Prompt Class, this brings up the spawn window to enable the User to input their desired drone count.
    When drone count entered, it runs the comp_drone_spawner.sh to start gazebo simulator and spawn drones in the environment.

    """
    def __init__(self, app, fonts, rosAvailable):
        """
        Initialises the spawn prompt panel variables.
        
        Args:
            - app: The main application instance.
            - fonts: Dictionary of Pygame font objects.
            - rosAvailable: Boolean indicating if ROS2 is available on the system.

        """
        self.app = app
        self.fonts = fonts
        self.button_rects = []
        self.user_input = ""     # only digits will be accepted
        self.start_time = time.time()
        self.load_icon = pygame.image.load("media/images/3drone.png").convert_alpha()
        self.load_icon = pygame.transform.scale(self.load_icon, (80,80))
        self.bg = pygame.image.load("media/images/bg1.png").convert()
        self.logo = pygame.image.load("media/images/hhlogo.png").convert_alpha()
        self.runningScript = False
        self.rosAvailable = rosAvailable
        self.simulateThread = threading.Thread(target=self.simulateEnvSetup)
        self.waitSimThread = threading.Thread(target=self.waitForSim)
        self.requestCount = 1

    def draw_prompt(self, screen):
        """
        Draw the full-screen spawn prompt interface.
        Here the user can input the number of drones to spawn (1-10) and submit the request.

        Once the request is submitted, a loading animation is shown until the simulator is ready.
        
        Args:
            - screen: The Pygame surface to draw on.

        """
        self.button_rects = []

        panel = pygame.Surface((1900, 860))
        panel.fill(BLACK)
        panel.blit(self.bg, (0,0))

        center_x = 1900 // 2

        # --- Logo placeholder (optional left side)
        logo_rect = pygame.Rect(center_x - 300, 200, 80, 80)
        panel.blit(self.logo, (center_x - 220, 200))

        # --- Title (centered horizontally)
        title = self.fonts['inter_bold_large'].render("Horizon Hive", True, WHITE)
        title_rect = title.get_rect(center=(center_x, logo_rect.top + 40))
        panel.blit(title, title_rect)

        # --- Prompt text (centered horizontally)
        question = self.fonts['inter_small'].render("How many drones would you like to spawn? (1-6)", True, WHITE)
        question_rect = question.get_rect(center=(center_x, title_rect.bottom + 40))
        

        # --- Input + Submit group (centered as a unit)
        input_w, input_h = 220, 50
        btn_w = 50
        group_w = input_w + btn_w
        group_x = center_x - group_w // 2
        group_y = question_rect.bottom + 40

        # Input box
        input_rect = pygame.Rect(group_x, group_y, input_w, input_h)
        dark_input = pygame.Rect(group_x, group_y, input_w+btn_w, input_h)

        txt = self.user_input if self.user_input else ""
        text_surface = self.fonts['inter_medium'].render(txt, True, WHITE)
        if self.runningScript is not True:
            pygame.draw.rect(panel, DARK_GRAY, dark_input)
            pygame.draw.rect(panel, WHITE, input_rect, 2)
            panel.blit(text_surface, text_surface.get_rect(center=input_rect.center))

            # Submit button right next to input
            submit_rect = pygame.Rect(input_rect.right, group_y, btn_w, input_h)
            pygame.draw.rect(panel, WHITE, submit_rect, 2)
            arrow_surface = self.fonts['inter_medium'].render(">", True, WHITE)
            panel.blit(arrow_surface, arrow_surface.get_rect(center=submit_rect.center))

            # Save absolute rects
            self.button_rects.append((submit_rect, "submit"))
        else:
            question = self.fonts['inter_small'].render("Connecting to drones... Please wait", True, WHITE)
            # --- Drone sine-wave icon (centered horizontally)
            t = time.time() - self.start_time
            dy = int(math.sin(t * 7) * 10)  # oscillate ±30px
            icon_size = 80
            panel.blit(self.load_icon, (center_x - icon_size // 2, input_rect.bottom + 120 + dy))
        panel.blit(question, question_rect)
        # Final blit
        screen.blit(panel, (0, 0))

    def startSim(self):
        self.runningScript = True
        self.requestCount = int(self.user_input)
        if self.rosAvailable is True:
            print("ROS found")
            self.runScript(self.user_input)
        else:
            print("ROS NOT FOUND")
            self.simulateThread.start()

    def handle_key(self, event):
        """
        Restrict input to digits only
        
        This function handles key events for the spawn prompt. It allows only digit inputs (0-9) and handles backspace and enter keys.
        On enter, if the input is valid, it starts the drone spawning process.

        Args:
            - event: The Pygame event to handle.

        """
        if self.runningScript is not True:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_BACKSPACE:
                    self.user_input = self.user_input[:-1]
                elif event.key == pygame.K_RETURN:
                    if self.user_input != "":
                        self.startSim()
                    return
                elif event.unicode.isdigit():
                    if int(self.user_input + event.unicode) <= 10 and int(self.user_input + event.unicode) >= 1:
                        self.user_input += event.unicode
        return None

    def runScript(self, numDrones):
        """
        Run the script to start the simulator and spawn drones

        This function executes the drone spawner script comp_drone_spawner.sh with the specified number of drones requested by the user.
        It sets up the necessary environment variables for ROS2 and starts a thread to wait for the simulator to be ready.

        Args:
            - numDrones: The number of drones to spawn as a string.
        
        Notes:
            - The function handles environment setup for Qt/Wayland compatibility.
            - It ensures both GUI and spawner run on the same ROS domain.

        """
        import os, subprocess, threading

        count = int(numDrones)
        if self.rosAvailable:
            env = os.environ.copy()
            # keep GUI and spawner on the same domain
            env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "0")

            # ---- Qt / Wayland fixes ----
            env.pop("QT_PLUGIN_PATH", None)   # remove OpenCV's Qt plugin path
            env["QT_QPA_PLATFORM"] = "xcb"    # force xcb even if on Wayland
            # -----------------------------
            argv = ["./comp_drone_spawner.sh", str(count), "gazebo:=false", "rviz:=false"]
            proc = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env
            )

            # stream logs so the PIPE never fills
            def _pump():
                try:
                    if proc.stdout:
                        for line in proc.stdout:
                            print(f"[spawner] {line.rstrip()}")
                except Exception as e:
                    print(f"[spawner-log-error] {e}")

            threading.Thread(target=_pump, daemon=True).start()

        self.waitSimThread.start()
    
    def killSim(self):
        """
        Kill the simulator processes

        This function terminates all relevant simulator processes to ensure a clean shutdown.

        """
        commands = ['pkill -f "ign gazebo"', 'pkill -f "parameter_bridge"', 'pkill -f "robot_state_publisher"', 'pkill -f "multi_drone_composition_controller"', 'pkill -f "controller_node"']
        for command in commands:
            subprocess.Popen(command, shell=True)
    
    def simulateEnvSetup(self):
        """
        SimulateEnvSetup function - Initialize simulated environment for testing.
        Generates a number of simulated drones and incidents to populate the GUI when running
        without live ROS2 data, mimicking a real deployment environment.
        Args:
            - None
        Returns:
            - None
        Side Effects:
            - Populates self.app.drones with generated drone objects.
            - Assigns default waypoints to drones if available.
            - Populates self.app.incidents with randomly generated incidents.
            - Sets self.app.simReady to True to indicate simulation initialization is complete.
        """

        time.sleep(0.01)
        for _ in range(self.requestCount):
            self.app.drones.append(self.app.generate_drone())
            if _ <= len(self.app.default_waypoints)-1:
                self.app.drones[-1]['waypoints'] = self.app.default_waypoints[_]


        for _ in range(random.randint(3,10)):
            self.app.incidents.append(self.app.generate_random_incident())
        self.app.simReady = True


    def waitForSim(self):
        """
        WaitForSim function - Block until ROS/Gazebo simulation is discoverable, then initialize UI.
        Waits for drones and related ROS2 topics to appear (with staged sleeps and a max timeout). On success,
        creates drone entries, discovers/validates camera topics, initializes CameraComponent, and subscribes
        to odometry/incident/button topics. On timeout or error, falls back to generating simulated drones
        and incidents (like simulateEnvSetup). Marks the UI as ready at the end.
        Args:
            - None
        Returns:
            - None
        Side Effects:
            - Sleeps for staged delays (10s + 15s) before polling for topics.
            - Repeatedly calls ros_handler.discover_topics() while polling.
            - Populates self.app.drones with discovered (or simulated) entries.
            - Optionally deep-copies default waypoints into each drone.
            - Initializes self.app.camera_component and subscribes to available topics.
            - Updates self.app.odometry_topics, self.app.incident_topics, self.app.button_topics.
            - Sets self.app.simReady = True when initialization completes or on fallback.
            - Prints progress and diagnostic information to the console.
        """

        print("Waiting for Sim...")
        
        # Give shell script time to actually spawn the drones
        print("Giving shell script 10 seconds to spawn drones...")
        time.sleep(10)  # Wait for script to spawn drones before checking
        
        # Additional delay for parameter bridges to establish all topic connections
        print("Giving parameter bridges 15 more seconds to establish all topic connections...")
        time.sleep(15)  # Wait for parameter bridges to create all camera/sensor topics
        
        start_time = time.time()
        max_wait = 60  # 60s timeout; adjust based on spawn time (Gazebo can take 30s+ for 10 drones)
        
        try:
            while self.app.simReady is not True:
                if time.time() - start_time > max_wait:
                    print(f"Timeout waiting for {self.requestCount} drones. Falling back to simulation mode.")
                    # Fallback: Generate fake data like simulateEnvSetup
                    for _ in range(self.requestCount):
                        self.app.drones.append(self.app.generate_drone())
                        if _ <= len(self.app.default_waypoints)-1:
                            self.app.drones[-1]['waypoints'] = self.app.default_waypoints[_]
                    for _ in range(random.randint(3, 10)):
                        self.app.incidents.append(self.app.generate_random_incident())
                    self.app.simReady = True
                    break
                
                # Re-discover topics to catch new ones (bug fix)
                self.app.ros_handler.discover_topics()  # Refresh topics *before* checking
                
                drone_amount = self.app.ros_handler.get_drone_id_topics_amount()
                print(f"Detected {drone_amount}/{self.requestCount} drones...")  # Debug print every ~0.5s
                
                if drone_amount == self.requestCount:
                    print("Found ROS, starting to subscribe...")

                    for i in range(1, drone_amount + 1):
                        ns = f'rs1_drone_{i}'
                        self.app.drones.append({
                            'id': i,
                            'ns': ns,
                            'battery': '-',                 # unknown yet
                            'gps': '0.000, 0.000',          # will be filled from odom
                            'altitude': '-',                # will be filled from odom
                            'state': 'IDLE',                # DroneController prints IDLE on startup
                            'setPose': '-',
                            'nearPose': '-',
                            'yaw': 0.0,
                            'waypoints': []
                        })
                        
                        if (i - 1) < len(self.app.default_waypoints):
                            import copy
                            self.app.drones[-1]['waypoints'] = copy.deepcopy(self.app.default_waypoints[i - 1])

                    deadline = time.time() + 20  # wait up to 20s for cameras to appear
                    cams = []
                    expected_cameras = drone_amount * 2  # Each drone should have front + bottom cameras
                    while time.time() < deadline:
                        try:
                            self.app.ros_handler.discover_topics()     # refresh
                            cams = self.app.ros_handler.get_available_camera_topics()
                            
                            # Check if we have cameras from all drones, not just any cameras
                            drone_cameras_found = set()
                            for cam_topic in cams:
                                # Extract drone ID from topic like "/rs1_drone_1/front/image"
                                if '/rs1_drone_' in cam_topic:
                                    drone_id = cam_topic.split('/rs1_drone_')[1].split('/')[0]
                                    drone_cameras_found.add(drone_id)
                            
                            print(f"Camera discovery: found {len(cams)} topics from {len(drone_cameras_found)} drones (expecting {drone_amount} drones)")
                            
                            # Break if we have cameras from all drones OR reasonable timeout
                            if len(drone_cameras_found) >= drone_amount or (len(cams) > 0 and time.time() - deadline + 20 > 15):
                                break
                                
                        except Exception as e:
                            print(f"Camera discovery error: {e}")
                        time.sleep(0.5)

                    print(f"UI sees {len(cams)} camera topics: {cams}")
                    
                    # Init camera BEFORE setting simReady to avoid race
                    self.app.camera_component = CameraComponent( # self.app.CameraComponent
                        self.app.ros_handler, 
                        display_rect=(1240, 460, 640, 360),
                        instant_switching=True,
                        preload_all=True
                    )
                    
                    self.app.odometry_topics = self.app.ros_handler.get_available_odometry_topics()
                    print(f"UI sees {len(self.app.odometry_topics)} odomometry topics: {self.app.odometry_topics}")
                    for t in self.app.odometry_topics:
                        self.app.ros_handler.subscribe_to_odometry_topic(t)

                    self.app.incident_topics = self.app.ros_handler.get_available_incident_topics()
                    print(f"UI sees {len(self.app.incident_topics)} incident topics: {self.app.incident_topics}")
                    for t in self.app.incident_topics:
                        self.app.ros_handler.subscribe_to_incident_topic(t)

                    self.app.button_topics = self.app.ros_handler.get_available_button_topics()
                    print(f"UI sees {len(self.app.button_topics)} button topics: {self.app.button_topics}")
                    for t in self.app.button_topics:
                        self.app.ros_handler.subscribe_to_button_topic(t)



                    # init camera (if you want even with 0 topics it should be safe)
                    self.app.camera_component = CameraComponent(
                        self.app.ros_handler,
                        display_rect=(1240, 460, 640, 360),
                        instant_switching=True,
                        preload_all=True
                    )
                    self.app.simReady = True
                    break
                
                time.sleep(0.5)  # Allow ROS/DDS time to discover; poll every 500ms (balance responsiveness/efficiency)
        except Exception as e:
            print(f"Error in waitForSim thread: {e}")
            # Fallback on error too
            for _ in range(self.requestCount):
                self.app.drones.append(self.app.generate_drone())
                if _ <= len(self.app.default_waypoints)-1:
                    self.app.drones[-1]['waypoints'] = self.app.default_waypoints[_]
            for _ in range(random.randint(3, 10)):
                self.app.incidents.append(self.app.generate_random_incident())
            self.app.simReady = True
        
        print("Sim ready!")

    def buttonLogic(self, gmx, gmy):
        if self.runningScript is not True:
            for rect, action in self.button_rects:
                if rect.collidepoint(gmx, gmy):
                    if action == "submit":
                        if self.user_input != "":
                            self.startSim()
    

