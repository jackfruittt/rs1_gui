import pygame, math, time, subprocess, threading, random
from constants import *

class SpawnPromptPanel:
    def __init__(self, app, fonts, rosAvailable):
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
        """Draw the full-screen spawn prompt interface"""
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
        question = self.fonts['inter_small'].render("How many drones would you like to spawn? (1-10)", True, WHITE)
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

    def handle_key(self, event):
        """Restrict input to digits only"""
        if self.runningScript is not True:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_BACKSPACE:
                    self.user_input = self.user_input[:-1]
                elif event.key == pygame.K_RETURN:
                    if self.user_input != "":
                        self.runningScript = True
                        self.requestCount = int(self.user_input)
                        if self.rosAvailable is True:
                            self.runScript(self.user_input)
                        else:
                            self.simulateThread.start()
                    return
                elif event.unicode.isdigit():
                    if int(self.user_input + event.unicode) < 11:
                        self.user_input += event.unicode
        return None
    
    def runScript(self, numDrones):
        """Run the script to start the simulator and spawn drones"""
        count = int(numDrones)
        if self.rosAvailable is True:
            subprocess.Popen(
                ["./launch_multi_drone_composition.sh", str(count), "gazebo:=true", "rviz:=true"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        self.waitSimThread.start()
    
    def killSim(self):
        """Kill the simulator processes"""
        commands = ['pkill -f "ign gazebo"', 'pkill -f "parameter_bridge"', 'pkill -f "robot_state_publisher"', 'pkill -f "multi_drone_composition_controller"', 'pkill -f "controller_node"']
        for command in commands:
            subprocess.Popen(command, shell=True)
    
    def simulateEnvSetup(self):
        time.sleep(5)
        for _ in range(self.requestCount):
            self.app.drones.append(self.app.generate_drone())

        for _ in range(random.randint(3,10)):
            self.app.incidents.append(self.app.generate_random_incident())
        self.app.simReady = True
    
    def waitForSim(self):
        while self.app.simReady is not True:
            # check topics being published and when it's determined the sim is setup and ready, then set simReady to True
            # to connect to the ros handler, it'll be `self.app.ros_handler`
            return

    def get_button_rects(self):
        return self.button_rects
