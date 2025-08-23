import pygame
import cv2
import numpy as np


class VideoPlayer:
    def __init__(self, video_path):
        self.video_path = video_path
        self.cap = cv2.VideoCapture(video_path)
        self.last_good_frame_surface = None
        
        # Get video properties
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_delay = int(1000 / self.fps) if self.fps > 0 else 33
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    def video_loop(self, screen):
        """Original video_loop function"""
        ret, frame = self.cap.read()

        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            # Don't read again immediately — we'll just reuse last frame
        else:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_surface = pygame.surfarray.make_surface(np.flipud(np.rot90(frame_rgb)))
            self.last_good_frame_surface = frame_surface

        # Always blit the last good frame if we have one
        if self.last_good_frame_surface is not None:
            # screen.blit(feather_image(self.last_good_frame_surface, 150, 150, feather_top=False, feather_right=True, feather_bottom=True), (20, 20))
            screen.blit(self.last_good_frame_surface, (20, 20))
    
    def cleanup(self):
        """Release video resources"""
        if self.cap:
            self.cap.release()