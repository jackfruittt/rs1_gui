import pygame
from constants import *


class Popup:
    def __init__(self, fonts):
        self.fonts = fonts
        self.popup_visible = False
        self.selected_incident = None
        self.popup_scale = 0
        self.animating_in = False
        self.animating_out = False
        self.anim_start_time = 0
    
    def draw_popup(self, screen):
        """Original draw_popup function"""
        scale = self.popup_scale / 100
        if scale <= 0:
            return

        popup_width = int(600 * scale)
        popup_height = int(400 * scale)
        popup_x = 340 + (600 - popup_width) // 2
        popup_y = 160 + (400 - popup_height) // 2

        popup_surface = pygame.Surface((popup_width, popup_height))
        popup_surface.fill(WHITE)

        if self.selected_incident:
            title = self.fonts['font'].render(self.selected_incident["title"], True, BLACK)
            time_text = self.fonts['small_font'].render(self.selected_incident["time"], True, BLACK)
            popup_surface.blit(title, (20, 20))
            popup_surface.blit(time_text, (20, 60))

            # Close button
            pygame.draw.rect(popup_surface, GRAY, (popup_width - 110, popup_height - 50, 100, 40))
            close_text = self.fonts['small_font'].render("Close", True, BLACK)
            popup_surface.blit(close_text, (popup_width - 85, popup_height - 40))

            # Image placeholder
            img_w, img_h = int(popup_width * 0.6), int(popup_height * 0.4)
            img_x = (popup_width - img_w) // 2
            img_y = (popup_height - img_h) // 2
            
            pygame.draw.rect(popup_surface, LIGHT_GRAY, (img_x, img_y, img_w, img_h))

        screen.blit(popup_surface, (popup_x, popup_y))
    
    def handle_popup_animation(self):
        """Original handle_popup_animation function"""
        elapsed = pygame.time.get_ticks() - self.anim_start_time
        progress = min(1.0, elapsed / ANIM_DURATION)
        
        if self.animating_in:
            self.popup_scale = int(100 * progress)
            if progress >= 1:
                self.animating_in = False
                self.popup_visible = True
        elif self.animating_out:
            self.popup_scale = int(100 * (1 - progress))
            if progress >= 1:
                self.animating_out = False
                self.popup_visible = False
                self.popup_scale = 0
    
    def show_popup(self, incident):
        """Show popup for an incident"""
        self.selected_incident = incident
        self.animating_in = True
        self.anim_start_time = pygame.time.get_ticks()
    
    def hide_popup(self):
        """Hide the popup"""
        self.animating_out = True
        self.anim_start_time = pygame.time.get_ticks()
    
    def handle_close_click(self, pos):
        """Handle close button click"""
        if not self.popup_visible:
            return False
        
        popup_width = int(600 * self.popup_scale / 100)
        popup_height = int(400 * self.popup_scale / 100)
        popup_x = 340 + (600 - popup_width) // 2
        popup_y = 160 + (400 - popup_height) // 2
        
        rel_x = pos[0] - popup_x
        rel_y = pos[1] - popup_y
        
        if popup_width - 110 < rel_x < popup_width - 10 and popup_height - 50 < rel_y < popup_height - 10:
            self.hide_popup()
            return True
        return False