import os
import pygame
import time
from numpy import cos, sin

class CordobesSimulator:
    def __init__(self, **kwargs):
        self.screen_size = kwargs['screen_size']
        self.params = kwargs['params']

        pygame.init()

        self.frame = [30, 30]
        self.screen = pygame.display.set_mode(
                (self.screen_size[0],
                self.screen_size[1])
                )
        self.running = True

        # title and icon
        pygame.display.set_caption("FeatherBot Sim")
        # <a href="htts://www.flaticon.com/free-icons/cock" title="cock icons">Cock icons created by Freepik - Flaticon</a>
        BASE_DIR = os.path.dirname(os.path.realpath(__file__))
        icon = pygame.image.load(F'{BASE_DIR}/rooster.png')
        pygame.display.set_icon(icon)

        # player
        self.player_logo = pygame.image.load(F'{BASE_DIR}/nabo.png')
        # <a href="https://www.flaticon.com/free-icons/rooster" title="rooster icons">Rooster icons created by Culmbio - Flaticon</a>
        self.player_x = 300
        self.player_y = 300
        self.speed = 0
        self.angle = -90
        self.angle_speed = 0
        self.speed_x = 0
        self.speed_y = 0

        # enemy
        self.enemy_logo = pygame.image.load(F'{BASE_DIR}/examen.png')
        #<a href="https://www.flaticon.es/iconos-gratis/examen" title="examen iconos">Examen iconos creados por BomSymbols - Flaticon</a>


    def gameloop(self):
    # Game loop
    #while running and not rospy.is_shutdown():
        self.screen.fill((230, 255, 255))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            #if keystroke is pressed
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.running = False
                if event.key == pygame.K_k:
                    #up
                    self.speed = -self.params['speed']
                if event.key == pygame.K_j:
                    #down
                    self.speed = self.params['speed']
                if event.key == pygame.K_h:
                    #left
                    self.angle_speed = self.params['angle_speed']
                if event.key == pygame.K_l:
                    #right
                    self.angle_speed = -self.params['angle_speed']

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_k:
                    #up
                    self.speed = 0
                if event.key == pygame.K_j:
                    #down
                    self.speed = 0
                if event.key == pygame.K_h:
                    #left
                    self.angle_speed = 0
                if event.key == pygame.K_l:
                    #right
                    self.angle_speed = 0

        #screen is drawn first and then we draw our player  caracter
        self.angle += self.angle_speed
        speed_x, speed_y = rect_speed(self.speed, self.angle)
        self.player_x += speed_x
        self.player_y += speed_y
        # boundaries
        if self.player_x + self.frame[0] > self.screen_size[0]:
            self.player_x = self.screen_size[0] - self.frame[0]
        if self.player_y + self.frame[1] > self.screen_size[1]:
            self.player_y = self.screen_size[1] - self.frame[1]
        if self.player_x - self.frame[0] < 0:
            self.player_x = self.frame[0]
        if self.player_y - self.frame[1] < 0:
            self.player_y = self.frame[1]

        self.player(self.player_x, self.player_y, self.angle)
        self.enemy(150, 150)
        pygame.display.update()


    def enemy(self, x, y):
        self.screen.blit(self.enemy_logo, (x, y))#drawing

    def player(self, x, y, angle):
        logo_rotated = pygame.transform.rotate(self.player_logo, angle + 90)
        self.screen.blit(logo_rotated, (x, y))#drawing

def rect_speed(linear, angle):
    angle_ = angle * 3.1416 / 180
    if angle > 180:
        speed_x = -linear * cos(angle_)
        speed_y = linear * sin(angle_)
    else:
        speed_x = linear * cos(angle_)
        speed_y = -linear * sin(angle_)
    return speed_x, speed_y

if __name__ == "__main__":
    sim = CordobesSimulator(
        screen_size = [650, 650],
        params = {
            'speed': 0.6,
            'angle_speed': 0.1
            }
    )
    while sim.running:
        sim.gameloop()