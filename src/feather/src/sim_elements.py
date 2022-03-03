import os
import pygame
from numpy import cos, sin

class LaserState:
    HOLD = 'hold'
    FIRE = 'fire'
    BRUSH = 'brush'

class Obstacle:
    def __init__(self, x, y, width, height, pic):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.angle = -90
        BASE_DIR = os.path.dirname(os.path.realpath(__file__))
        self.logo = pygame.image.load(F'{BASE_DIR}/{pic}')

    def get_hitbox(self):
        return [
            self.x,
            self.y,
            self.x + self.width,
            self.y + self.height,
            ]

class Player:
    def __init__(self, x, y, pic):
        self.x = x
        self.y = y
        self.angle = -90
        self.angle_speed = 0
        self.speed_x = 0
        self.speed_y = 0
        self.speed = 0
        BASE_DIR = os.path.dirname(os.path.realpath(__file__))
        self.logo = pygame.image.load(F'{BASE_DIR}/{pic}')

    def update_params(self, params):
        self.params = params

    def move_forward(self):
        self.speed = -self.params['speed']
    def move_backward(self):
        self.speed = self.params['speed']
    def turn_left(self):
        self.angle_speed = -self.params['angle_speed']
    def turn_right(self):
        self.angle_speed = self.params['angle_speed']
    def stop_move(self):
        self.speed = 0
    def stop_rotating(self):
        self.angle_speed = 0
    def dynamics(self):
        self.angle += self.angle_speed
        speed_x, speed_y = rect_speed(self.speed, self.angle)
        self.x += speed_x
        self.y += speed_y
    def keystroke_movements(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_k:
                #up
                self.move_forward()
            if event.key == pygame.K_j:
                #down
                self.move_backward()
            if event.key == pygame.K_h:
                #left
                self.turn_left()
            if event.key == pygame.K_l:
                #right
                self.turn_right()

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_k:
                #up
                self.stop_move()
            if event.key == pygame.K_j:
                #down
                self.stop_move()
            if event.key == pygame.K_h:
                #left
                self.stop_rotating()
            if event.key == pygame.K_l:
                #right
                self.stop_rotating()

class Laser:
    def __init__(self, pic):
        BASE_DIR = os.path.dirname(os.path.realpath(__file__))
        self.logo = pygame.image.load(F'{BASE_DIR}/{pic}')
        self.speed = 0
        self.x = 0
        self.y = 0
        self.angle = 0
        self.params = {'bullet_speed': 1}
        self.state = LaserState.HOLD

    def shot(self):
        print('Shooting')
        self.speed = self.params['bullet_speed']

def rect_speed(linear, angle):
    angle_ = angle * 3.1416 / 180
    #print(F'speed = {linear}, angle = {angle}')
    if angle > 180:
        speed_x = -linear * cos(angle_)
        speed_y = linear * sin(angle_)
    else:
        speed_x = linear * cos(angle_)
        speed_y = -linear * sin(angle_)
    return speed_x, speed_y
