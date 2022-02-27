import os
import pygame
from numpy import cos, sin

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
#class Bullet:

def rect_speed(linear, angle):
    angle_ = angle * 3.1416 / 180
    if angle > 180:
        speed_x = -linear * cos(angle_)
        speed_y = linear * sin(angle_)
    else:
        speed_x = linear * cos(angle_)
        speed_y = -linear * sin(angle_)
    return speed_x, speed_y