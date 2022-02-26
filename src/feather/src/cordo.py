#!/usr/bin/env python3
#ros dependencies
import rospy
from feather.msg import Status, LidarData
from sensor_msgs.msg import PointCloud, Image
from feather.msg import Status
from feather.srv import SendCommand

#sim dependencies
import os
import pygame
import time
from numpy import cos, sin

def enemy(x, y):
    screen.blit(enemy_logo, (x, y))#drawing

def player(x, y, angle):
    logo_rotated = pygame.transform.rotate(player_logo, angle + 90)
    screen.blit(logo_rotated, (x, y))#drawing

def rect_speed(linear, angle):
    angle_ = angle * 3.1416 / 180
    if angle > 180:
        speed_x = -linear * cos(angle_)
        speed_y = linear * sin(angle_)
    else:
        speed_x = linear * cos(angle_)
        speed_y = -linear * sin(angle_)
    return speed_x, speed_y


def handle_move(req):
    print(req)
    global speed, angle_speed, params
    if req.command == 1:
        speed = -params['speed']
    elif req.command == 2:
        speed = params['speed']
    elif req.command == 3:
        angle_speed = params['angle_speed']
    elif req.command == 4:
        angle_speed = -params['angle_speed']
    elif req.command == 5:
        speed = 0
        angle_speed =0

    return 12

if __name__ == "__main__":
    #init rosnode
    global speed, angle_speed, params
    rospy.init_node('feather_sim')
    pub = rospy.Publisher("status", Status, queue_size = 10)
    rate = rospy.Rate(1)#1Hz
    move_server = rospy.Service('send_command', SendCommand, handle_move)
    counter = 0
    msg_to_publish = Status()
    BASE_DIR = os.path.dirname(os.path.realpath(__file__))
    # Init Pygame
    pygame.init()

    screen_size = [650, 650]
    frame = [30, 30]
    screen = pygame.display.set_mode(
            (screen_size[0],screen_size[1])
            )
    running = True

    # title and icon
    pygame.display.set_caption("FeatherBot Sim")
    # <a href="https://www.flaticon.com/free-icons/cock" title="cock icons">Cock icons created by Freepik - Flaticon</a>
    icon = pygame.image.load(F'{BASE_DIR}/rooster.png')
    pygame.display.set_icon(icon)

    # player
    player_logo = pygame.image.load(F'{BASE_DIR}/nabo.png')
    # <a href="https://www.flaticon.com/free-icons/rooster" title="rooster icons">Rooster icons created by Culmbio - Flaticon</a>
    player_x = 300
    player_y = 300
    speed = 0
    angle = -90
    angle_speed = 0
    params = {
            'speed': 0.6,
            'angle_speed': 0.1,
            }
    speed_x = 0
    speed_y = 0

    # enemy
    enemy_logo = pygame.image.load(F'{BASE_DIR}/examen.png')
    #<a href="https://www.flaticon.es/iconos-gratis/examen" title="examen iconos">Examen iconos creados por BomSymbols - Flaticon</a>

    # Game loop
    while running and not rospy.is_shutdown():
        screen.fill((230, 255, 255))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            #if keystroke is pressed
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                if event.key == pygame.K_k:
                    #up
                    speed = -params['speed']
                if event.key == pygame.K_j:
                    #down
                    speed = params['speed']
                if event.key == pygame.K_h:
                    #left
                    angle_speed = params['angle_speed']
                if event.key == pygame.K_l:
                    #right
                    angle_speed = -params['angle_speed']

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_k:
                    #up
                    speed = 0
                if event.key == pygame.K_j:
                    #down
                    speed = 0
                if event.key == pygame.K_h:
                    #left
                    angle_speed = 0
                if event.key == pygame.K_l:
                    #right
                    angle_speed = 0

        #screen is drawn first and then we draw our player  caracter
        angle += angle_speed
        speed_x, speed_y = rect_speed(speed, angle)
        player_x += speed_x
        player_y += speed_y
        # boundaries
        if player_x + frame[0] > screen_size[0]:
            player_x = screen_size[0] - frame[0]
        if player_y + frame[1] > screen_size[1]:
            player_y = screen_size[1] - frame[1]
        if player_x - frame[0] < 0:
            player_x = frame[0]
        if player_y - frame[1] < 0:
            player_y = frame[1]

        player(player_x, player_y, angle)
        enemy(150, 150)
        #print(F"X = {player_x}, Y = {player_y}, angle = {angle}")
        pygame.display.update()
        msg_to_publish.speed = speed
        msg_to_publish.angular_speed = angle_speed
        msg_to_publish.battery = 75
        msg_to_publish.acceleration = 0

        pub.publish(msg_to_publish)
