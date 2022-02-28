import os
import pygame
import time
from sim_elements import *
from mainwindow_gui import mapping

class CordobesSimulator:
    def __init__(self, **kargs):
        self.screen_size = kargs['screen_size']
        self.params = kargs['params']

        pygame.init()

        self.frame = [30, 30]
        self.screen = pygame.display.set_mode(
                (self.screen_size[0],
                self.screen_size[1])
                )
        self.running = True

        # title and icon
        pygame.display.set_caption("FeatherBot Sim")
        # <a href="htts://ww.flaticon.com/free-icons/cock" title="cock icons">Cock icons created by Freepik - Flaticon</a>
        BASE_DIR = os.path.dirname(os.path.realpath(__file__))
        icon = pygame.image.load(F'{BASE_DIR}/rooster.png')
        pygame.display.set_icon(icon)

        # player
        self.gallito = Player(300, 300, 'nabo.png')
        self.gallito.update_params(self.params)
        # <a href="https://ww.flaticon.com/free-icons/rooster" title="rooster icons">Rooster icons created by Culmbio - Flaticon</a>

        # obstacles
        self.obstacles = []
        student_obstacle1 = Obstacle(
            250,
            150,
            100,
            100,
            'examen.png'
            )
        student_obstacle2 = Obstacle(
            450,
            450,
            100,
            100,
            'examen.png'
            )


        self.obstacles.append(student_obstacle1)
        self.obstacles.append(student_obstacle2)
        self.get_walls()


        #<a href="https://ww.flaticon.es/iconos-gratis/examen" title="examen iconos">Examen iconos creados por BomSymbols - Flaticon</a>

        #bullet
        self.laser = Laser('bullet.png')
        #<a href="https://ww.flaticon.com/free-icons/bullet" title="bullet icons">Bullet icons created by Freepik - Flaticon</a>


    def gameloop(self):
    # Game loop
        self.screen.fill((230, 255, 255))
        self.lidar_points = []
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            #if keystroke is pressed
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.running = False
                if event.key == pygame.K_SPACE:
                    self.laser.x = self.gallito.x
                    self.laser.y = self.gallito.y
                    self.laser.angle = self.gallito.angle
                    self.laser.state = LaserState.BRUSH
                    self.laser.shot()


            self.gallito.keystroke_movements(event)

        #screen is dran first and then we draw our player  caracter
        #player dinamics
        self.gallito.dynamics()

        #get lidar points
        self.get_lidar_data()

        # boundary collisions
        self.boundary_collisions()

        self.draw_element(self.gallito)
        for obstacle in self.obstacles:
            self.draw_element(obstacle)

        pygame.display.update()

    def boundary_collisions(self):
        if self.gallito.x + self.frame[0] > self.screen_size[0]:
            self.gallito.x = self.screen_size[0] - self.frame[0]
        if self.gallito.y + self.frame[1] > self.screen_size[1]:
            self.gallito.y = self.screen_size[1] - self.frame[1]
        if self.gallito.x - self.frame[0] < 0:
            self.gallito.x = self.frame[0]
        if self.gallito.y - self.frame[1] < 0:
            self.gallito.y = self.frame[1]

    def get_lidar_data(self):
        i = 0
        while self.laser.state == LaserState.BRUSH:
            speed_x, speed_y = rect_speed(
                self.laser.speed,
                self.laser.angle + i
                )
            self.laser.x -= speed_x
            self.laser.y -= speed_y

            is_collision, lidar_x, lidar_y  = self.collision(
                self.laser.x,
                self.laser.y,
                self.obstacles
                )
            if is_collision:
                #relative coords
                lidar_x = ((self.gallito.x + 24) + lidar_x) * -1
                lidar_y = (lidar_y - self.gallito.y - 24) * -1
                #adjust to a -12 and 12 frame
                frame_size = 10
                #lidar_x = mapping(self.screen_size[0], 0, -frame_size, frame_size, lidar_x)
                #lidar_y = mapping(self.screen_size[1], 0, -frame_size, frame_size, lidar_y)
                self.lidar_points.append([lidar_x, lidar_y])
                i += 1
                #reset laser
                self.laser.x = self.gallito.x
                self.laser.y = self.gallito.y
                self.screen.fill((230, 0, 0))
            if i >= 360:
                self.laser.speed = 0
                self.laser.state = LaserState.HOLD

    def collision(self, x, y, obstacles):
        for obstacle in obstacles:
            coordinates = obstacle.get_hitbox()
            if x > coordinates[0] and x < coordinates[2] and y > coordinates[1] and y < coordinates[3]:
                #print(F'COLLISION DETECTED  AT\n({x}, {y}')
                return True, x, y
        return False, 0, 0

    def get_walls(self):
        floor = Obstacle(
            0,
            self.screen_size[0],
            self.screen_size[0],
            4,
            'examen.png'
            )
        ceiling = Obstacle(
            0,
            0,
            self.screen_size[0],
            4,
            'examen.png'
            )
        left_all = Obstacle(
            0,
            0,
            4,
            self.screen_size[1],
            'examen.png'
            )
        right_all = Obstacle(
            self.screen_size[0],
            0,
            10,
            self.screen_size[1],
            'rooster.png'
            )
        self.obstacles.append(floor)
        self.obstacles.append(ceiling)
        self.obstacles.append(left_all)
        self.obstacles.append(right_all)

    def enemy(self, obstacle):
        self.screen.blit(
            obstacle.logo,
            (obstacle.x, obstacle.y)
            )#draing

    def draw_element(self, element):
        logo_rotated = pygame.transform.rotate(
            element.logo, element.angle + 90
            )
        self.screen.blit(
            logo_rotated,
            (element.x, element.y)
            )#drawing

if __name__ == "__main__":
    sim = CordobesSimulator(
        screen_size = [650, 650],
        params = {
            'speed': 0.6,
            'angle_speed': 0.1,
            'bullet_speed': 1,
            }
    )
    while sim.running:
        sim.gameloop()
