import os
import pygame
import time
from sim_elements import *

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
        self.gallito = Player(300, 300, 'nabo.png')
        self.gallito.update_params(self.params)
        # <a href="https://www.flaticon.com/free-icons/rooster" title="rooster icons">Rooster icons created by Culmbio - Flaticon</a>

        # obstacles
        self.obstacles = []
        student_obstacle1 = Obstacle(
            150,
            150,
            100,
            100,
            'examen.png'
            )
        student_obstacle2 = Obstacle(
            550,
            550,
            100,
            100,
            'examen.png'
            )


        self.obstacles.append(student_obstacle1)
        self.obstacles.append(student_obstacle2)
        self.get_walls()


        #<a href="https://www.flaticon.es/iconos-gratis/examen" title="examen iconos">Examen iconos creados por BomSymbols - Flaticon</a>

        #bullet
        self.laser = Laser('bullet.png')
        #<a href="https://www.flaticon.com/free-icons/bullet" title="bullet icons">Bullet icons created by Freepik - Flaticon</a>


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
                if event.key == pygame.K_SPACE:
                    self.laser.x = self.gallito.x
                    self.laser.y = self.gallito.y
                    self.laser.angle = self.gallito.angle
                    self.laser.state = LaserState.BRUSH
                    self.laser.shot()


            self.gallito.keystroke_movements(event)

        #screen is drawn first and then we draw our player  caracter
        #player dinamics
        self.gallito.dynamics()

        #bullet dinamics
        i = 0
        while self.laser.state == LaserState.BRUSH:
                
            speed_x, speed_y = rect_speed(
                self.laser.speed,
                self.laser.angle + i
                )
            self.laser.x -= speed_x
            self.laser.y -= speed_y
            print(F'bullet {i}')

            collision = self.collision(
                self.laser.x,
                self.laser.y,
                self.obstacles
                )
            if collision:
                self.laser.x = self.gallito.x
                self.laser.y = self.gallito.y
                self.screen.fill((230, 0, 0))
                i += 1
                #publish
                    #time.sleep(1)
            #self.draw_element(self.laser)
            #pygame.display.update()
            if i >= 360:
                self.laser.speed = 0
                self.laser.state = LaserState.HOLD
                

        # boundaries
        '''
        if self.player_x + self.frame[0] > self.screen_size[0]:
            self.player_x = self.screen_size[0] - self.frame[0]
        if self.player_y + self.frame[1] > self.screen_size[1]:
            self.player_y = self.screen_size[1] - self.frame[1]
        if self.player_x - self.frame[0] < 0:
            self.player_x = self.frame[0]
        if self.player_y - self.frame[1] < 0:
            self.player_y = self.frame[1]
        '''

        self.draw_element(self.gallito)
        self.draw_element(self.laser)
        for obstacle in self.obstacles:
            self.draw_element(
                    obstacle,
                    mode = "obstacle",
                    list_ = "hole",
                    )
        #self.enemy(self.student_obstacle1)
        #self.scan(
        #    self.laser.x,
        #    self.laser.y,
        #    self.laser.angle
        #    )

        pygame.display.update()

    def collision(self, x, y, obstacles):
        for obstacle in obstacles:
            coordinates = obstacle.get_hitbox()
            if x > coordinates[0] and x < coordinates[2] and y > coordinates[1] and y < coordinates[3]:
                print(F'COLLISION DETECTED  AT\n({x}, {y}')
                return True
        return False


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
        left_wall = Obstacle(
            0,
            0,
            4,
            self.screen_size[1],
            'examen.png'
            )
        right_wall = Obstacle(
            self.screen_size[0],
            0,
            10,
            self.screen_size[1],
            'rooster.png'
            )
        self.obstacles.append(floor)
        self.obstacles.append(ceiling)
        self.obstacles.append(left_wall)
        self.obstacles.append(right_wall)

    def enemy(self, obstacle):
        self.screen.blit(
            obstacle.logo,
            (obstacle.x, obstacle.y)
            )#drawing

    def scan(self, x, y, angle):
        logo_rotated = pygame.transform.rotate(
            self.laser.logo,
            angle
            )
        self.screen.blit(logo_rotated, (x + 10 , y + 10))#drawing

    def draw_element(self, element, **kwargs):
        try:
            if kwargs['mode'] == "obstacle":
                # Initialing Color
                color = (0,0,0)
                # Drawing Rectangle
                pygame.draw.rect(
                        self.screen,
                        color,
                        pygame.Rect(
                            element.x,
                            element.y,
                            element.width,
                            element.height
                            )
                        )
        except:
            pass

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
            'speed': 0.2,
            'angle_speed': 0.2,
            'bullet_speed': 2,
            }
    )
    while sim.running:
        sim.gameloop()
