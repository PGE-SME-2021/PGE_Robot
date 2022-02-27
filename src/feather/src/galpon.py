import os
import pygame
import time
from sim_elements import Obstacle, Player, rect_speed

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
        self.player_x = 300
        self.player_y = 300
        self.speed = 0
        self.angle = -90
        self.angle_speed = 0
        self.speed_x = 0
        self.speed_y = 0

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
        self.bullet_logo = pygame.image.load(F'{BASE_DIR}/bullet.png')
        self.bullet_speed = 0
        self.bullet_x = 300
        self.bullet_y = 300
        self.bullet_angle = self.angle
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
                if event.key == pygame.K_SPACE:
                    self.bullet_x = self.gallito.x
                    self.bullet_y = self.gallito.y
                    self.bullet_angle = self.gallito.angle
                    self.bullet_speed = self.params['bullet_speed']

                if event.key == pygame.K_q:
                    self.running = False
                if event.key == pygame.K_k:
                    #up
                    self.gallito.move_forward()
                if event.key == pygame.K_j:
                    #down
                    self.gallito.move_backward()
                if event.key == pygame.K_h:
                    #left
                    self.gallito.turn_left()
                if event.key == pygame.K_l:
                    #right
                    self.gallito.turn_right()

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_k:
                    #up
                    self.gallito.stop_move()
                if event.key == pygame.K_j:
                    #down
                    self.gallito.stop_move()
                if event.key == pygame.K_h:
                    #left
                    self.gallito.stop_rotating()
                if event.key == pygame.K_l:
                    #right
                    self.gallito.stop_rotating()

        #screen is drawn first and then we draw our player  caracter
        #player dinamics
        self.gallito.dynamics()

        #bullet dinamics
        speed_x, speed_y = rect_speed(
            self.bullet_speed,
            self.bullet_angle
            )
        self.bullet_x -= speed_x
        self.bullet_y -= speed_y
        if self.bullet_y == 0:
            self.bullet_speed = 0
            self.bullet_y = self.player_y

        collision = self.collision(self.bullet_x, self.bullet_y, self.obstacles)
        if collision:
            self.bullet_speed = 0
            self.bullet_x = self.player_x
            self.bullet_y = self.player_y

        # boundaries
        if self.player_x + self.frame[0] > self.screen_size[0]:
            self.player_x = self.screen_size[0] - self.frame[0]
        if self.player_y + self.frame[1] > self.screen_size[1]:
            self.player_y = self.screen_size[1] - self.frame[1]
        if self.player_x - self.frame[0] < 0:
            self.player_x = self.frame[0]
        if self.player_y - self.frame[1] < 0:
            self.player_y = self.frame[1]

        self.draw_element(self.gallito)
        for obstacle in self.obstacles:
            self.draw_element(obstacle)
        #self.enemy(self.student_obstacle1)
        self.bullet(
            self.bullet_x,
            self.bullet_y,
            self.bullet_angle
            )
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
            1,
            'examen.png'
            )
        ceiling = Obstacle(
            0,
            0,
            self.screen_size[0],
            1,
            'examen.png'
            )
        left_wall = Obstacle(
            0,
            0,
            1,
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

    def bullet(self, x, y, angle):
        logo_rotated = pygame.transform.rotate(
            self.bullet_logo,
            angle
            )
        self.screen.blit(logo_rotated, (x + 10 , y + 10))#drawing
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