#!
import pygame
import time

# Init Pygame
pygame.init()

screen = pygame.display.set_mode((650,650))
running = True

# title and icon
pygame.display.set_caption("FeatherBot Sim")
# <a href="https://www.flaticon.com/free-icons/cock" title="cock icons">Cock icons created by Freepik - Flaticon</a>
icon = pygame.image.load('rooster.png')
pygame.display.set_icon(icon)

# player
player_logo = pygame.image.load("nabo.png")
# <a href="https://www.flaticon.com/free-icons/rooster" title="rooster icons">Rooster icons created by Culmbio - Flaticon</a>
player_x = 300
player_y = 300
speed = 2

def player(x, y):
    screen.blit(player_logo, (x, y))#drawing

# Game loop
while running:
    screen.fill((230, 255, 255))
    for event in pygame.event.get():
        #print(F"event = {event.type}")
        if event.type == pygame.QUIT:
            running == False

    time.sleep(0.01)
    #screen is drawn first and then we draw our player  caracter
    player_x += speed
    player_y += speed
    player(player_x, player_y)

    pygame.display.update()


