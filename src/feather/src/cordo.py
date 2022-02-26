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
player_logo = pygame.image.load("cordobes.png")
playerX = 300
playerY = 300
speed = 2

def player():
    screen.blit(player_logo, (playerX, playerY))#drawing

# Game loop
while running:
    screen.fill((230, 255, 255))
    for event in pygame.event.get():
        #print(F"event = {event.type}")
        if event.type == pygame.QUIT:
            running == False

    time.sleep(0.01)
    #screen is drawn first and then we draw our player  caracter
    playerX -= speed
    player()

    pygame.display.update()


