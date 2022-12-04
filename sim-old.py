import pygame
from pygame.locals import *
import sys
import random
import numpy as np
from math import degrees, radians

from cart2 import x, t, cartpole

pygame.init()
HEIGHT = 400
WIDTH = 600
ACC = 9.8 * 20
FPS = 120  # adjust N in cart1
CART_POS = [100, 360]
CART_HEIGHT = [0, -30]
frame = 0
robot = False
# scale is about 20 pixels per meter, adjust sl in cart1 corresponding to length

Clock = pygame.time.Clock()

displaysurface = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Game")

cart_img = pygame.image.load("transport-industrial-truck.png")
pole_img = pygame.image.load("pole.png")


class Pole(pygame.sprite.Sprite):
    def __init__(self, picture, length):
        super().__init__()
        # state = posx, posy, angle, vel, acc, length
        self.state = np.array(
            [
                CART_POS[0] + CART_HEIGHT[0],
                CART_POS[1] + CART_HEIGHT[1],
                30,
                0,
                0,
                2 * length,
            ]
        )

        self.image = pygame.transform.scale(
            picture, (self.state[-1], self.state[-1]))
        self.orig_image = self.image
        self.rect = self.image.get_rect(center=(self.state[0], self.state[1]))

    def update(self, dstate, dt=1 / FPS):
        self.state[2] = t[frame]
        return (radians(self.state[2]), radians(self.state[3]))

    def move(self):
        pass

    def rotate(self):
        """Rotate the image of the sprite around its center."""
        self.image = pygame.transform.rotozoom(
            self.orig_image, -self.state[2], 1)
        # Create a new rect with the center of the old rect.
        self.rect = self.image.get_rect(center=(self.state[0], self.state[1]))


class Player(pygame.sprite.Sprite):
    def __init__(self, picture, width, height):
        super().__init__()

        # state = posx, posy, velx, vely, accx, accy, width, height
        self.state = np.array(
            [CART_POS[0], CART_POS[1], 0, 0, 0, ACC, width, height])
        self.image = pygame.transform.scale(
            picture, (self.state[-2], self.state[-1]))
        self.rect = self.image.get_rect()

    def update(self):
        self.state[0] = x[frame] % WIDTH

        self.rect.midbottom = (self.state[0], self.state[1])

        hits = pygame.sprite.spritecollide(self, platforms, False)
        if P1.state[3] > 0:
            if hits:
                self.state[3] = 0
                self.state[1] = hits[0].rect.top + 1


class platform(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.surf = pygame.Surface((WIDTH, 20))
        self.surf.fill((255, 0, 0))
        self.rect = self.surf.get_rect(center=(WIDTH / 2, HEIGHT - 10))

    def move(self):
        pass


PT1 = platform()
P1 = Player(cart_img, 100, 30)
P2 = Pole(pole_img, 50)

cart_sprites = pygame.sprite.Group()
cart_sprites.add(P1)
cart_sprites.add(P2)

platforms = pygame.sprite.Group()
platforms.add(PT1)

img_sprites = pygame.sprite.Group()
img_sprites.add(P2)

state = np.array([5, 0.0, 0, 0.2])
while True:
    frame += 1
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                P1.jump()

    displaysurface.fill((255, 255, 255))
    for entity in platforms:
        displaysurface.blit(entity.surf, entity.rect)

    for entity in cart_sprites:
        entity.update()
    cart_sprites.draw(displaysurface)

    pygame.display.update()
    # Limits tick rate
    Clock.tick_busy_loop(FPS)
