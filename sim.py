import pygame
from pygame.locals import *
import sys
import random
import numpy as np
from math import degrees, radians, sin, cos
import random
from control import matlab, lqr

pygame.init()
HEIGHT = 400
WIDTH = 600
FPS = 100  # adjust tau in CartPole
CART_POS = [100, 50]
CART_WIDTH = 75
CART_HEIGHT = 20
frame = 0
USER = True
SCALE = 20
# scale is about 10 pixels per meter


class CartPole:
    g = 9.81
    massCart = 1
    massPole = 0.15
    poleLength = 2.5
    tau = 0.01
    forceMag = 30
    # variables in the linear approximation
    dissipation = 0
    downwards = 1

    def ddt_calc(self, t, dt, f):
        num = (
            f * cos(t)
            - self.massPole * self.poleLength * cos(t) * sin(t) * dt**2
            + (self.massPole + self.massCart) * self.g * sin(t)
        )
        denum = self.poleLength * (self.massCart + self.massPole * sin(t) ** 2)
        return num / denum

    def ddx_calc(self, t, dt, f):
        num = f + self.massPole * \
            sin(t) * (self.g * cos(t) - self.poleLength * dt**2)
        denum = self.massCart + self.massPole * sin(t) ** 2
        return num / denum

    def __init__(self, params=None, sim="default"):
        if params:
            self.state = params
        else:
            self.state = [
                random.random() - 0.5,
                (random.random() - 0.5) * 1,
                radians(random.random() * 360),
                (random.random() - 0.5) * 0.5,
            ]
        self.sim = sim

        # Matrices of the linear approximation of state-space model
        self.A_lin = np.array(
            [
                [0, 1, 0, 0],
                [
                    0,
                    -self.dissipation / self.massCart,
                    -self.massPole * self.g / self.massCart,
                    0,
                ],
                [0, 0, 0, 1],
                [
                    0,
                    -self.downwards
                    * self.dissipation
                    / (self.massCart * self.poleLength),
                    -self.downwards
                    * (self.massPole + self.massCart)
                    * self.g
                    / (self.poleLength * self.massCart),
                    0,
                ],
            ]
        )
        # nx1
        self.B_lin = np.array(
            [
                0,
                1 / self.massCart,
                0,
                self.downwards * -1 / (self.massCart * self.poleLength),
            ]
        )[:, np.newaxis]

        self.state_penalty = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 10, 0], [0, 0, 0, 100]]
        )
        self.force_penalty = np.array([0.1])

        # LQR matrices
        # reinforcement

    def dstate(self, state, force):
        x, dx, t, dt = state
        ddt = self.ddt_calc(t, dt, force)
        ddx = self.ddx_calc(t, dt, force)

        return np.array([dx, ddx, dt, ddt])

    def dstate_linear(self, state, force):
        k = self.A_lin @ state + self.B_lin[:, 0] * force
        print(k)
        return self.A_lin @ state + self.B_lin[:, 0] * force

    def setControl(self, mode, target=np.array([10, 0, 0, 0])):
        self.mode = mode
        if mode == "linear_place":
            desired = np.array([[-3, -3.1, -3.2, -3.3]])
            self.gain = np.array(matlab.place(self.A_lin, self.B_lin, desired))
        elif mode == "linear_qr":
            K, S, E = lqr(
                self.A_lin, self.B_lin, self.state_penalty, self.force_penalty
            )
            self.gain = np.squeeze(np.array(K))
            self.target = target
            print(self.A_lin, self.B_lin, self.gain, self.gain.shape)
        elif mode == "reinforcement":
            pass

    def control(self):
        if "linear" in self.mode:
            # print(self.A_lin)
            print(
                self.gain,
                self.target,
                self.state,
                -(self.gain @ (self.state - self.target)),
            )
            return -(self.gain @ (self.state - self.target))

    def update(self, ext):
        if USER:
            force = ext * self.forceMag
        else:
            force = self.control()
        # np array operations
        if self.sim == "linear":
            dstate = self.tau * self.dstate_linear(self.state, force)
        else:
            dstate = self.tau * self.dstate(self.state, force)
        self.state += dstate
        print(system.state, force)


Clock = pygame.time.Clock()

displaysurface = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Game")

cart_img = pygame.image.load("transport-industrial-truck.png")
pole_img = pygame.image.load("pole.png")


class Pole(pygame.sprite.Sprite):
    def __init__(self, picture, length):
        super().__init__()
        # state = posx, posy, angle, vel, acc, length
        self.height = 2 * length
        self.angle = 0
        self.center = (0, 0)

        self.image = pygame.transform.scale(
            picture, (self.height, self.height))
        self.orig_image = self.image
        self.rect = self.image.get_rect(center=self.center)

    def update(self, angle, dt=1 / FPS):
        # self.state[2] = t[frame]
        self.angle = angle
        self.center = (
            P1.x,
            P1.y - CART_HEIGHT,
        )
        self.rotate()

    def move(self):
        pass

    def rotate(self):
        """Rotate the image of the sprite around its center."""
        self.image = pygame.transform.rotozoom(self.orig_image, self.angle, 1)
        # self.image = pygame.transform.rotozoom(self.orig_image, self.angle + 180, 1)
        # Create a new rect with the center of the old rect.
        self.rect = self.image.get_rect(center=self.center)


class Player(pygame.sprite.Sprite):
    def __init__(self, picture, width, height):
        super().__init__()

        # state = posx, posy, width, height
        self.x, self.y = CART_POS[0], CART_POS[1]
        self.yv = 15
        self.image = pygame.transform.scale(picture, (width, height))
        self.rect = self.image.get_rect()

    # def jump(self):
    #     hits = pygame.sprite.spritecollide(self, platforms, False)
    #     if hits:
    #         self.yv = -15

    def update(self, x):
        self.x = x % WIDTH
        self.rect.midbottom = (self.x, self.y)

        self.y += self.yv
        hits = pygame.sprite.spritecollide(self, platforms, False)
        if hits:
            self.yv = 0
            self.y = hits[0].rect.top + 1


class platform(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.surf = pygame.Surface((WIDTH, 20))
        self.surf.fill((255, 0, 0))
        self.rect = self.surf.get_rect(center=(WIDTH / 2, HEIGHT - 10))

    def move(self):
        pass


PT1 = platform()
# config
# system = CartPole([15, 0, np.pi, 2], sim="default")
system = CartPole([15, 0, 0, 2], sim="default")
system.setControl("linear_qr")
P1 = Player(cart_img, CART_WIDTH, CART_HEIGHT)
P2 = Pole(pole_img, system.poleLength * SCALE)

cart_sprites = pygame.sprite.Group()
cart_sprites.add(P1)
cart_sprites.add(P2)

platforms = pygame.sprite.Group()
platforms.add(PT1)

img_sprites = pygame.sprite.Group()
img_sprites.add(P2)


def get_input():
    pressed_keys = pygame.key.get_pressed()
    global USER
    if pressed_keys[K_UP]:
        USER = False
    if pressed_keys[K_DOWN]:
        USER = True
    if not USER:
        return 0
    if pressed_keys[K_LEFT]:
        return -1
    elif pressed_keys[K_RIGHT]:
        return 1
    return 0


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

    # cart system
    # for entity in cart_sprites:
    #     entity.update(system.state)
    P1.update(system.state[0] * SCALE)
    P2.update(degrees(system.state[2]))
    cart_sprites.draw(displaysurface)

    k = get_input()
    system.update(k)
    # displaysurface.blit(P1.surf, P1.rect)
    # img_sprites.draw(displaysurface)

    pygame.display.update()
    # Limits tick rate
    Clock.tick_busy_loop(50)
