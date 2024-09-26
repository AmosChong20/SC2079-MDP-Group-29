import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__ + '\..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
import algo.utils as utils
import pygame
import algo.constants as c

class Obstacle(pygame.sprite.Sprite):
    def __init__(self, x: int, y: int, facing: str, id: int = -1) -> None:
        """Obstacle constructor

        Args:
            x_g (int): x coordinate of obstacle (bottom left grid)
            y_g (int): y coordinate of obstacle (bottom left grid)
            facing (str): {'N', 'S', 'E', 'W'} direction of image i.e. which direction agent needs to face to see image

        Parameters:
            theta (float): direction of image in radians
        """
        # pygame.sprite.Sprite.__init__(self)
        # self.image = pygame.image.load('algo/objects/images/obstacle.png')
        # self.rect = self.image.get_rect()
        # self.rect.bottomleft = utils.coords_to_pixelcoords(x, y)
        
        grid_x, grid_y = utils.android_to_grid(x, y)
        self.x_g = grid_x
        self.y_g = grid_y
        self.android_x = x
        self.android_y = y
        self.facing = facing
        self.id = id
        self.theta = utils.facing_to_rad(facing)

        # if facing == 'E':
        #     self.image = pygame.transform.rotate(self.image, 270)
        # elif facing == 'S':
        #     self.image = pygame.transform.rotate(self.image, 180)
        # elif facing == 'W':
        #     self.image = pygame.transform.rotate(self.image, 90)
    
    def __repr__(self) -> str:
        return f"Obstacle({self.android_x}, {self.android_y}, {self.facing}, {self.id})"

# class VirtualWall(pygame.sprite.Sprite):
#     def __init__(self, x_g: int, y_g: int) -> None:
#         pygame.sprite.Sprite.__init__(self)
#         self.x_g = x_g
#         self.y_g = y_g
#         self.width = (min(self.x_g + 8, c.GRID_SIZE) - self.x_g)*200/c.GRID_SIZE*c.MAP_WIDTH/200
#         self.height = (min(self.y_g + 8, c.GRID_SIZE) - self.y_g)*200/c.GRID_SIZE*c.MAP_HEIGHT/200
#         print(f"Virtual wall at ({self.x_g}, {self.y_g}) with width {self.width} and height {self.height}")
#         self.image = pygame.Surface((self.width, self.height))
#         self.image.fill((255, 0, 0, 24))
#         self.rect = self.image.get_rect()
#         self.rect.bottomleft = utils.coords_to_pixelcoords(self.x_g, self.y_g)

        
