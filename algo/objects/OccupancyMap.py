import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__ + '\..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from objects.Obstacle import Obstacle
import utils
import numpy as np
from typing import List
import matplotlib.pyplot as plt
from simulation.testing import get_maps

class OccupancyMap:
    def __init__(self, obstacles: List[Obstacle]=[]) -> None:
        """OccupancyMap Constructor

        Args:
            obstacles (List[Obstacle]): list of obstacle objects

        Parameters:
            xmin (float): left border
            xmax (float): right border
            ymin (float): bottom border
            ymax (float): top border
            start_g ((int, int)): starting grid numbers
            grid_vertices (np.array): 41x41 np array grid vertices for path planning (agent travels along grid lines)
            grid_display (np.array): 40x40 np array grid representing map for display purposes
            checkpoints (List[Checkpoint]): list of checkpoint objects
        """

        assert len(obstacles) <= 8      # ensure list has at most 8 obstacles
        self.xmin, self.xmax, self.ymin, self.ymax = 0., 200., 0., 200.
        self.obstacles = []

        self.occupancy_grid = np.zeros((40, 40))  # 40x40 occupancy grid
        
        self.add_obstacles_to_grids(obstacles)

    def add_obstacles_to_grids(self, obstacles: List[Obstacle]) -> None:
        """Function to update grids and checkpoints from list of obstacles

        Args:
            obstacles (List[Obstacle]): list of obstacle objects to add to the map
        """
        assert len(self.obstacles) + len(obstacles) <= 8    # ensure list has at most 8 obstacles

        self.obstacles += obstacles     # add obstacles to obstacle list

        # self.occupancy_grid[:3, :] = 1
        # self.occupancy_grid[-3:, :] = 1
        # self.occupancy_grid[:, :3] = 1
        # self.occupancy_grid[:, -3:] = 1

        # self.occupancy_grid[2, 2:8] = 0
        # self.occupancy_grid[2:8, 2] = 0

        for obstacle in obstacles:
            # add obstacle to grid vertices (including virtual wall)
            i_start = max(obstacle.x_g - 3, 0)      # 0: first index
            i_end = min(obstacle.x_g + 4, 39)       # 39: last index
            j_start = max(obstacle.y_g - 4, 0)      # 0: first index
            j_end = min(obstacle.y_g + 3, 39)       # 39: last index
            self.occupancy_grid[j_start:j_end+1, i_start:i_end+1] = 1

    
    def rotate_point(self, x, y, angle):
        x_rot = x*np.cos(angle) - y*np.sin(angle)
        y_rot = x*np.sin(angle) + y*np.cos(angle)
        return x_rot, y_rot

    def collide_with_point(self, x, y, theta):
        corners = [(x, y), (x + 30, y), (x, y - 30), (x + 30, y - 30)]
        for corner in corners:
            x_rot, y_rot = self.rotate_point(corner[0] - x, corner[1] - y, theta)
            x_g, y_g = utils.coords_to_grid(x_rot + x, y_rot + y)
            # print(f"Corner: ({corner[0]}, {corner[1]}) -> {utils.coords_to_android(x_rot + x, y_rot + y)}")
            if x_g < 0 or x_g >= 40 or y_g < 0 or y_g >= 40:
                return 1
            elif self.occupancy_grid[x_g, y_g] == 1:
                return 1
        
        return 0


if __name__ == '__main__':
    obstacles = [Obstacle(8, 8, 'N', 1), Obstacle(3, 18, 'S', 2), Obstacle(10, 18, 'E', 3), Obstacle(14, 5, 'W', 4), 
                 Obstacle(13, 13, 'N', 5)]
    map = OccupancyMap(obstacles) 
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)
    print(map.occupancy_grid)

    print(map.collide_with_point(60, 100, np.pi/2))
    