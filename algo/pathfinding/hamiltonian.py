import math
import random
import itertools
import numpy as np
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__ + "/..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import utils
from objects.Obstacle import Obstacle
from objects.OccupancyMap import OccupancyMap
import pathfinding.reeds_shepp as rs
import constants as c
import copy
from typing import List

class Hamiltonian():
    def __init__(self, map: OccupancyMap, obstacles: List[Obstacle], x_start: float, y_start: float, theta_start: float, theta_offset=0, 
                          metric='euclidean', minR=25) -> None:
        assert -np.pi < theta_start, theta_offset <= np.pi
        self.map = map
        self.obstacles = obstacles
        self.checkpoints = []
        self.start = (x_start, y_start, theta_start)
        self.theta_offset = theta_offset
        self.metric = metric
        self.minR = minR

    def find_brute_force_path(self):
        obstacle_permutations = itertools.permutations(self.obstacles)
        shortest_distance = float('inf')
        for obstacle_path in obstacle_permutations:
            current_pos = self.start
            total_distance = 0
            for obstacle in obstacle_path:
                checkpoint = obstacle_to_checkpoint(self.map, obstacle, self.theta_offset, get_all=False)
                if self.metric == 'euclidean':
                    distance = utils.l2(current_pos[0], current_pos[1], checkpoint[0], checkpoint[1])
                elif self.metric == 'reeds-shepp':
                    distance = rs.get_optimal_path_length(current_pos, checkpoint, self.minR)

                total_distance += distance
                current_pos = checkpoint
            if total_distance < shortest_distance:
                shortest_distance = total_distance
                shortest_path = obstacle_path[:]
        return shortest_path

    def find_nearest_neighbor_path(self):
        current_pos = self.start
        path = []

        obstacles = self.obstacles.copy()

        checkpoints = {}
        while obstacles:
            nearest_neighbor = None
            minDist = float('inf')
            for obstacle in obstacles:
                checkpoint = obstacle_to_checkpoint(self.map, obstacle, self.theta_offset, get_all=False)

                if checkpoint == None:
                    obstacles.remove(obstacle)
                    continue
                checkpoints[obstacle] = checkpoint
                if self.metric == 'euclidean':
                    dist = utils.l2(current_pos[0], current_pos[1], checkpoint[0], checkpoint[1])
                elif self.metric == 'reeds-shepp':
                    dist = rs.get_optimal_path_length(current_pos, checkpoint, self.minR)
                
                if dist < minDist:
                    minDist = dist
                    nearest_neighbor = obstacle
                
            
            path.append(nearest_neighbor)
            obstacles.remove(nearest_neighbor)
            current_pos = obstacle_to_checkpoint(self.map, nearest_neighbor, self.theta_offset, get_all=False)
        return path

def obstacle_to_checkpoint(map: OccupancyMap, obstacle: Obstacle, theta_offset, get_all=False):
    obstacle_x, obstacle_y = utils.android_to_coords(obstacle.android_x, obstacle.android_y)
    obstacle_x += offset_x(obstacle.facing)
    obstacle_y += offset_y(obstacle.facing)
    obstacle_image_to_pos_theta = offset_theta(obstacle.facing, np.pi)

    print(f"Obstacle position: ({obstacle_x}, {obstacle_y})")

    valid_checkpoints = []
    camera_viewpoints = []

    theta_scan_list = [0, np.pi/36, -np.pi/36, np.pi/18, -np.pi/18, np.pi/12, -np.pi/12, 
                       np.pi/9, -np.pi/9, np.pi/7.2, -np.pi/7.2, np.pi/6, -np.pi/6,
                       np.pi*180/35, -np.pi*180/35, np.pi/4.5, -np.pi/4.5, np.pi/4, -np.pi/4]
    r_scan_list = [20, 19, 21, 18, 22, 17, 23, 16, 24, 15, 25, 26, 27, 28, 29, 30]

    
    for r_scan in r_scan_list:
        for theta_scan in theta_scan_list:
            cur_image_to_pos_theta = utils.M(obstacle_image_to_pos_theta + theta_scan)
            camera_x = obstacle_x - r_scan*np.cos(cur_image_to_pos_theta)
            camera_y = obstacle_y - r_scan*np.sin(cur_image_to_pos_theta)
            camera_theta = utils.M(cur_image_to_pos_theta - theta_offset)

            robot_x, robot_y = utils.get_bottom_left_position_from_camera_pov(camera_x, camera_y, camera_theta)

            if not map.collide_with_point(robot_x, robot_y, camera_theta):
                if not get_all:
                    return (robot_x, robot_y, camera_theta, obstacle.id)
                else:
                    valid_checkpoints.append((robot_x, robot_y, camera_theta, obstacle.id))
                    camera_viewpoints.append((camera_x, camera_y, camera_theta))

    return valid_checkpoints, camera_viewpoints


def offset_x(facing: str):
    if facing == 'N':
        return 5.
    elif facing == 'S':
        return 5.
    elif facing == 'E':
        return 10.
    elif facing == 'W':
        return 0.


def offset_y(facing: str):
    if facing == 'N':
        return 10.
    elif facing == 'S':
        return 0.
    elif facing == 'E':
        return 5.
    elif facing == 'W':
        return 5.
    
def offset_theta(facing: str, theta_offset: float):
    return utils.M(utils.facing_to_rad(facing) + theta_offset)

def generate_random_obstacles(grid_size, obstacle_count):
    if grid_size < 100:
        offset = 5
    else:
        offset = 50
    obstacles = []
    directions = ['N', 'S', 'E', 'W']

    while len(obstacles) < obstacle_count:
        x = random.randint(offset, grid_size - offset)
        y = random.randint(offset, grid_size - offset)
        direction = random.choice(directions)
        obstacles.append(Obstacle(x, y, direction))

    return obstacles


def print_grid(grid_size, obstacles):
    path = []
    for y in range(grid_size - 1, -1, -1):
        for x in range(grid_size):
            position = (x, y)
            if (0 <= x <= 2) and (0 <= y <= 2):
                print("C", end=" ")  # Starting point
            elif any(obstacle.x_g == x and obstacle.y_g == y for obstacle in obstacles):
                direction = next(
                    (obstacle.facing for obstacle in obstacles if (obstacle.x_g, obstacle.y_g) == position), None)
                print(direction, end=" " if direction else ".")  # Obstacle facing direction
            elif (x, y) in path:
                print("*", end=" ")  # Mark the path with "*"
            else:
                print(".", end=" ")  # Empty space
        print()


if __name__ == "__main__":
    # Conversion function tests
    # obstacle = Obstacle(8, 8, 'N', 1)
    # x_g, y_g = obstacle.x_g, obstacle.y_g
    # android_x, android_y = obstacle.android_x, obstacle.android_y
    # x, y = utils.grid_to_coords(x_g, y_g)
    # new_x_g, new_y_g = utils.coords_to_grid(x, y)
    # coord_x, coord_y = utils.android_to_coords(android_x, android_y)
    # new_android_x, new_android_y = utils.coords_to_android(coord_x, coord_y)
    # print(f"Obstacle: {obstacle}, Grid position: ({x_g}, {y_g}), Grids to Coords: ({x}, {y}), Coords to Grid position: ({new_x_g}, {new_y_g})")
    # print(f"Android position: ({android_x}, {android_y}),  Android to Coords: ({coord_x}, {coord_y}), Coords to Android position: ({new_android_x}, {new_android_y})")
    # print()

    # print(f"Camera POV (8, 8)")
    # print("Facing North")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, math.pi/2)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")
    # print(f"Facing East")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, 0)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")
    # print(f"Facing South")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, -math.pi/2)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")
    # print(f"Facing West")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, math.pi)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")
    # print("Facing North East")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, math.pi/4)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")
    # print("Facing North West")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, 3*math.pi/4)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")
    # print("Facing South East")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, -math.pi/4)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")
    # print("Facing South West")
    # bottom_left = utils.get_bottom_left_position_from_camera_pov(70, 70, -3*math.pi/4)
    # print(f"Robot position (bottom left): {utils.coords_to_android(bottom_left[0], bottom_left[1])}")

    
    obstacles = [Obstacle(8, 8, 'N', 1), Obstacle(3, 15, 'S', 2), Obstacle(10, 18, 'E', 3), Obstacle(14, 5, 'W', 4), 
                 Obstacle(13, 13, 'N', 5)]
    obstacles = [Obstacle(14, 5, 'W', 4)]
    map = OccupancyMap(obstacles) 
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)
    print(map.occupancy_grid)

    print(map.collide_with_point(60, 100, np.pi/2))
    x_start = 1
    y_start = 1
    robot_position = utils.grid_to_coords(x_start, y_start)
    tsp = Hamiltonian(map, obstacles, robot_position[0], robot_position[1], 0, 0, 'euclidean')
    path = tsp.find_nearest_neighbor_path()
    print("\nShortest Path:")
    print(path)

