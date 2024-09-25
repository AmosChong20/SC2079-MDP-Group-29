import math
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__ + "/..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from algo import utils
from algo.pathfinding import *
from algo.pathfinding.pathcommands import *
from algo.pathfinding.hamiltonian import Hamiltonian
from algo.pathfinding.hybrid_astar import HybridAStar
from algo.objects.OccupancyMap import OccupancyMap
from algo.objects.Obstacle import Obstacle
from algo.pathfinding.hamiltonian import obstacle_to_checkpoint
import numpy as np
import constants as c


class task1():
    def __init__(self):
        self.checkpoints = []
        self.paths = []
        self.commands = []
        self.android = []
        self.obstacleID = []
        self.imageID: list[str] = []
        
    def generate_path(self, message):
        robot_position = utils.android_to_coords(message["data"]["robot"]["x"], message["data"]["robot"]["y"])
        obstacles = []
        L=26.5*np.pi/4/5 # Can try changing to 26.25 
        minR=26.5

        for obstacle in message["data"]["obstacles"]:
            # obsDIR = obstacle["dir"]
            # if obsDIR == "N":
            #     invertObs = "S"
            # elif obsDIR == "S":
            #     invertObs = "N"
            # elif obsDIR == "W":
            #     invertObs = "E"
            # elif obsDIR == "E":
            #     invertObs = "W"
            obstacles.append(Obstacle(obstacle["x"], obstacle["y"], obstacle["dir"], int(obstacle["id"])))

        map = OccupancyMap(obstacles)
        tsp = Hamiltonian(map, obstacles, robot_position[0], robot_position[1], math.pi/2, 0, 'euclidean', minR) # 3rd element: (N: np.pi/2, E: 0)
        current_pos = tsp.start
        obstacle_path = tsp.find_nearest_neighbor_path()
        print("Obstacle path: ", obstacle_path)
        for idx, obstacle in enumerate(obstacle_path):
            valid_checkpoints, camera_viewpoints = obstacle_to_checkpoint(map, obstacle, theta_offset=0, get_all=True)
            print(f"Obstacle {obstacle.id} at (x: {obstacle.android_x}, y: {obstacle.android_y}) has {len(valid_checkpoints)} checkpoints.")
            path = None
            while path == None and valid_checkpoints:
                checkpoint = valid_checkpoints.pop(0)
                camera_viewpoint = camera_viewpoints.pop(0)

                # print(f"Routing to obstacle ({utils.android_to_coords(obstacle.android_x, obstacle.android_y)}), x: {checkpoint[0]}, y: {checkpoint[1]} theta: {checkpoint[2]*180/np.pi}...")
                algo = HybridAStar(map=map, 
                            x_0=current_pos[0], y_0=current_pos[1], theta_0=current_pos[2], 
                            x_f=checkpoint[0], y_f=checkpoint[1], 
                            theta_f=checkpoint[2], steeringChangeCost=10, gearChangeCost=10, 
                            L=L, minR=minR, heuristic='euclidean', simulate=False, thetaBins=24)
                path, pathHistory = algo.find_path()
                if path == None:
                    print("Path failed to converge, trying another final position...")

            if path != None:
                print(f"Camera Viewpoint: {camera_viewpoint}, Checkpoint: {checkpoint}, Obstacle: {utils.android_to_coords(obstacle.android_x, obstacle.android_y)}")
                self.paths.append(path)
                current_pos = (path[-1].x, path[-1].y, path[-1].theta)
                commands, pathDisplay = construct_path_2(path, L, minR)
                self.commands.append(commands)
                self.android.append(pathDisplay)
                self.obstacleID.append(checkpoint[3])
                print_path(path)
            
            else:
                print("Path could not be found, routing to next obstacle...")
        
    
    def get_command_to_next_obstacle(self):
        nextCommand = None
        nextPath = None
        if self.commands:
            nextCommand = self.commands.pop(0)
        if self.android:
            nextPath = self.android.pop(0)
        return construct_json(nextCommand, nextPath)

    def get_obstacle_id(self):
        obstacle_id = self.obstacleID.pop(0)
        return obstacle_id
    
    def has_task_ended(self):
        return not self.commands
    
    def update_image_id(self, imageID):
        self.imageID.append(imageID)

    def get_image_id(self):
        return self.imageID


if __name__ == "__main__":
    message = {"type": "START_TASK", "data": {"task": "EXPLORATION", "robot": {"id": "R", "x": 1, "y": 1, "dir": 'N'},
                                               "obstacles": [{"id": "00", "x": 8, "y": 8, "dir": 'N'},
                                                             {"id": "01", "x": 7, "y": 15, "dir": 'S'},
                                                            #  {"id": "02", "x": 10, "y": 18, "dir": 'E'},
                                                             {"id": "03", "x": 14, "y": 5, "dir": 'W'},
                                                             {"id": "04", "x": 13, "y": 13, "dir": 'N'}]}}
    main = task1()
    main.generate_path(message)
    while not main.has_task_ended():
        print(main.get_command_to_next_obstacle())
        print("ID: " + str(main.get_obstacle_id()))
