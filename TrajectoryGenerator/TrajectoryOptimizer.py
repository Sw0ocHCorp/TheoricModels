import sys
import os
# Get the current script's directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory by going one level up
parent_dir = os.path.dirname(current_dir)
# Add the parent directory to sys.path
sys.path.append(parent_dir)

import utils
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from copy import deepcopy
from time import perf_counter
#import cv2 as cv
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, CubicHermiteSpline

#ALGORITHME DE PATH PLANNING | Arrivée au Waypoint dans la direction voulue / Vitesse voulue au Waypoint non assurée
#Idée de depart: On va faire le chemin à l'envers
    #De cette manière, on va pouvoir réaliser plus précisément la manoeuvre permettant de rallier l'angle voulue au niveau du waypoint, en partant de l'orientation courrante du robot(au départ)
def get_side_waypoints(robot, theoric_path, target_waypoint, speed_error_tolerance= 0.1, dist_error_tolerance= 0.1, theta_error_tolerance= 5, dt= 1/50):
    _ ,dist_starting_ramp= utils.compute_time_dist_to_speed(robot.current_lin_speed, robot.max_lin_speed, robot.max_lin_speed)
    _, dist_finish_ramp= utils.compute_time_dist_to_speed(robot.max_lin_speed, target_waypoint.speed, -robot.max_lin_speed)   
    _, dist_target_speed= utils.compute_time_dist_to_speed(robot.current_lin_speed, target_waypoint.speed, robot.max_lin_speed)
    last_pos= sorted(theoric_path, key= lambda pos: abs(utils.distance(pos, [target_waypoint.x, target_waypoint.y])))[0]
    i_start= np.where((theoric_path == robot.current_location).all(axis=1))[0][0]
    i_first_ramp= -2
    i_second_ramp= -2
    i_finish= np.where((theoric_path == last_pos).all(axis=1))[0][0]
    path= theoric_path[i_start:i_finish, :]
    dist_path= 0
    dist_path_r= 0
    for i in range(1, len(path)):
        j= len(path) - i-1
        dist_path+= abs(utils.distance(path[i], path[i-1]))
        dist_path_r += abs(utils.distance(path[j], path[j+1]))
        if i_first_ramp == -2 and dist_path >= dist_starting_ramp:
            i_first_ramp= i
        if i_second_ramp == -2 and dist_path_r >= dist_finish_ramp:
            i_second_ramp= j

    if dist_target_speed > dist_path:
        raise ValueError("The target speed is not reachable")
    #SINON SI on a pas le temps de réaliser les 2 rampes de vitesses (On va trouver un compromis)
    elif i_second_ramp < i_first_ramp:
        v_max= np.sqrt(2 * robot.max_lin_speed * dist_path + (robot.current_lin_speed**2 + target_waypoint.speed**2) / 2)
        _, dist_starting_ramp = utils.compute_time_dist_to_speed(robot.current_lin_speed, v_max, robot.max_lin_speed)
        _, dist_finish_ramp = utils.compute_time_dist_to_speed(v_max, target_waypoint.speed, -robot.max_lin_speed)
        i_first_ramp= (i_first_ramp + i_second_ramp)//2
        i_second_ramp= i_first_ramp+1
    else:

        pass
    t=1



if __name__ == "__main__":
    fig, ax = plt.subplots(1, 1)
    field = np.array([[utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2], 
                      [utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                      [-utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                      [-utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2]], dtype=float)
    
    target_waypoints = np.array([utils.WayPoints(0,0,0,math.pi/2), utils.WayPoints(1, 1, 0.5, math.pi), 
                                 utils.WayPoints(-1, 1, 1.0, -math.pi/2),
                                 utils.WayPoints(-1, -1, 1.5, 0), 
                                 utils.WayPoints(1, -1, 2.0, math.pi/2)])
    
    robot = utils.Robot([0, 0], math.pi/2, 0, 2.5, math.pi/10)
    target_locs = np.array([[wp.x, wp.y] for wp in target_waypoints])
    tangents = np.array([[math.cos(wp.theta), math.sin(wp.theta)] for wp in target_waypoints])
    
    # Create cubic Hermite splines for x and y coordinates
    """cs_x = CubicSpline(np.arange(len(target_locs)), target_locs[:, 0])
    cs_y = CubicSpline(np.arange(len(target_locs)), target_locs[:, 1])"""
    cs_x = CubicHermiteSpline(np.arange(len(target_locs)), target_locs[:, 0], tangents[:, 0])
    cs_y = CubicHermiteSpline(np.arange(len(target_locs)), target_locs[:, 1], tangents[:, 1])
    t= cs_y.integrate(cs_y.x[0], cs_y.x[-1])
    # Evaluate the spline at several points
    t_new = np.linspace(0, len(target_locs) - 1, 200)
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)
    
    #ax.plot(x_new, y_new, label='Cubic Hermite Spline Path')
    ax.scatter(target_locs[:, 0], target_locs[:, 1], marker="o", color="red", label="Target Waypoints")
    for i in range(1, len(target_locs)):
        speed_dynamic= get_side_waypoints(robot, np.vstack((x_new, y_new)).T, target_waypoints[i], 0.1, 0.1, 1, 1/50)
    for wp in target_waypoints:
        ax.arrow(wp.x, wp.y, 0.1 * math.cos(wp.theta), 0.1 * math.sin(wp.theta), head_width=0.05, head_length=0.1, fc='blue', ec='blue')
    
    ax.legend()
    plt.show()