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

def generate_cubic_hermite_spline(P0, P1, T0, T1, delta_t= 1/50):
    path= []
    pose= P0
    t=0.0
    dist= 0.0
    while np.sqrt((P1[0] - pose[0])**2 + (P1[1] - pose[1])**2) > 1e-6:
        #New pose computing
        h00 = 2*t**3 - 3*t**2 + 1
        h10 = t**3 - 2*t**2 + t
        h01 = -2*t**3 + 3*t**2
        h11 = t**3 - t**2
        pose= h00 * P0 + h10 * T0 + h01 * P1 + h11 * T1
        path.append(pose)
        if len(path) > 1:
            dist+= abs(utils.distance(path[-1], path[-2]))
        t+= delta_t
    path.append(P1)
    return path, dist, t

#def generate_cubic_be

#Fonction Dichotomie pour trouver la vitesse max du profil de vitesse trianulaire | Quand robot.max_lin_speed est inatteignable
def get_new_vmax(robot, target_waypoint, dist_path, speed_tolerance= 0.1):
    values= [robot.current_lin_speed, target_waypoint.speed, robot.max_lin_speed]
    l_bound= min(values)
    h_bound= max(values)
    target_speed= (h_bound + l_bound) / 2
    #Tant que l'espace de recherche n'est pas assez petit (Tant qu'on a pas trouvé la vitesse max du profil de vitesse triangulaire)
    while(abs(h_bound - l_bound) > speed_tolerance):
        #Calcul des distances d'accélération et de décélération
        _, dist_accel= utils.compute_time_dist_to_speed(robot.current_lin_speed, target_speed, robot.max_lin_speed)
        _, dist_decel= utils.compute_time_dist_to_speed(target_speed, target_waypoint.speed, -robot.max_lin_speed)
        if dist_accel + dist_decel > dist_path:
            h_bound= target_speed
            target_speed= (h_bound + l_bound) / 2
        #Si on a une vitesse trop petite | Augmentation de la borne basse de l'intervalle de recherche
        elif dist_accel + dist_decel < dist_path:
            l_bound= target_speed
            target_speed= (h_bound + l_bound) / 2
    return target_speed

#ALGORITHME DE PATH PLANNING | Progression Chemin Théorique et Chemin réel avec le même dt & Arrivée au Waypoint dans la direction voulue / Vitesse voulue
def get_side_waypoints(robot, target_waypoint, dt= 1/200):
    real_path= [utils.WayPoints(robot.current_location[0], robot.current_location[1], robot.current_lin_speed, robot.current_theta)]
    theoric_path, theoric_dist, _ = generate_cubic_hermite_spline(np.array([robot.current_location[0], robot.current_location[1]]), 
                                                 np.array([target_waypoint.x, target_waypoint.y]), 
                                                 np.array([math.cos(robot.current_theta), math.sin(robot.current_theta)]), 
                                                 np.array([math.cos(target_waypoint.theta), math.sin(target_waypoint.theta)]), dt)
    theoric_path= np.array(theoric_path)
    #Calcul des distances d'accélération et de décélération
    vmax= robot.max_lin_speed
    t_start ,dist_starting_ramp= utils.compute_time_dist_to_speed(robot.current_lin_speed, vmax, robot.max_lin_speed)
    _, dist_target_speed= utils.compute_time_dist_to_speed(robot.current_lin_speed, target_waypoint.speed, robot.max_lin_speed)
    t_finish, dist_finish_ramp= utils.compute_time_dist_to_speed(target_waypoint.speed, vmax, robot.max_lin_speed) 
    #SI la vitesse cible n'est pas atteignable (La vitesse au waypoint cible est trop importante)
    if dist_target_speed > theoric_dist:
        raise ValueError("The target speed is not reachable")
    #SINON SI on a pas le temps de réaliser les 2 rampes de vitesses complètes
        #On va diminuer pour trouver la vitesse max atteignable pour faire un profil de vitesse triangulaire
    elif dist_starting_ramp + dist_finish_ramp > theoric_dist:
        vmax= get_new_vmax(robot, target_waypoint, theoric_dist)
        t_start ,dist_starting_ramp= utils.compute_time_dist_to_speed(robot.current_lin_speed, vmax, robot.max_lin_speed)
        t_finish, dist_finish_ramp= utils.compute_time_dist_to_speed(target_waypoint.speed, vmax, robot.max_lin_speed) 
    dist_vmax= theoric_dist - dist_starting_ramp - dist_finish_ramp
    real_dist=0
    dist= 0
    time= 0
    while len(theoric_path) > 0:
        if len(theoric_path) > 1:
            dist+= abs(utils.distance(theoric_path[0], theoric_path[1]))
        else:
            dist= theoric_dist
        while real_dist < dist:
            if dist < dist_starting_ramp:
                speed= real_path[-1].speed + robot.max_lin_speed*dt
            elif dist < dist_starting_ramp + dist_vmax:
                speed= vmax
            elif dist <= theoric_dist:
                speed= real_path[-1].speed - robot.max_lin_speed*dt
            if speed <= 0.0:
                break
            x= real_path[-1].x + (speed * math.cos(real_path[-1].theta))*dt
            y= real_path[-1].y + (speed * math.sin(real_path[-1].theta))*dt
            real_dist+= abs(utils.distance([x, y], [real_path[-1].x, real_path[-1].y]))
            if len(theoric_path) > 1:
                theta= math.atan2(theoric_path[1, 1] - theoric_path[0, 1], theoric_path[1, 0] - theoric_path[0, 0])
            else:
                theta= target_waypoint.theta
            real_path.append(utils.WayPoints(x, y, speed, theta))
            time+= dt
        theoric_path= np.delete(theoric_path, 0, 0)
    #On va maintenant mettre à jour l'état du robot
    robot.current_location= [real_path[-1].x, real_path[-1].y]
    robot.current_lin_speed= real_path[-1].speed
    robot.current_theta= real_path[-1].theta
    return real_path


if __name__ == "__main__":
    fig, ax = plt.subplots(2, 1)
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
    cs_x = CubicHermiteSpline(np.arange(len(target_locs)), target_locs[:, 0], tangents[:, 0])
    cs_y = CubicHermiteSpline(np.arange(len(target_locs)), target_locs[:, 1], tangents[:, 1])
    t= cs_y.integrate(cs_y.x[0], cs_y.x[-1])
    echantillon= 50
    dt= 1/echantillon
    # Evaluate the spline at several points
    t_new = np.linspace(0, len(target_locs) - 1, echantillon)
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)
    wp_infos= []
    ax[0].set_title("Algorithme de Path Planning -> " + str(dt) + "Hz")
    ax[0].plot(x_new, y_new, label='Cubic Hermite Spline Path (Theoric Path)')
    ax[0].scatter(target_locs[:, 0], target_locs[:, 1], marker="o", color="red", label="Target Waypoints")
    #TEST Algo V1
    side_waypoints= np.array(get_side_waypoints(robot, target_waypoints[1], dt))
    wp_infos.append([side_waypoints.shape[0]-1, side_waypoints[-1].speed, target_waypoints[1].speed])
    for i in range(2, len(target_waypoints)):
        new_side_waypoints= np.array(np.array(get_side_waypoints(robot, target_waypoints[i], dt)))
        side_waypoints= np.concat((side_waypoints, new_side_waypoints))
        wp_infos.append([side_waypoints.shape[0]-1, side_waypoints[-1].speed, target_waypoints[i].speed])
    side_waypoints= side_waypoints.tolist()
        #ax.plot([side_waypoints[i].x for i in range(len(side_waypoints))], [side_waypoints[i].y for i in range(len(side_waypoints))])
    ax[0].plot([side_waypoints[i].x for i in range(len(side_waypoints))], [side_waypoints[i].y for i in range(len(side_waypoints))], label="Real Path")
    for wp in target_waypoints:
        ax[0].arrow(wp.x, wp.y, 0.1 * math.cos(wp.theta), 0.1 * math.sin(wp.theta), head_width=0.05, head_length=0.1, fc='green', ec='green')
    ax[1].plot([side_waypoints[i].speed for i in range(len(side_waypoints))], label="Profil de Vitesse")
    ax[1].scatter([wp_infos[i][0] for i in range(len(wp_infos))], [wp_infos[i][1] for i in range(len(wp_infos))], label="Vitesse Réelle au Waypoint Target", color="red")
    ax[1].scatter([wp_infos[i][0] for i in range(len(wp_infos))], [wp_infos[i][2] for i in range(len(wp_infos))], label="Vitesse Attendue au Waypoint Target", color="green")
    ax[0].legend()
    ax[1].legend()
    plt.show()