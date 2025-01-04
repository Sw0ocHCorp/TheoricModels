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


#ALGORITHME DE PATH PLANNING | Arrivée au Waypoint dans la direction voulue / Vitesse voulue
def get_side_waypoints(robot, theoric_path, target_waypoint):
    real_path= [utils.WayPoints(robot.current_location[0], robot.current_location[1], robot.current_lin_speed, robot.current_theta)]
    target_wp=[]
    vmax= robot.max_lin_speed
    #Calcul des distances d'accélération et de décélération
    _ ,dist_starting_ramp= utils.compute_time_dist_to_speed(robot.current_lin_speed, vmax, robot.max_lin_speed)
    _, dist_finish_ramp= utils.compute_time_dist_to_speed(vmax, target_waypoint.speed, -robot.max_lin_speed)   
    _, dist_target_speed= utils.compute_time_dist_to_speed(robot.current_lin_speed, target_waypoint.speed, robot.max_lin_speed)
    last_pos= sorted(theoric_path, key= lambda pos: abs(utils.distance(pos, [target_waypoint.x, target_waypoint.y])))[0]
    i_start= np.where((theoric_path == robot.current_location).all(axis=1))[0]
    if i_start.size == 0:
        p= theoric_path.tolist()
        p.sort(key= lambda pos: abs(utils.distance(pos, robot.current_location)))
        i_start= np.where((theoric_path == p[0]).all(axis=1))[0]
        #i_start= np.sort(np.argsort([utils.distance(robot.current_location, pos) for pos in theoric_path])[:2])
    i_first_ramp= -2
    i_second_ramp= -2
    i_finish= np.where((theoric_path == last_pos).all(axis=1))[0]
    #Récupération du chemin théorique entre le point de départ et le Waypoint cible
    path= deepcopy(theoric_path[i_start[0]:i_finish[0], :])
    dist_path= 0
    dist_path_r= 0
    #Récupération des indices de fin et début des phases d'accélérations / décélération
    for i in range(1, len(path)):
        j= len(path) - i-1
        dist_path+= abs(utils.distance(path[i], path[i-1]))
        dist_path_r += abs(utils.distance(path[j], path[j+1]))
        if i_first_ramp == -2 and dist_path >= dist_starting_ramp:
            i_first_ramp= i
        if i_second_ramp == -2 and dist_path_r >= dist_finish_ramp:
            i_second_ramp= j
    #SI la vitesse cible n'est pas atteignable
    if dist_target_speed > dist_path:
        raise ValueError("The target speed is not reachable")
    #SINON SI on a pas le temps de réaliser les 2 rampes de vitesses complètes (On va trouver un compromis)
        #On va changer de profil de vitesse Profil Trapézoïdal -> Profil Triangulaire
    elif i_second_ramp < i_first_ramp:
        vmax= get_new_vmax(robot, target_waypoint, dist_path)
        _, dist_starting_ramp= utils.compute_time_dist_to_speed(robot.current_lin_speed, vmax, robot.max_lin_speed)
    #On va maintenant générer le chemin réel grâce au profil de vitesse qu'on a déterminé et la direction du chemin théorique
    theoric_dist= 0
    real_dist= 0
    last_pos= path[0]
    path = np.delete(path, 0, 0)

    n=1
    while len(path) > 0:
        #Calcul de la vitesse du Robot en fonction de sa phase dans le profil de vitesse
            #Utilisation de n pour avoir des accélération et décélération plus importantes
            #i_first_ramp et i_second_ramp sont calculés sur le chemin théorique
            #Hors, la distance entre les points du chemin théorique et le chemin réel n'est pas la même (car distance entre 2 sides waypoints en fonction de la vitesse)
            # Si dans la le calcul de la vitesse, on uilise 1 / i_first_ramp ou 1 / i_second_ramp, on va avoir des variation de vitesse plus faibles
            #Les vitesses seront trop importantes, ce qui va engendrer une grosse dérive entre le chemin théorique et le chemin réel
        if real_dist < dist_starting_ramp:
            speed= min(vmax, real_path[-1].speed + (robot.max_lin_speed * n/i_first_ramp))
        elif real_dist < dist_path - dist_finish_ramp:
            speed= vmax
        elif real_dist < dist_path:
            speed= max(target_waypoint.speed, real_path[-1].speed - (robot.max_lin_speed * n/i_first_ramp))
        #Suppression des positions intermédiaire pour rattrapper le retard avec le vrai tracé
        if theoric_dist <= real_dist:
            n+=1
            #MAJ de la distance du tracé théorique
            theoric_dist += abs(utils.distance(path[0], last_pos))
            last_pos= path[0]
            path = np.delete(path, 0, 0)
        #Calcul de la nouvelle position du Robot si on a du retard par rapport au tracé théorique
        if real_dist <= theoric_dist:
            x= real_path[-1].x + (speed * math.cos(real_path[-1].theta) * 1/i_first_ramp)
            y= real_path[-1].y + (speed * math.sin(real_path[-1].theta) * 1/i_first_ramp)
            #MAJ de la distance réelle parcourue par le Robot
            real_dist+= abs(utils.distance([x, y], [real_path[-1].x, real_path[-1].y]))
            if len(path) >= 2:
                theta= math.atan2(path[1, 1] - path[0, 1], path[1, 0] - path[0, 0])
            else:
                theta= target_waypoint.theta
            real_path.append(utils.WayPoints(x, y, speed, theta))
            n= 1
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
    
    target_waypoints = np.array([utils.WayPoints(0,0,0,math.pi/2), utils.WayPoints(3, 3, 0.5, math.pi), 
                                 utils.WayPoints(-3, 3, 1.0, -math.pi/2),
                                 utils.WayPoints(-3, -3, 1.5, 0), 
                                 utils.WayPoints(3, -3, 2.0, math.pi/2)])
    
    robot = utils.Robot([0, 0], math.pi/2, 0, 2.5, math.pi/10)
    target_locs = np.array([[wp.x, wp.y] for wp in target_waypoints])
    tangents = np.array([[math.cos(wp.theta), math.sin(wp.theta)] for wp in target_waypoints])
    
    # Create cubic Hermite splines for x and y coordinates
    cs_x = CubicHermiteSpline(np.arange(len(target_locs)), target_locs[:, 0], tangents[:, 0])
    cs_y = CubicHermiteSpline(np.arange(len(target_locs)), target_locs[:, 1], tangents[:, 1])
    t= cs_y.integrate(cs_y.x[0], cs_y.x[-1])
    echantillon= 1000
    # Evaluate the spline at several points
    t_new = np.linspace(0, len(target_locs) - 1, echantillon)
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)
    wp_infos= []
    ax[0].set_title("Algorithme de Path Planning -> " + str(echantillon) + " Points entre chaque Waypoints pour le Chemin Théorique")
    ax[0].plot(x_new, y_new, label='Cubic Hermite Spline Path (Theoric Path)')
    ax[0].scatter(target_locs[:, 0], target_locs[:, 1], marker="o", color="red", label="Target Waypoints")
    side_waypoints= np.array(get_side_waypoints(robot, np.vstack((x_new, y_new)).T, target_waypoints[1]))
    wp_infos.append([side_waypoints.shape[0]-1, side_waypoints[-1].speed, target_waypoints[1].speed])
    for i in range(2, len(target_locs)):
        new_side_waypoints= np.array(get_side_waypoints(robot, np.vstack((x_new, y_new)).T, target_waypoints[i]))
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