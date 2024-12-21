import sys
import os
# Get the current script's directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory by going one level up
parent_dir = os.path.dirname(current_dir)
# Add the parent directory to sys.path
sys.path.append(parent_dir)
#sys.path.append('C:/Users/nclsr/Desktop/RCT/RoboCup/TheoricModels/PassingAlgorithms')

import utils
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from copy import deepcopy
from time import perf_counter
#import cv2 as cv
import matplotlib.pyplot as plt

#CALCUL DE LA MAP CONTROLEE PAR UNE TEAM ROBOT
def compute_control_map(robots, robot_init_speeds, robot_max_speeds, x, y, ax= None):
    start= perf_counter()
    robots_map= np.zeros((x.shape[0], y.shape[0]), dtype= float)
    for r in range(robots_map.shape[1]):
        for c in range(robots_map.shape[0]):
            target_pos= [x[c], y[r]]
            time_robot= math.inf
            for i, rob in enumerate(robots):
                if (utils.compute_time_to_position(rob, robot_init_speeds[i], robot_max_speeds[i], target_pos) < time_robot):
                    time_robot= utils.compute_time_to_position(rob, robot_init_speeds[i], robot_max_speeds[i], target_pos)
            robots_map[c, r]= time_robot
    print("Time Elapsed: ", perf_counter() - start)
    if ax is not None:
        im= ax.imshow(np.array(robots_map).T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Robot To Position", orientation="vertical", ax= ax) 
    return robots_map

#MAP DE TEMPS DEPLACEMENT DE BALLE
def compute_ball_map(ball_init_pos, x, y, ax= None, pass_init_speed= None, cf= 0.3):
    start= perf_counter()
    ball_map= np.zeros((x.shape[0], y.shape[0]), dtype= float)
    for r in range(ball_map.shape[1]):
        for c in range(ball_map.shape[0]):
            target_pos= [x[c], y[r]]
            pass_speed= pass_init_speed
            if pass_speed is None:
                pass_speed= utils.compute_ball_v0(abs(utils.distance(ball_init_pos, target_pos)))
            ball_map[c, r]= utils.time_ball_reach_pos(pass_speed, ball_init_pos, target_pos, cf)
    print("Time Elapsed: ", perf_counter() - start)
    if ax is not None:
        im= ax.imshow(np.array(ball_map).T, cmap= 'hot', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Ball To Position", orientation="vertical", ax= ax)
    return ball_map

#MAP DES ZONES CONTROLEES SUR LE TERRAIN PAR LES 2 EQUIPES
def compute_team_controlled_map(teammate_map, ops_map, ax= None):
    start= perf_counter()
    diff= teammate_map - ops_map
    print("Time Elapsed: ", perf_counter() - start)
    if ax is not None:
        im= ax.imshow(diff.T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Difference Teammates / Opponents", orientation="vertical", ax= ax)
    return diff

def compute_intercept_priorities(x, y , carrier, tms, ops, tms_current_speeds, tms_max_speeds, ops_current_speeds, ops_max_speeds, ax= None):
    priority_map= np.zeros((x.shape[0], y.shape[0]), dtype= float)
    for col, xi in enumerate(x):
        for row, yi in enumerate(y):
            target_pos= [xi, yi]
            ops_dist= math.inf
            for opponent in ops:
                intercept= utils.find_closest_point_on_segment(opponent, carrier, target_pos)
                d= abs(utils.distance(carrier, intercept))
                ops_dist = min(ops_dist, d)
            tm_dist= math.inf
            for tm in tms:
                intercept= utils.find_closest_point_on_segment(tm, carrier, target_pos)
                d= abs(utils.distance(carrier, intercept))
                tm_dist = min(tm_dist, d)
            
            if ops_dist <= tm_dist: 
                priority_map[col, row] = 1.0
    if ax is not None:
        im= ax.imshow(priority_map.T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Priorités sur l'Interception Teammates / Opponents", orientation="vertical", ax= ax)
    return priority_map
            

#CALCUL DES ZONES DE PASSES POUR L'EQUIPE POSSEDANT LA BALLE
def compute_pass_map(control_map, tm_intercept_map, ops_intercept_map, priority_map, ax= None, cf= 0.3):
    #On va uniquement travailler dans la zone conquise par l'équipe en attaque
    start= perf_counter()
    pass_map= deepcopy(control_map)
    pass_map[pass_map > 0.0]= 0.0
    pass_map[pass_map < 0.0]= 1.0
    cols, rows= np.where(pass_map > 0.0)
    for col in set(cols):
        for row in set(rows):
            diff_ops= ops_intercept_map[col, row]
            diff_tm= tm_intercept_map[col, row]
            intercept_priority= priority_map[col, row] 
            #SI Personne ne peut intercepter la balle | PASSE IMPOSSIBLE
            if diff_tm == 0.0 and diff_ops == 0.0:
                pass_map[col, row]*= 0.0
            #SI notre Team est prioritaire et qu'on peut intercepter le ballon
            #OU Qu'on est pas prioritaire mais qu'on peut intercepter le ballon alors que les adversaires NON | PASSE POSSIBLE
            elif (intercept_priority == 0.0 and diff_tm < 0.0) or \
                (intercept_priority == 1.0 and diff_tm < 0.0 and diff_ops == 0.0):
                pass_map[col, row]*= -(abs(diff_tm) - abs(diff_ops))
            #SINON SI les Adversaires sont prioritaires et peuvent intercepter la balle
            #OU Que les Adversaires ne sont pas prioritaires mais qu'ils peuvent intercepter le ballon alors que notre Team NON | PASSE IMPOSSIBLE
            elif (intercept_priority == 1.0 and diff_ops < 0.0) or \
                (intercept_priority == 0.0 and diff_ops < 0.0 and diff_tm == 0.0):
                pass_map[col, row]*= 0.0
            #VOIR DANS QUEL CAS ON EST DANS LE else
            #S'IL Y A DES CAS NON ANTICIPES
            else:
                t=1
            

    if ax is not None:
        im= ax.imshow(pass_map.T, cmap= 'gray', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Difference Teammates / Opponents", orientation="vertical", ax= ax)
    return pass_map

#MAP PERMETTANT DE SAVOIR LES ZONES D'INTERCEPTION DE BALLE 
    #Pour Defense car calcul d'interception sur le segment Porteur - Target Position
    #On ne prend pas en compte la Vitesse Courrante de la Balle à Target Position
def compute_intercept_map(robot_map, ball_map, robots_pos, 
                          robots_current_speeds, robots_max_speeds, 
                          x, y, ball_init_speed= None, ax= None):
    #Detection des Interceptions Triviales | Permet de réduire le temps de calcul
    intercept_map= robot_map - ball_map
    #Récupération des positions pour lesquelles le robot est plus lent que la balle pour atteindre la position
    cols, rows= np.where(intercept_map > 0.0)
    #On va check que le robot ne peut pas intercepter le ballon sur le segment Porteur - Target Position
    #Detection des interceptions en chemin vers target_pos |Interceptions non Triviales
    for col in set(cols):
        for row in set(rows):
            map_value= 0.0
            target_pos= [x[col], y[row]]
            stop_loc = target_pos
            pass_speed= ball_init_speed
            #SI on a spécifier une vitesse de passe | Calcul de position d'arrêt de la balle pour vérifier si la balle pourra atteindre target_pos
            if pass_speed is not None:
                theta= math.atan2(target_pos[1] - carrier[1], target_pos[0] - carrier[0])
                _, stop_loc= utils.get_stop_ball(pass_speed, carrier, theta, cf= 0.3)
            #SINON | Calcul de vitesse de passe pour que V_balle = 0 au niveau de target_pos
            else: 
                pass_speed = utils.compute_ball_v0(abs(utils.distance(carrier, target_pos)))
            #SI la balle est arrêté au minimum à target_pos | LA PASSE N'EST PAS IMPOSSIBLE
            if abs(utils.distance(carrier, stop_loc)) >= abs(utils.distance(carrier, target_pos)):
                robot_intercept_time= math.inf
                robot_time= math.inf
                ball_intercept_time= math.inf
                ball_time= math.inf
                #Calcul du temps pour que le robot le plus proche atteigne la position d'interception du segment Porteur de balle - Target Position
                for i, robot in enumerate(robots_pos):
                    closest_pt= utils.find_closest_point_on_segment(robot, carrier, target_pos)
                    time_= utils.compute_time_to_position(robot, robots_current_speeds[i], robots_max_speeds[i], closest_pt)
                    if time_ < robot_intercept_time:
                        robot_intercept_time= time_
                        robot_time= utils.compute_time_to_position(robot, robots_current_speeds[i], robots_max_speeds[i], target_pos)
                        ball_intercept_time = utils.time_ball_reach_pos(pass_speed, carrier, closest_pt, cf= 0.3)
                        ball_time = utils.time_ball_reach_pos(pass_speed, carrier, target_pos, cf= 0.3)
                if (robot_intercept_time < ball_intercept_time):
                    map_value= robot_intercept_time - ball_intercept_time 
            intercept_map[col, row] = map_value
    if ax is not None:
        im= ax.imshow(intercept_map.T, cmap= 'gray', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="", orientation="vertical", ax= ax)
    return intercept_map

if __name__ == "__main__":
    field = np.array([[utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2], 
                        [utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                        [-utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                        [-utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2]], dtype= float)
    granul= 0.1
    y= np.arange(-utils.FIELD_HEIGHT/2 + granul/2, (utils.FIELD_HEIGHT + granul)/2 - granul/2, granul)
    x= np.arange(-utils.FIELD_WIDTH/2 + granul/2, (utils.FIELD_WIDTH + granul)/2 - granul/2, granul)
    """carrier= np.array([0, -5], dtype= float)
    tms= np.array([[0, 5]], dtype= float)
    ops= np.array([[0, 0]], dtype= float)
    tms_init_speed= np.array([0], dtype= float)
    tms_max_speed= np.array([2.5])
    ops_init_speed= np.array([0], dtype= float)
    ops_max_speed= np.array([2.5], dtype= float)"""
    carrier = np.array([0, -5], dtype= float)
    tms= np.array([[-4, 0], [-3, 6], [3, -2], [10, 0]], dtype= float)
    ops= np.array([[-2, -5], [2, -5], [0, -3], [-7, 0], [-10, 0]], dtype= float)
    tms_init_speed= np.array([0, 0, 0, 0], dtype= float)
    tms_max_speed= np.array([2.5, 2.5, 2.5, 1])
    ops_init_speed= np.array([0, 0, 0, 0, 0], dtype= float)
    ops_max_speed= np.array([2.5, 2.5, 2.5, 2.5, 1], dtype= float)
    pass_speed= None
    figure= plt.figure()
    (fig1, fig2)= figure.subfigures(2,1)
    ax= fig1.subplots(1,1)
    ax2= fig2.subplots(2,3)
    #fig,ax = plt.subplots(6,1)
    p = Polygon(field, color= (0, 0.7, 0, 0.4))
    ax.add_patch(p)
    ax.set_xlim([-(utils.FIELD_WIDTH/2) + 0.25,(utils.FIELD_WIDTH/2) + 0.25])
    ax.set_ylim([-(utils.FIELD_HEIGHT/2) + 0.25, (utils.FIELD_HEIGHT/2) + 0.25])
    ax.scatter([carrier[0]], [carrier[1]], marker='o', color= "black", label= "Carrier")
    ax.scatter(tms[:,0], tms[:,1], marker= "o", color= "purple", label= "Teammates")
    ax.scatter(ops[:,0], ops[:,1], marker= "o", color= "orange", label= "Opponents")
    ax.set_title("Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speed) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speed) + "m/s\n Version ANALYTIQUE")
    ax.legend()
    print("> Teammates Time Map")
    tm_map= compute_control_map(tms, tms_init_speed, tms_max_speed, x, y, ax= ax2[0,0])
    ax2[0,0].set_title("Temps deplacement Teammates")
    print("> Opponents Time Map")
    ops_map= compute_control_map(ops, ops_init_speed, ops_max_speed, x, y, ax= ax2[0,1])
    ax2[0,1].set_title("Temps deplacement Opponents")
    print("> Ball Time Map")
    ball_map= compute_ball_map(carrier, x, y, pass_init_speed= pass_speed, cf= 0.3, ax= ax2[0,2])
    ax2[0,2].set_title("Temps deplacement Balle")
    print("> Teammates - Opponents Control Map")
    teams_control_map= compute_team_controlled_map(tm_map, ops_map, ax= ax2[1,0])
    ax2[1,0].set_title("Diff temp Teammates - Opponents")
    ops_intercept_map= compute_intercept_map(ops_map, ball_map, ops, ops_init_speed, ops_max_speed, x, y, ball_init_speed= pass_speed, ax= None)
    
    tms_intercept_map= compute_intercept_map(tm_map, ball_map, tms, tms_init_speed, tms_max_speed, x, y, ball_init_speed= pass_speed, ax= ax2[1,1])
    ax2[1,1].set_title("Teammates Intercept Map")
    print("> Teammates Pass Zones")
    priority_map= compute_intercept_priorities(x, y, carrier, tms, ops, 
                                                tms_init_speed, tms_max_speed, 
                                                ops_init_speed, ops_max_speed, ax= None)
    pass_map= compute_pass_map(teams_control_map, tms_intercept_map,ops_intercept_map, priority_map, ax= ax2[1,2], cf= 0.3)
    ax2[1,2].set_title("Teammates Pass Map")
    """pass_tm_map= compute_intercept_map(tm_map, ball_map, ax= ax2[1,2])
    ax2[1,2].set_title("Diff temp Teammates - Balle")"""
    """pass_ops_map= compute_intercept_map(ops_map, ball_map, ax= ax2[1,1])
    ax2[1,1].set_title("Diff temp Opponents - Balle")"""
    """tm_zone_pass_map= compute_intercept_map(tm_map, ball_map, ax2[1,1])
    ax2[1,1].set_title("Diff temps Teammates - Balle")
    final_control_map= compute_final_control_map(teams_control_map, tm_zone_pass_map, carrier, tms, ops, 
                                                    tms_init_speed, tms_max_speed,
                                                    ops_init_speed, ops_max_speed, ax= ax2[1,2])
    ax2[1,2].set_title("Heatmap Zone Pass Availables")"""
    plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.45)
    plt.show()
