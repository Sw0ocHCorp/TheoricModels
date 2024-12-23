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

#MAP Des Zones Contrôlées par une Team sur le Terrain
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

#MAP du temps de déplacement de la balle sur le terrain
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

#MAP des zones contrôlées par les 2 équipes
def compute_team_controlled_map(teammate_map, ops_map, ax= None):
    start= perf_counter()
    diff= teammate_map - ops_map
    print("Time Elapsed: ", perf_counter() - start)
    if ax is not None:
        im= ax.imshow(diff.T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Difference Teammates / Opponents", orientation="vertical", ax= ax)
    return diff

#MAP des priorités d'interception de balle entre les 2 équipes 
    #l'équipe prioritaire est celle qui se trouve entre le porteur de balle et l'autre équipe -> susceptible d'intercepter le ballon en premier
def compute_intercept_priorities(x, y , carrier, tms, ops, tms_current_speeds, tms_max_speeds, ops_current_speeds, ops_max_speeds, ax= None):
    #A revoir, résultats douteux
    #Potentiellement inutile quand la vitesse sera changée en vecteur
    priority_map= np.zeros((x.shape[0], y.shape[0]), dtype= float)
    for col, xi in enumerate(x):
        for row, yi in enumerate(y):
            target_pos= [xi, yi]
            dist_passe= abs(utils.distance(carrier, target_pos))
            angle_passe= math.atan2(target_pos[1] - carrier[1], target_pos[0] - carrier[0])
            bounds= [utils.modulo_2Pi(angle_passe - math.pi/2 + math.pi/5), utils.modulo_2Pi(angle_passe + math.pi/2 - math.pi/5)]
            #Récupération du teammate le plus proche du point de passe
            #/!\ Le Teammate doir se trouver dans les 180° centrée sur l'axe Porteur de balle - Target Position
            tm_intercept_dist= math.inf
            tm_dist= math.inf
            for tm in tms:
                intercept= utils.find_closest_point_on_segment(tm, carrier, target_pos)
                d= abs(utils.distance(carrier, intercept))
                c_tm_angle= math.atan2(tm[1] - carrier[1], tm[0] - carrier[0])
                if d < tm_intercept_dist and d  > 0.1 and c_tm_angle > bounds[0] and c_tm_angle < bounds[1]:
                    tm_intercept_dist= d
                    tm_dist = abs(utils.distance(tm, carrier))
            #Récupération de l'adversaire le plus proche du point de passe
            #/!\ L'Adversaire doir se trouver dans les 180° centrée sur l'axe Porteur de balle - Target Position
            ops_intercept_dist= math.inf
            ops_dist= math.inf
            for opponent in ops:
                intercept= utils.find_closest_point_on_segment(opponent, carrier, target_pos)
                d= abs(utils.distance(carrier, intercept))
                c_ops_angle= math.atan2(opponent[1] - carrier[1], opponent[0] - carrier[0])
                if d < ops_intercept_dist and d  > 0.1 and c_ops_angle > bounds[0] and c_ops_angle < bounds[1]:
                    ops_intercept_dist= d
                    ops_dist = abs(utils.distance(opponent, carrier))
            if ops_dist <= tm_dist: 
                priority_map[col, row] = utils.OPPONENTS_PRIORITY
            else:
                v= 1
    if ax is not None:
        im= ax.imshow(priority_map.T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Intercept Priorities Teammates / Opponents", orientation="vertical", ax= ax)
    return priority_map
            
#MAP des opportunités d'interception de balle
    #Defense: calcul d'interception sur le segment Porteur - Target Position
    #Attaque: calcul des zones de passes possibles
def compute_intercept_map(robot_map, ball_map, robots_pos, 
                          robots_current_speeds, robots_max_speeds, 
                          x, y, granul, time_reduction= 0.0, ball_init_speed= None, ax= None):
    #Detection des Interceptions Triviales | Permet de réduire le temps de calcul
    intercept_map= robot_map - ball_map
    intercept_map[intercept_map == -math.inf] = 0.0
    #Récupération des positions pour lesquelles le robot est plus lent que la balle pour atteindre la position
    indexs= np.where(intercept_map > 0.0)
    #On va check que le robot ne peut pas intercepter le ballon sur le segment Porteur - Target Position
    #Detection des interceptions en chemin vers target_pos |Interceptions non Triviales
    for col, row in zip(indexs[0], indexs[1]):
        #On considère la passe à target_pos comme impossible de base
            #On va prouver que la passe est faisable pour changer la valeur de la case
        map_value= 0.0
        target_pos= [x[col], y[row]]
        pass_speed= ball_init_speed
        #SI on a spécifier une vitesse de passe | Calcul de position d'arrêt de la balle pour vérifier si la balle pourra atteindre target_pos
        if pass_speed is None:
            pass_speed = utils.compute_ball_v0(abs(utils.distance(carrier, target_pos)))
        theta= math.atan2(target_pos[1] - carrier[1], target_pos[0] - carrier[0])
        stop_time, stop_pos= utils.get_stop_ball(pass_speed, carrier, theta, cf= 0.3)
        #SI la balle s'arrête après target_pos | LA PASSE N'EST PAS IMPOSSIBLE
        if abs(utils.distance(carrier, stop_pos)) >= abs(utils.distance(carrier, target_pos))- granul:
            robot_intercept_time= math.inf
            robot_time= math.inf
            ball_intercept_time= math.inf
            ball_time= math.inf
            robot_intercept= None
            #Calcul du temps pour que le robot le plus proche atteigne la position d'interception du segment Porteur de balle - Target Position
            for i, robot in enumerate(robots_pos):
                closest_pt= utils.find_closest_point_on_segment(robot, carrier, target_pos)
                time_= utils.compute_time_to_position(robot, robots_current_speeds[i], robots_max_speeds[i], closest_pt)
                if time_ < robot_intercept_time:
                    robot_intercept= closest_pt
                    robot_intercept_time= time_
                    robot_time= utils.compute_time_to_position(robot, robots_current_speeds[i], robots_max_speeds[i], target_pos)
                    ball_intercept_time = utils.time_ball_reach_pos(pass_speed, carrier, closest_pt, cf= 0.3)
                    ball_time = utils.time_ball_reach_pos(pass_speed, carrier, target_pos, cf= 0.3)
            #SI le robot le plus proche peut intercepter la balle avant qu'elle n'atteigne target_pos -> On remplit la case avec la différence temporelle entre robot et balle
            if robot_intercept_time < max(0,ball_intercept_time - time_reduction):
                map_value= robot_intercept_time - ball_intercept_time 
            #SINON SI la Team du Robot ne peut pas intercepter le ballon en chemin la balle MAIS elle s'arrête à target_pos | On va toujours pouvoir l'intercepter à target_pos
                #On marque cet situation comme -math.inf pour rendre l'algo compréhensible (on s'occupera de ce cas dans la construction de la MAP de passe)
            elif abs(abs(utils.distance(carrier, stop_pos)) - abs(utils.distance(carrier, target_pos))) < granul:
                map_value= -math.inf
        intercept_map[col, row] = map_value
    if ax is not None:
        im= ax.imshow(intercept_map.T, cmap= 'gray', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Difference Robots / Ball", orientation="vertical", ax= ax)
    return intercept_map


#CALCUL des zones de passes réelle pour l'équipe porteuse de balle (en attaque)
def compute_pass_map(control_map, tm_map, ops_map, tm_intercept_map, ops_intercept_map, priority_map, ax= None, cf= 0.3):
    #On va uniquement travailler dans la zone conquise par l'équipe en attaque
    start= perf_counter()
    pass_map= deepcopy(control_map)
    pass_map[pass_map > 0.0]= 0.0
    pass_map[pass_map == -math.inf] = 0.0
    pass_map[pass_map < 0.0]= 1.0
    indexs= np.where(pass_map > 0.0)
    for col, row in zip(indexs[0], indexs[1]):
        diff_ops= ops_intercept_map[col, row]
        diff_tm= tm_intercept_map[col, row]
        intercept_priority= priority_map[col, row] 
        #SI la team n'a pa pu intercepter la balle en chemin mais que la balle s'est arrêtée à la position testée | La balle sera interceptée par l'équipe la plus rapide pour atteindre la position testée
        if diff_ops == -math.inf:
            diff_ops= min(ops_map[col, row] - tm_map[col, row], 0.0)
        if diff_tm == -math.inf:
            diff_tm= min(tm_map[col, row] - ops_map[col, row], 0.0)
        #SI Personne ne peut intercepter la balle | PASSE IMPOSSIBLE
        if diff_tm == 0.0 and diff_ops == 0.0:
            pass_map[col, row]*= 0.0
        #SI notre Team est prioritaire et qu'on peut intercepter le ballon
        #OU Qu'on est pas prioritaire mais qu'on peut intercepter le ballon alors que les adversaires NON | PASSE POSSIBLE
        elif (intercept_priority == utils.TEAMMATES_PRIORITY and diff_tm < 0.0) or \
                (intercept_priority == utils.OPPONENTS_PRIORITY and diff_tm < 0.0 and diff_ops == 0.0):
            pass_map[col, row]*= -(abs(diff_tm) - abs(diff_ops))
        #SINON SI les Adversaires sont prioritaires et peuvent intercepter la balle
        #OU Que les Adversaires ne sont pas prioritaires mais qu'ils peuvent intercepter le ballon alors que notre Team NON | PASSE IMPOSSIBLE
        elif (intercept_priority == utils.OPPONENTS_PRIORITY and diff_ops < 0.0) or \
                (intercept_priority == utils.TEAMMATES_PRIORITY and diff_ops < 0.0 and diff_tm == 0.0):
            pass_map[col, row]*= 0.0
        #VOIR DANS QUEL CAS ON EST DANS LE else
        #S'IL Y A DES CAS NON ANTICIPES
        else:
            pass_map[col, row]*= 0.0
            
    if ax is not None:
        im= ax.imshow(pass_map.T, cmap= 'gray', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Difference Teammates / Opponents", orientation="vertical", ax= ax)
    return pass_map

#MAP des zones de shoot possibles pour l'équipe porteuse de balle
def compute_shoot_map(pass_map, carrier, ops, ops_current_speeds, ops_max_speeds, x, y, shooting_speed= None, n_targets= 5, cf= 0.3, ax= None):
    #TEAMMATES
    #On va travailler dans les zones de passes possibles
    shoot_map= deepcopy(pass_map)
    indexs= np.where(shoot_map < 0.0)
    targets= np.arange(-1, 1 + (2/(n_targets-1)), 2/(n_targets-1))
    for col, row in zip(indexs[0], indexs[1]):
        shoot_map[col, row] = 0
        shoot_pos= [x[col], y[row]]
        diff_ball_ops= -1
        for i, target_y in enumerate(targets):
            target= [-11, target_y]
            if shooting_speed is None:
                shooting_speed= utils.compute_ball_v0(33, cf= cf)
            time_ball= utils.time_ball_reach_pos(shooting_speed, shoot_pos, target, cf= cf)
            time_ops= math.inf
            for j, op in enumerate(ops):
                intercept= utils.find_closest_point_on_segment(op, shoot_pos, target)
                time_= utils.compute_time_to_position(op, ops_current_speeds[j], ops_max_speeds[j], intercept)
                if time_ < time_ops:
                    time_ops= time_
            if time_ball < time_ops and time_ops - time_ball > diff_ball_ops:
                diff_ball_ops = time_ops - time_ball 
                shoot_map[col, row]= i+1
    #CARRIER         
    sorted_x= sorted(x, key= lambda pos: abs(carrier[0] - pos))
    sorted_y= sorted(y, key= lambda pos: abs(carrier[1] - pos))
    for i, target_y in enumerate(targets):
        target= [-11, target_y]
        if shooting_speed is None:
            shooting_speed= utils.compute_ball_v0(abs(utils.distance(carrier, target))*2, cf= cf)
        time_ball= utils.time_ball_reach_pos(shooting_speed, carrier, target, cf= cf)
        time_ops= math.inf
        for j, op in enumerate(ops):
            intercept= utils.find_closest_point_on_segment(op, carrier, target)
            time_= utils.compute_time_to_position(op, ops_current_speeds[j], ops_max_speeds[j], intercept)
            if time_ < time_ops:
                time_ops= time_
        if time_ball < time_ops:
            row= np.where(y == sorted_y[0])[0]
            col= np.where(x == sorted_x[0])[0]
            shoot_map[col, row]= i+1
            break
    if ax is not None:
        im= ax.imshow(shoot_map.T, origin = "lower")
        plt.colorbar(im, label="Target For Shoot", orientation="vertical", ax= ax)
    return shoot_map

if __name__ == "__main__":
    field = np.array([[utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2], 
                        [utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                        [-utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                        [-utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2]], dtype= float)
    granul= 0.1
    y= np.arange(-utils.FIELD_HEIGHT/2 + granul/2, (utils.FIELD_HEIGHT + granul)/2 - granul/2, granul)
    x= np.arange(-utils.FIELD_WIDTH/2 + granul/2, (utils.FIELD_WIDTH + granul)/2 - granul/2, granul)
    carrier= np.array([0, -5], dtype= float)
    tms= np.array([[0, 3], [10,0]], dtype= float)
    ops= np.array([[0, 0], [-10, 0]], dtype= float)
    tms_init_speed= np.array([0,0], dtype= float)
    tms_max_speed= np.array([2.5, 2.5])
    ops_init_speed= np.array([0, 0], dtype= float)
    ops_max_speed= np.array([2.5, 2.5], dtype= float)
    """carrier = np.array([0, -5], dtype= float)
    tms= np.array([[-4, 0], [-3, 6], [3, -2], [10, 0]], dtype= float)
    ops= np.array([[-2, -5], [2, -5], [0, -3], [-7, 0], [-10, 0]], dtype= float)
    tms_init_speed= np.array([0, 0, 0, 0], dtype= float)
    tms_max_speed= np.array([2.5, 2.5, 2.5, 1])
    ops_init_speed= np.array([0, 0, 0, 0, 0], dtype= float)
    ops_max_speed= np.array([2.5, 2.5, 2.5, 2.5, 1], dtype= float)"""
    pass_speed= 10
    cf= 0.3
    figure= plt.figure()
    (fig1, fig2)= figure.subfigures(2,1)
    ax= fig1.subplots(1,1)
    ax2= fig2.subplots(2,4)
    #fig,ax = plt.subplots(6,1)
    p = Polygon(field, color= (0, 0.7, 0, 0.4))
    ax.add_patch(p)
    ax.set_xlim([-(utils.FIELD_WIDTH/2) + 0.25,(utils.FIELD_WIDTH/2) + 0.25])
    ax.set_ylim([-(utils.FIELD_HEIGHT/2) + 0.25, (utils.FIELD_HEIGHT/2) + 0.25])
    ax.scatter([carrier[0]], [carrier[1]], marker='o', color= "black", label= "Carrier")
    ax.scatter(tms[:,0], tms[:,1], marker= "o", color= "purple", label= "Teammates")
    ax.scatter(ops[:,0], ops[:,1], marker= "o", color= "orange", label= "Opponents")
    if pass_speed is not None:
        ax.set_title("Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speed) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speed) + "m/s\n Vitesse de passes= " + str(pass_speed) + "m/s")
    else:
        ax.set_title("Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speed) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speed) + "m/s\n Vitesse de passes dynamiques")
    ax.legend()
    print("> Teammates Time Map")
    tm_map= compute_control_map(tms, tms_init_speed, tms_max_speed, x, y, ax= ax2[0,0])
    ax2[0,0].set_title("Temps deplacement Teammates")
    print("> Opponents Time Map")
    ops_map= compute_control_map(ops, ops_init_speed, ops_max_speed, x, y, ax= ax2[0,1])
    ax2[0,1].set_title("Temps deplacement Opponents")
    print("> Ball Time Map")
    ball_map= compute_ball_map(carrier, x, y, pass_init_speed= pass_speed, cf= cf, ax= ax2[0,2])
    ax2[0,2].set_title("Temps deplacement Balle")
    print("> Teammates - Opponents Control Map")
    priority_map= compute_intercept_priorities(x, y, carrier, tms, ops, 
                                                tms_init_speed, tms_max_speed, 
                                                ops_init_speed, ops_max_speed)
    #ax2[0,3].set_title("Intercept Priorities")
    teams_control_map= compute_team_controlled_map(tm_map, ops_map, ax= ax2[0,3])
    ax2[0,3].set_title("Diff temp Teammates - Opponents")   
    #ax2[1,0].set_title("Diff temp Teammates - Opponents")   
    tms_intercept_map= compute_intercept_map(tm_map, ball_map, tms, tms_init_speed, tms_max_speed, x, y, granul, time_reduction= 0.1, ball_init_speed= pass_speed, ax= ax2[1,0])
    ax2[1,0].set_title("Teammates Intercept Map")
    #ax2[1,1].set_title("Teammates Intercept Map")
    print("> Teammates Pass Zones")
    ops_intercept_map= compute_intercept_map(ops_map, ball_map, ops, ops_init_speed, ops_max_speed, x, y, granul, ball_init_speed= pass_speed, ax= ax2[1,1])
    ax2[1,1].set_title("Opponents Intercept Map")
    #ax2[1,2].set_title("Opponents Intercept Map")
    pass_map= compute_pass_map(teams_control_map, tm_map, ops_map, tms_intercept_map,ops_intercept_map, priority_map, ax= ax2[1,2], cf= cf)
    ax2[1,2].set_title("Teammates Pass Map")
    #ax2[1,3].set_title("Teammates Pass Map")
    shooting_map= compute_shoot_map(pass_map, carrier, ops, ops_init_speed, ops_max_speed, x, y, shooting_speed= None, n_targets= 5, cf= 0.3, ax= ax2[1,3])
    ax2[1,3].set_title("Team Shooting Map")
    plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.85, wspace=0.3, hspace=0.45)
    plt.show()
