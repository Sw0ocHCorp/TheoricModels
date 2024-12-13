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
def compute_control_map(robots, robot_init_speeds, robot_max_speeds, field_dim, ax= None, x_step= 0.5, y_step= 0.5):
    start= perf_counter()
    rows= np.arange(-utils.FIELD_HEIGHT/2 + y_step/2, (utils.FIELD_HEIGHT + y_step)/2 - y_step/2, y_step)
    cols= np.arange(-utils.FIELD_WIDTH/2 + x_step/2, (utils.FIELD_WIDTH + x_step)/2 - x_step/2, x_step)
    robots_map= np.zeros((cols.shape[0], rows.shape[0]), dtype= float)
    for r in range(robots_map.shape[1]):
        for c in range(robots_map.shape[0]):
            target_pos= [cols[c], rows[r]]
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
def compute_ball_map(ball_init_pos, field_dim, ax= None, pass_init_speed= None, x_step= 0.5, y_step= 0.5, cf= 0.3):
    rows= np.arange(-utils.FIELD_HEIGHT/2 + y_step/2, (utils.FIELD_HEIGHT + y_step)/2 - y_step/2, y_step)
    cols= np.arange(-utils.FIELD_WIDTH/2 + x_step/2, (utils.FIELD_WIDTH + x_step)/2 - x_step/2, x_step)
    start= perf_counter()
    ball_map= np.zeros((cols.shape[0], rows.shape[0]), dtype= float)
    for r in range(ball_map.shape[1]):
        for c in range(ball_map.shape[0]):
            target_pos= [cols[c], rows[r]]
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
def compute_team_control_map(teammate_map, ops_map, ax= None):
    start= perf_counter()
    diff= teammate_map - ops_map
    print("Time Elapsed: ", perf_counter() - start)
    if ax is not None:
        im= ax.imshow(diff.T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Difference Teammates / Opponents", orientation="vertical", ax= ax)
    return diff

#AFFINAGE DES ZONES DE CONTROLE DE LA TEAM ROBOT EN FONCTION DES PASSES REALISABLES
def fine_tune_control_map(control_map, field_dim, ball_pos,tm_pos, ops_pos, 
                            tm_init_speeds, tm_max_speeds, ops_init_speeds, ops_max_speeds, 
                            ball_init_speed= None, ax= None, x_step= 0.5, y_step= 0.5, cf= 0.3):
    #Points de la zone de controle des teammates
    y= np.arange(-utils.FIELD_HEIGHT/2 + y_step/2, (utils.FIELD_HEIGHT + y_step)/2 - y_step/2, y_step)
    x= np.arange(-utils.FIELD_WIDTH/2 + x_step/2, (utils.FIELD_WIDTH + x_step)/2 - x_step/2, x_step)
    start= perf_counter()
    cols, rows= np.where(control_map <= 0.0)
    fine_tuned_map= deepcopy(control_map)
    fine_tuned_map[fine_tuned_map > 0.0]= 0.0
    for r in set(rows):
        for c in set(cols):
            pass_speed= ball_init_speed
            target_pos= [x[c], y[r]]
            diff= fine_tuned_map[c, r]
            if ball_init_speed is None:
                pass_speed= utils.compute_ball_v0(abs(utils.distance(ball_pos, target_pos)))
            #Temps pour que la balle atteigne la position d'interception du teammate le plus proche du segment Porteur de balle - Target Position
            ball_tm_time= 0.0
            #Temps pour que le teammate le plus proche atteigne la position d'interception du segment Porteur de balle - Target Position
            tm_time= math.inf
            for i, tm in enumerate(tm_pos):
                intercept= utils.find_closest_point_on_segment(tm, ball_pos, target_pos)
                if (utils.compute_time_to_position(tm, tm_init_speeds[i], tm_max_speeds[i], intercept) < tm_time):
                    #Calcul du temps pour que la balle atteigne la position d'interception
                    ball_tm_time= utils.time_ball_reach_pos(pass_speed, ball_pos, intercept, cf)
                    tm_time= utils.compute_time_to_position(tm, tm_init_speeds[i], tm_max_speeds[i], intercept)
            #Temps pour que l'Adversaire le plus proche atteigne la position d'interception du segment Porteur de balle - Target Position
            ops_time= math.inf
            #Temps pour que la balle atteigne la position d'interception de l'Adversaire le plus proche du segment Porteur de balle - Target Position
            ball_ops_time= 0.0
            for i, ops in enumerate(ops_pos):
                intercept= utils.find_closest_point_on_segment(ops, ball_pos, target_pos)
                if (utils.compute_time_to_position(ops, ops_init_speeds[i], ops_max_speeds[i], intercept) < ops_time):
                    ops_time= utils.compute_time_to_position(ops, ops_init_speeds[i], ops_max_speeds[i], intercept)
                    #Calcul du temps pour que la balle atteigne la position d'interception
                    ball_ops_time= utils.time_ball_reach_pos(pass_speed, ball_pos, intercept, cf)
            if (ops_time <= ball_ops_time):
                fine_tuned_map[c, r]= 0.0
    print("Time Elapsed: ", perf_counter() - start)
    if ax is not None:
        im= ax.imshow(fine_tuned_map.T, cmap= 'gray', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="Time Difference Teammates / Opponents Fine Tuned", orientation="vertical", ax= ax)
    return fine_tuned_map

#MAP PERMETTANT DE SAVOIR LES ZONES D'INTERCEPTION DE BALLE (Pour passe ou Defense)
def compute_intercept_map(robot_map, ball_map, ax= None):
    intercept_map= robot_map - ball_map
    #Si Le robot le plus proche met plus de temps que le ballon pour atteindre la position, Interception impossible | 0
    intercept_map[intercept_map > 0.0]= 0.0
    if ax is not None:
        im= ax.imshow(intercept_map.T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="", orientation="vertical", ax= ax)
    return intercept_map

#MAP FINALE PERMETTANT DE SAVOIR LES ZONES DE PASS POSSIBLES
def compute_final_control_map(diff_map, intercept_map, carrier, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speed, ops_max_speed, ax= None):
    relu_diff= deepcopy(diff_map)
    relu_diff[diff_map > 0.0]= 0.0
    intercept_map= deepcopy(intercept_map)
    for c in range(relu_diff.shape[0]):
        for r in range(relu_diff.shape[1]):
            tm_time= math.inf
            for i, tm in enumerate(tm_pos):
                closest_pt= utils.find_closest_point_on_segment(tm, carrier, [c, r])
                if (utils.compute_time_to_position(tm, tm_init_speed[i], tm_max_speed[i], closest_pt) < tm_time):
                    tm_time= utils.compute_time_to_position(tm, tm_init_speed[i], tm_max_speed[i], closest_pt)
            ops_time= math.inf
            for i, ops in enumerate(ops_pos):
                closest_pt= utils.find_closest_point_on_segment(ops, carrier, [c, r])
                if (utils.compute_time_to_position(ops, ops_init_speed[i], ops_max_speed[i], closest_pt) < ops_time):
                    ops_time= utils.compute_time_to_position(ops, ops_init_speed[i], ops_max_speed[i], closest_pt)
            if (ops_time < tm_time):
                relu_diff[c, r]= 0.0
    final_map= relu_diff * intercept_map
    if ax is not None:
        im= ax.imshow(final_map.T, cmap= 'bwr', interpolation= 'nearest', origin = "lower")
        plt.colorbar(im, label="", orientation="vertical", ax= ax)
    return final_map

if __name__ == "__main__":
    field = np.array([[utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2], 
                        [utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                        [-utils.FIELD_WIDTH/2, utils.FIELD_HEIGHT/2], 
                        [-utils.FIELD_WIDTH/2, -utils.FIELD_HEIGHT/2]], dtype= float)
    granul= 0.1
    carrier = np.array([0, -5], dtype= float)
    tms= np.array([[-3, 5]], dtype= float)
    ops= np.array([[0, -3], [-2, -3]], dtype= float)
    tms_init_speed= np.array([0], dtype= float)
    tms_max_speed= np.array([2.5])
    ops_init_speed= np.array([0,0], dtype= float)
    ops_max_speed= np.array([2.5,2.5], dtype= float)
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
    ax.legend()
    print("> Teammates Time Map")
    tm_map= compute_control_map(tms, tms_init_speed, tms_max_speed, field, ax= ax2[0,0], x_step= granul, y_step= granul)
    ax2[0,0].set_title("Temps deplacement Teammates")
    print("> Opponents Time Map")
    ops_map= compute_control_map(ops, ops_init_speed, ops_max_speed, field, ax= ax2[0,1], x_step= granul, y_step= granul)
    ax2[0,1].set_title("Temps deplacement Opponents")
    print("> Ball Time Map")
    ball_map= compute_ball_map(carrier, field, pass_init_speed= None, x_step= granul, y_step= granul, cf= 0.3, ax= ax2[0,2])
    ax2[0,2].set_title("Temps deplacement Balle")
    print("> Teammates - Opponents Control Map")
    teams_control_map= compute_team_control_map(tm_map, ops_map, ax= ax2[1,0])
    ax2[1,0].set_title("Diff temp Teammates - Opponents")
    print("> Teammates Fine-Tuned Pass Zones")
    fine_tuned_map= fine_tune_control_map(teams_control_map, field, carrier, tms, ops, 
                                            tms_init_speed, tms_max_speed, ops_init_speed, ops_max_speed, ball_init_speed= None, 
                                            ax= ax2[1,1], x_step= granul, y_step= granul, cf= 0.3)
    ax2[1,1].set_title("Fine Tuned Control Map")
    """pass_tm_map= compute_intercept_map(tm_map, ball_map, ax= ax2[1,0])
    ax2[1,0].set_title("Diff temp Teammates - Balle")"""
    """pass_ops_map= compute_intercept_map(ops_map, ball_map, ax= ax2[1,1])
    ax2[1,1].set_title("Diff temp Opponents - Balle")"""
    """tm_zone_pass_map= compute_intercept_map(tm_map, ball_map, ax2[1,1])
    ax2[1,1].set_title("Diff temps Teammates - Balle")
    final_control_map= compute_final_control_map(teams_control_map, tm_zone_pass_map, carrier, tms, ops, 
                                                    tms_init_speed, tms_max_speed,
                                                    ops_init_speed, ops_max_speed, ax= ax2[1,2])
    ax2[1,2].set_title("Heatmap Zone Pass Availables")"""
    plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.55)
    plt.show()
