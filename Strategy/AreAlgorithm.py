import sys
import os
# Get the current script's directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory by going one level up
parent_dir = os.path.dirname(current_dir)
# Add the parent directory to sys.path
sys.path.append(parent_dir)

import NaiveAlgorithm as NA
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from copy import deepcopy
from time import perf_counter
import utils
from queue import Queue
GRAVITY= 9.81
FIELD_WIDTH= 22
FIELD_HEIGHT= 14

def get_valid_neighbours(pos, pass_possibles, pass_step):
    neighbours= []
    for i in range(-1, 2):
        n1= [pos[0] +i*pass_step,pos[1] + pass_step]
        n2= [pos[0] +i*pass_step,pos[1] - pass_step]
        if n1 not in pass_possibles and abs(n1[0]) <= FIELD_WIDTH/2 and abs(n1[1]) <= FIELD_HEIGHT/2:
            neighbours.append(n1)
        if n2 not in pass_possibles and abs(n2[0]) <= FIELD_WIDTH/2 and abs(n2[1]) <= FIELD_HEIGHT/2:
            neighbours.append(n2)
    n7 = [pos[0] + pass_step, pos[1]]
    if n7 not in pass_possibles and abs(n7[0]) <= FIELD_WIDTH/2 and abs(n7[1]) <= FIELD_HEIGHT/2:
        neighbours.append(n7)
    n8 = [pos[0] - pass_step, pos[1]]
    if n8 not in pass_possibles and abs(n8[0]) <= FIELD_WIDTH/2 and abs(n8[1]) <= FIELD_HEIGHT/2:
        neighbours.append(n8)
    return neighbours

#VERSION OPTIMISEE DE LA HEATMAP (OPTIMISATION DES POSITIONS VISITEES Avec Des composantes A*)
def compute_possibles_pass_area(carrier_pos, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_step= 0.25, cf= 0.3):
    pass_possibles= []
    pass_no= []
    buffer_positions= Queue()
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = utils.modulo_2Pi(theta_direct + math.pi)
        dist_pass= abs(utils.distance(tm, carrier_pos))
        ball_init_speed = utils.compute_ball_v0(dist_pass)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible, real_intercept= NA.is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, tm, ball_init_speed, cf= cf)
        if pass_possible == True:
                if real_intercept is not None and real_intercept not in pass_possibles:
                    buffer_positions.put(real_intercept)
                    pass_possibles.append(real_intercept)
                elif tm not in pass_possibles:
                    buffer_positions.put(tm)
                    pass_possibles.append(deepcopy(tm.tolist()))
        while buffer_positions.qsize() >0:
            t= buffer_positions.qsize()
            pos= buffer_positions.get()
            a= buffer_positions.qsize()
            neighbours= get_valid_neighbours(pos, pass_possibles, pass_step)
            for n in neighbours:
                dist_pass= abs(utils.distance(n, carrier_pos))
                ball_init_speed = utils.compute_ball_v0(dist_pass)
                pass_possible, real_intercept= NA.is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, n, ball_init_speed, cf= cf)
                if pass_possible == True and abs(utils.distance(n, tm)) < abs(utils.distance(n, carrier_pos)):
                    if real_intercept is not None and real_intercept not in pass_possibles:
                        buffer_positions.put(real_intercept)
                        pass_possibles.append(real_intercept)
                    elif n not in pass_possibles:
                        buffer_positions.put(n)
                        pass_possibles.append(deepcopy(n))
                else:
                    pass_no.append(n)
    return np.array(pass_possibles), None, np.array(pass_no), None
        
if __name__ == "__main__":
    #Dimensions du terrain
    field = np.array([[FIELD_WIDTH/2, -FIELD_HEIGHT/2], 
                        [FIELD_WIDTH/2, FIELD_HEIGHT/2], 
                        [-FIELD_WIDTH/2, FIELD_HEIGHT/2], 
                        [-FIELD_WIDTH/2, -FIELD_HEIGHT/2]], dtype= float)
    carrier = np.array([0, -5], dtype= float)
    tms= np.array([[0, 5]], dtype= float)
    ops= np.array([[2, 0], [-2, 0]], dtype= float)
    tms_init_speed= np.array([0], dtype= float)
    tms_max_speed= np.array([2.5])
    ops_init_speeds= np.array([0, 0], dtype= float)
    ops_max_speeds= np.array([2.5, 2.5], dtype= float)

    fig,ax = plt.subplots(2,1)
    """plot_situation_problem(ax[0], compute_possibles_pass_it, field, carrier, tms, ops, "Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s\n Version ITERATIVE",
                            tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)"""
    NA.plot_situation_problem(ax[0], compute_possibles_pass_area, field, carrier, tms, ops, "Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s\n Version A*",
                            tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)
    NA.plot_situation_problem(ax[1], NA.compute_possibles_pass_a, field, carrier, tms, ops, "Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s\n Version ANALYTIQUE",
                            tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)
    """NA.pass_zone_validator(ax[1], NA.is_pass_possible_a, field, carrier, tms, ops, "Validation des zones de passes possibles\n vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s",
                                0.25, 0.3, tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)"""
    """plot_situation_problem(ax[1], compute_possibles_pass_block, field, carrier, tms, ops, "VERSION ANALYTIQUE Avec positions de blocages de l'Adversaire\n vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s",
                            tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)"""
    
    plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.55)
    plt.show()