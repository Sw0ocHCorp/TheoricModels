import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from copy import deepcopy
from time import perf_counter

TEAMMATES_PRIORITY= 0.0
OPPONENTS_PRIORITY= 1.0
GRAVITY= 9.81
FIELD_WIDTH= 22
FIELD_HEIGHT= 14

class WayPoints:
    def __init__(self, x, y, speed, theta):
        self.x= x
        self.y= y
        self.speed= speed
        self.theta= theta

#Classe modélisant un Robot
class Robot:
    def __init__(self, current_location, current_theta, current_lin_speed, max_lin_speed, max_rot_speed):
        self.current_location= current_location
        self.current_theta= current_theta
        self.current_lin_speed= current_lin_speed
        self.max_lin_speed= max_lin_speed
        self.max_rot_speed= max_rot_speed
    
    def update_robot_state(self, target_waypoint: WayPoints):
        direct_theta= math.atan2(target_waypoint.y - self.current_location[1], target_waypoint.x - self.current_location[0])
        remain_rotation= modulo_2Pi(self.current_theta - direct_theta)


def distance(pt1, pt2):
     if (pt1 is not None and pt2 is not None):
         return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
     else:
         return math.inf

def compute_ball_dynamics(ball_init_pos, theta, ball_speed, cf= 0.3, dt= 1/50):
    vx= ball_speed*math.cos(theta)
    vy= ball_speed*math.sin(theta)
    ball_pos= deepcopy(ball_init_pos)
    pos_history= [deepcopy(ball_pos)]
    while(abs(vx) > 0.05 or abs(vy) > 0.05):
        ball_pos[0] += vx*dt
        ball_pos[1] += vy*dt
        ball_speed -= cf*GRAVITY*dt
        vx= ball_speed*math.cos(theta)
        vy= ball_speed*math.sin(theta)
        pos_history.append(deepcopy(ball_pos))
        if (vx < 0.05 and vy < 0.05):
            break
    return pos_history

# VERSION ANALYTIQUE (Calcul via Equation) | Beacoup plus rapide
# Balle s'arrête plus proche que la VERSION ITERATIVE
def time_ball_reach_pos(init_speed, start_pos, finish_pos, carrier_current_theta= None, carrier_rot_speed= None, cf=0.3):
    #VERIFICATION si la position voulue est atteingnable | Que la balle n'a pas le temps de s'arrêter en chemin
    move_dist= abs(distance(start_pos, finish_pos))
    deccel= cf*9.8
    time_to_stop= init_speed / deccel
    #EXPLICATIONS CALCUL dist_to_stop: 
        #t_stop= V0/deccel
        #On sait que dist(t_stop) = V0*t_stop - (1/2)*deccel*(t_stop**2)
        #Donc, en remplaçant t_stop dans la formule 
        #dist(t_stop) = V0*(V0/deccel) - (1/2)*deccel*((V0/deccel)**2)
        #dist(t_stop) = (V0**2)/deccel - (1/2)*((V0**2)/deccel) = (V0**2)/deccel + (((-1/2)*(V0**2)) / deccel)
        #dist(t_stop) = (V0**2) / (2*deccel)
    dist_to_stop= (init_speed**2) / (2*deccel)
    #SI la distance entre les 2 points demandés est supérieure à la distance de stop de balle
        #Utilisation d'une marge pour éviter les erreur dues aux approximations de calcul
    #Le temps pour atteindre finish_pos est infini. finish_pos est inatteignable 
    if move_dist > dist_to_stop+ 1e-10:
        return math.inf
    #SI la position est atteignable
    #On va résoudre l'équation: move_dist = init_speed * t - 0.5 * deccel * t^2 
    #-> -0.5 * deccel * t^2 + init_speed * t - move_dist = 0
    A= -0.5 * deccel 
    B= init_speed 
    C= -move_dist
    discrim= int(B**2 - 4*A*C)
    if discrim < 0:
        return -math.inf
    time_rotation= 0
    if carrier_rot_speed is not None and carrier_current_theta is not None:
        target_theta= math.atan2(finish_pos[1] - start_pos[1], finish_pos[0] - start_pos[0])
        rotation = abs(target_theta - carrier_current_theta)
        time_rotation = rotation / carrier_rot_speed
    t1= (-B + math.sqrt(discrim)) / (2*A)
    t2= (-B - math.sqrt(discrim)) / (2*A)
    if t1 >= 0 and t2 >= 0:
        return min(t1, t2) + time_rotation
    elif t1 <0 and t2 >= 0:
        return max(t1, t2) + time_rotation
    else:
        return -math.inf
    
#Récupération de la position d'arrêt de la balle ainsi que le temps pour atteindre cette position
def get_stop_ball(init_speed, start_pos, theta, cf):
    deccel= cf*9.8
    time_to_stop= init_speed / deccel
    dist_to_stop= (init_speed**2) / (2*deccel)
    stop_pos= [start_pos[0] + dist_to_stop*math.cos(theta), start_pos[1] + dist_to_stop*math.sin(theta)]
    return time_to_stop, stop_pos

#Fonction pour calculer la vitesse initiale à laquelle on doit envoyer le ballon pour s'arrêter après une distance donnée
def compute_ball_v0(dist, cf= 0.3):
    deccel= cf*GRAVITY
    return math.sqrt(2*deccel*dist)

def find_closest_point_on_segment(pt, pt_seg1, pt_seg2) :
    A = pt[0] - pt_seg1[0]
    B = pt[1] - pt_seg1[1]
    C = pt_seg2[0] - pt_seg1[0]
    D = pt_seg2[1] - pt_seg1[1]

    dot = A * C + B * D
    len_sq = C * C + D * D
    param = -1
    if (len_sq != 0):
        param = dot / len_sq

    if (param < 0):
        xx = pt_seg1[0]
        yy = pt_seg1[1]
    elif (param > 1):
        xx = pt_seg2[0]
        yy = pt_seg2[1]
    else:
        xx = pt_seg1[0] + param * C
        yy = pt_seg1[1] + param * D

    dx = pt[0] - xx
    dy = pt[1] - yy

    return np.array([xx, yy])

def modulo_2Pi( angleRad):
    angleTemp = (angleRad - math.pi) % (2 * math.pi) + math.pi
    return (angleTemp + math.pi) % (2 * math.pi) - math.pi

def compute_time_to_position(starting_pos, current_speed, max_speed, finish_pos, current_theta= None, rot_speed= None):
    dist= abs(distance(starting_pos, finish_pos))
    #Pour l'instant on considère que l'accélération = vitesse max |en partant à vitesse initiale nulle, on met 1s pour atteindre la vitesse max
    accel= max_speed
    time_to_max= (max_speed - current_speed) / accel
    max_speed_dist= 0.5*accel*(time_to_max**2)
    time_rotation= 0
    if rot_speed is not None and current_theta is not None:
        target_theta= math.atan2(finish_pos[1] - starting_pos[1], finish_pos[0] - starting_pos[0])
        rotation = abs(modulo_2Pi(target_theta - current_theta))
        time_rotation = rotation / rot_speed
    if max_speed_dist >= dist:
        return math.sqrt((2*dist)/accel) + time_rotation
    else:
        remain_dist= dist - max_speed_dist
        return time_to_max + (remain_dist / max_speed) + time_rotation

#Fonction permettant de calculer le temps et la distance pour atteindre une vitesse donnée (à partir d'une vitesse initiale donnée)
def compute_time_dist_to_speed(starting_speed, target_speed, accel):
    time_to_speed = (target_speed - starting_speed) / accel
    distance = starting_speed * time_to_speed + 0.5 * accel * (time_to_speed ** 2)
    return time_to_speed, distance

def limit_to_interval(value, low_limit, high_limit):
    if (value > high_limit):
        return high_limit
    elif (value < low_limit):
        return low_limit
    else:
        return value


def linear_in_interval(value, borne_basse, borne_haute, valeur_borne_basse, valeur_borne_haute):
    if (valeur_borne_haute >= valeur_borne_basse):
        return limit_to_interval((value - borne_basse) / (borne_haute - borne_basse) * (valeur_borne_haute - valeur_borne_basse) + valeur_borne_basse, valeur_borne_basse, valeur_borne_haute)

    else:
        return limit_to_interval((value - borne_basse) / (borne_haute - borne_basse) * (valeur_borne_haute - valeur_borne_basse) + valeur_borne_basse, valeur_borne_haute, valeur_borne_basse)