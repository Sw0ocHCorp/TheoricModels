import sys
import os
# Get the current script's directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory by going one level up
parent_dir = os.path.dirname(current_dir)
# Add the parent directory to sys.path
sys.path.append(parent_dir)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from copy import deepcopy
from time import perf_counter
import utils
GRAVITY= 9.81
FIELD_WIDTH= 22
FIELD_HEIGHT= 14

# VERSION ITERATIVE |Lourd en calcul (Surtout en Python)
# Balle s'arrête plus loin que la VERSION ITERATIVE
def compute_ball_dynamics_it(ball_init_pos, theta, ball_speed, cf, dt= 1/50):
    vx= ball_speed*math.cos(theta)
    vy= ball_speed*math.sin(theta)
    ball_pos= deepcopy(ball_init_pos.tolist())
    pos_history= [deepcopy(ball_pos)]
    while(abs(vx) > 0.05 or abs(vy) > 0.05):
        ball_pos[0] += vx*dt
        ball_pos[1] += vy*dt
        ball_speed -= cf*9.8*dt
        vx= ball_speed*math.cos(theta)
        vy= ball_speed*math.sin(theta)
        pos_history.append(deepcopy(ball_pos))
        if (vx < 0.05 and vy < 0.05):
            break
    return pos_history



#VERSION VANILLA (Pas d'infos sur les vitesses et dynamiques de balle)
def is_pass_possible_v(carrier_pos, tm_pos, ops_pos, pass_pos):
    #Position d'interception de la balle par le Teammate
    tm_intercept= utils.find_closest_point_on_segment(tm_pos, carrier_pos, pass_pos)
    #Ops
    closest_dist= math.inf
    closest_ops= None
    ops_intercept= None
    ops_index=0
    for j, ops in enumerate(ops_pos):
        intercept= utils.find_closest_point_on_segment(ops, carrier_pos, pass_pos)
        if (utils.utils.distance(ops, intercept) < closest_dist):
            closest_dist= utils.utils.distance(ops, intercept)
            closest_ops= ops
            ops_intercept= intercept
            ops_index= j
    #utils.Distance entre la position d'interception du Teammate sa position actuelle
    dist_tm_intercept= abs(utils.utils.distance(tm_pos, tm_intercept))
    #utils.Distance entre la position d'interception du Teammate et la position du porteur de balle
    dist_ball_tm= abs(utils.utils.distance(tm_intercept, carrier_pos))
    #utils.Distance entre la position d'interception de l'adversaire et sa position actuelle
    dist_ops_intercept= abs(utils.utils.distance(closest_ops, ops_intercept)) *0.8
    #utils.Distance entre la position d'interception de l'adversaire et la position du porteur de balle
    dist_ball_ops= abs(utils.utils.distance(ops_intercept, carrier_pos))
    #SI le coéquipier atteint sa position d'interception avant la balle ET que la balle atteint la position d'interception de l'adversaire avant lui
        # | Passe valide
    if ((dist_tm_intercept < dist_ball_tm) and (dist_ops_intercept > dist_ball_ops)):
        return True
    #SINON | Passe pas possible
    else:
        return False

#VERSION AVEC CONNAISSANCE DES VITESSES DES TM ET OPS + CALCUL DE DYNAMIQUE DE BALLE | VERSION ITERATIVE
def is_pass_possible_it(carrier, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_pos, ball_speed,
                        cf, dt):
    real_intercept= None
    theta= math.atan2(pass_pos[1] - carrier[1], pass_pos[0] - carrier[0])
    pass_dist= abs(utils.utils.distance(carrier, pass_pos))
    ball_pos= compute_ball_dynamics_it(carrier, theta, ball_speed, cf, dt)
    #Calcul des temps du Teammate / Ops / Ball
    #Teammate
    real_ball_dist = utils.utils.distance(ball_pos[0], ball_pos[-1])
    teammate= tm_pos.tolist()
    tm_intercept= sorted(ball_pos, key= lambda pos: abs(utils.compute_time_to_position(teammate, tms_init_speed, tms_max_speed, pos)))[0]
    #Temps que va mettre la balle pour atteindre la position d'interception du Teammate
    time_ball_tm= ball_pos.index(tm_intercept)*dt
    #Temps que va mettre le Teammate pour atteindre la position d'interception de la balle
    time_tm= utils.compute_time_to_position(tm_pos, tm_init_speed, tm_max_speed, tm_intercept)
    #Ops
    time_ops= math.inf
    closest_ops= None
    ops_intercept= None
    ops_index=0
    for j, ops in enumerate(ops_pos):
        intercept= sorted(ball_pos, key= lambda pos: abs(utils.compute_time_to_position(ops.tolist(), ops_init_speeds[j], ops_max_speeds[j], pos)))[0]
        if (utils.compute_time_to_position(ops.tolist(), ops_init_speeds[j], ops_max_speeds[j], intercept) < time_ops):
            time_ops= utils.compute_time_to_position(ops.tolist(), ops_init_speeds[j], ops_max_speeds[j], intercept)
            closest_ops= ops
            ops_intercept= intercept
            ops_index= j
    #On prend une marge de -20% sur le temps de trajet de l'adversaire | Pour être sûr qu'on que l'Adversaire peut pas intercepter la balle si passe possible
    time_ops *= 0.8
    #Temps que va mettre la balle pour atteindre la position d'interception de l'adversaire
    time_ball_ops= ball_pos.index(ops_intercept)*dt
    #Temps que va mettre le Teammate pour atteindre la position d'arrêt de la balle
    time_last_pos_tm= utils.compute_time_to_position(tm_pos, tm_init_speed, tm_max_speed, ball_pos[-1])
    #Temps que va mettre l'adversaire pour atteindre la position d'arrêt de la balle |Marge de -20% sur le temps de trajet de l'adversaire
    time_last_pos_ops= utils.compute_time_to_position(closest_ops, ops_init_speeds[ops_index], ops_max_speeds[ops_index], ball_pos[-1])*0.8
    #Si la position d'interception du Teammate se trouve dans les limites du Terrain ==> TOUJOURS VRAI
    #ET que l'adversaire ne peut pas intercepter le ballon à la position la plus proche de lui (temps de trajet de l'adversaire > temps de trajet de la balle) ==> TOUJOURS VRAI
    #ET que le Teammate peut intercepter la balle (temps de trajet du Teammate < temps de trajet de la balle) OU que le Teammate mets moins de temps que l'Adversaire pour atteindre la position d'arrêt de la balle (dans les limites du terrain)
    if (abs(tm_intercept[0]) < FIELD_WIDTH/2 and abs(tm_intercept[1]) < FIELD_HEIGHT/2 and (time_ops > time_ball_ops)) and \
        (time_tm < time_ball_tm or ((abs(ball_pos[-1][0]) < FIELD_WIDTH/2 and abs(ball_pos[-1][1]) < FIELD_HEIGHT/2) and (time_last_pos_tm < time_last_pos_ops))):
        if ((abs(ball_pos[-1][0]) < FIELD_WIDTH/2 and abs(ball_pos[-1][1]) < FIELD_HEIGHT/2) and (time_last_pos_tm < time_last_pos_ops)):
            real_intercept= ball_pos[-1]
        return True, real_intercept
    else:
        return False, None

#VERSION AVEC CONNAISSANCE DES VITESSES DES TM ET OPS + CALCUL DE DYNAMIQUE DE BALLE | VERSION ANALYTIQUE
def is_pass_possible_a(carrier, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_pos, ball_speed,
                        cf):
    time_pass= utils.time_ball_reach_pos(ball_speed, carrier, pass_pos, cf)
    physic_pass_pos= None
    #SI la position de passe demandé est inateignable | Balle s'arrête vant
    if time_pass == math.inf:
        theta= math.atan2(pass_pos[1] - carrier[1], pass_pos[0] - carrier[0])
        time_pass, physic_pass_pos = utils.get_stop_ball(ball_speed, carrier, theta, cf)
    tm_intercept= utils.find_closest_point_on_segment(tm_pos, carrier, pass_pos)
    if physic_pass_pos is not None:
        tm_intercept= utils.find_closest_point_on_segment(tm_pos, carrier, physic_pass_pos)
    #Temps que va mettre le Teammate pour atteindre la position d'interception la plus proche de lui
    time_tm= utils.compute_time_to_position(tm_pos, tm_init_speed, tm_max_speed, tm_intercept)
    #Temps que va mettre l'Adversaire pour atteindre la position d'interception la plus proche de lui
    ops_time= math.inf
    #Position d'interception la plus procher de l'Adversaire
    ops_intercept= None
    closest_ops= None
    ops_index= -1
    for i, ops in enumerate(ops_pos):
        intercept= utils.find_closest_point_on_segment(ops, carrier, pass_pos)
        if physic_pass_pos is not None:
            intercept= utils.find_closest_point_on_segment(ops, carrier, physic_pass_pos)
        tm= utils.compute_time_to_position(ops, ops_init_speeds[i], ops_max_speeds[i], intercept)
        if (tm < ops_time):
            ops_time = tm
            closest_ops= ops
            ops_index= i
            ops_intercept= intercept
    #Marge de -20% sur le temps de trajet de l'adversaire | Pour être sûr qu'on que l'Adversaire peut pas intercepter la balle si passe possible
    ops_time *= 0.8
    #Temps que va mettre la balle pour atteindre la position d'interception du Teammate
    time_ball_tm= utils.time_ball_reach_pos(ball_speed, carrier, tm_intercept, cf)
    #Temps que va mettre la balle pour atteindre la position d'interception de l'Adversaire
    time_ball_ops = utils.time_ball_reach_pos(ball_speed, carrier, ops_intercept, cf)
    #SI la position d'interception du Teammate reste dans les limites du terrain
    #ET que l'Adversaire ne peut pas intercepter le ballon ALORS(ET) que le coéquipier peut intercepter le Ballon 
    if (abs(tm_intercept[0]) < FIELD_WIDTH / 2 and abs(tm_intercept[1]) < FIELD_HEIGHT/2) and \
            (ops_time > time_ball_ops):
        return True, physic_pass_pos
    else:
        return False, physic_pass_pos

#VERSION ANALYTIQUE AVEC PRISE EN COMPTE DE POSITION DE BLOCAGE DE L'ADVERSAIRE (Si l'Adversaire le plus proche est plus rapide que le Teammate)
def is_pass_possible_opti(carrier, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_pos, ball_speed, cf, block_margin= 2):
    #Test de passe Analytique
    pass_possible, physic_pass_pos= is_pass_possible_a(carrier, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_pos, ball_speed, cf)
    #Si passe possible, pas de besoin de bloquer l'adversaire
    if pass_possible == True:
        return True, physic_pass_pos, None
    #Si passe pas possible => OPTIMISATION 
    #On va tester si le blocage de l'adversaire est possible et permet de rendre la passe possible
    else:
        target_pos= deepcopy(pass_pos)
        if physic_pass_pos is not None:
            target_pos= deepcopy(physic_pass_pos)
        closest_ops= None
        ops_time= math.inf
        ops_index= -1
        ops_intercept= None
        for i, ops in enumerate(ops_pos):
            intercept= utils.find_closest_point_on_segment(ops, carrier, target_pos)
            tm= utils.compute_time_to_position(ops, ops_init_speeds[i], ops_max_speeds[i], intercept)
            if (tm < ops_time):
                ops_time = tm
                closest_ops= ops
                ops_index= i
                ops_intercept= intercept
        theta_ops= math.atan2(ops_intercept[1] - closest_ops[1], ops_intercept[0] - closest_ops[0])
        block_pos= utils.find_closest_point_on_segment(tm_pos, closest_ops, ops_intercept)

        if (abs(utils.utils.distance(ops_intercept, closest_ops)) > 2*block_margin):
            block_bound1 = [closest_ops[0] + block_margin*math.cos(theta_ops), closest_ops[1] + block_margin*math.sin(theta_ops)]
            block_bound2 = [ops_intercept[0] - block_margin*math.cos(theta_ops), ops_intercept[1] - block_margin*math.sin(theta_ops)]
            block_pos= utils.find_closest_point_on_segment(tm_pos, block_bound1, block_bound2)
        theta_ops = math.degrees(theta_ops)
        theta_block = math.degrees(math.atan2(block_pos[1] - ops_intercept[1], block_pos[0] - ops_intercept[0]))
        theta_block2= math.degrees(math.atan2(block_pos[1] - closest_ops[1], block_pos[0] - closest_ops[0]))
        tm_intercept= utils.find_closest_point_on_segment(tm_pos, carrier, target_pos)
        #Verification de la possibilité de blocage
        time_tm_block= utils.compute_time_to_position(tm_pos, tm_init_speed, tm_max_speed, block_pos)
        time_ops_block= utils.compute_time_to_position(closest_ops, ops_init_speeds[ops_index], ops_max_speeds[ops_index], block_pos)
        time_ball= [0.0, 0.0]
        intercepts= [tm_intercept, ops_intercept]
        pass_possible= False
        #Si le Teammate peut intercepter la balle avant l'adversaire en le bloquant
        if time_tm_block < time_ops_block:
            #On doit recalculer le temps des 2 robots vers les  2 points d'interceptions disponibles
            for i in range(len(intercepts)):
                time_tm = time_tm_block + utils.compute_time_to_position(block_pos, tm_init_speed, tm_max_speed, intercepts[i])
                time_ops = time_ops_block + utils.compute_time_to_position(block_pos, 0.0, tm_max_speed, intercepts[i])
                time_ball = utils.time_ball_reach_pos(ball_speed, carrier, intercepts[i], cf)
                #SI le Teammate est capable de récupérer le ballon avant l'adversaire
                if (abs(intercepts[1][0]) < FIELD_WIDTH/2 and abs(intercepts[1][1]) < FIELD_HEIGHT/2) and \
                    ((time_tm < time_ball) and (time_tm < time_ops)):
                    pass_possible= True
                    break
        else:
            return False, target_pos, None
        
        if pass_possible == True:
            return True, target_pos, block_pos.tolist()
        else:
            return False, target_pos, None



#VERSION VANILLA (Pas d'infos sur les vitesses et dynamiques de balle)
def compute_possibles_pass_v(carrier_pos, tm_pos, ops_pos, tm_init_speeds= None, tm_max_speeds= None, ops_init_speedss= None, ops_max_speedss= None, pass_lines= 11, pass_step= 0.5):
    pass_pos= []
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = utils.modulo_2Pi(theta_direct + math.pi)
        #dist_pass= abs(utils.distance(tm, carrier_pos))
        #ball_init_speed = utils.compute_ball_v0(dist_pass)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible= is_pass_possible_v(carrier_pos, tm, ops_pos, tm)
        if pass_possible == True:
                n_phases= [2,2]
                if tm not in pass_pos:
                    pass_pos.append(deepcopy(tm.tolist()))
        # --> PASSE EN MOUVEMENT <--
        theta_tm= [math.degrees(utils.modulo_2Pi(math.radians(theta))) for theta in range(int(math.degrees(theta_tm_direct - math.pi / 2)), int(math.degrees(theta_tm_direct + math.pi/2)), int(math.degrees(math.pi/pass_lines)))]
        for theta in theta_tm:
            phases= deepcopy(n_phases)
            stop_search= [False, False]
            bounds= np.array([deepcopy(tm), deepcopy(tm)])
            while stop_search.count(True) < 2:
                for k in range(len(phases)):
                    if stop_search[k] == False:
                        match phases[k]:
                            #Positionner les bornes des zones de passe les plus proches du Teammate (Notament quand passe directe impossible)
                            case 1:
                                step= pass_step
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position du Teammate
                                bounds[k,0] += step*math.cos(math.radians(theta))
                                bounds[k,1] += step*math.sin(math.radians(theta))
                                dist_pass= abs(utils.distance(bounds[k], carrier_pos))
                                #ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible= is_pass_possible_v(carrier_pos, tm, ops_pos, bounds[k])
                                dist_tm_bound= utils.distance(tm, bounds[k])
                                dist_carrier_bound= utils.distance(carrier_pos, bounds[k])
                                #Si la position de passe est hors terrain OU que la position de passe est plus proche du Carrier que du Teammate | On stoppe la recherche. Pas de passes possibles pour cet angle, dans cette direction
                                if (abs(bounds[k][0]) > FIELD_WIDTH/2 or abs(bounds[k][1]) > FIELD_HEIGHT/2) or (pass_possible == True and dist_carrier_bound < dist_tm_bound):
                                    stop_search[k]= True
                                #Sinon, si passe possible | On a trouver la borne la plus proche du Teammate -> Passage à la phase 2, dans cette direction
                                elif pass_possible == True:
                                    phases[k]= 2
                                    bound= bounds[k].tolist()
                                    if bound not in pass_pos and utils.distance(bounds[k], tm) < utils.distance(bounds[k], carrier_pos):
                                        pass_pos.append(deepcopy(bound))  
                            #Positionner les limites de zones de passes les plus éloignées du Teammate
                            case 2:
                                step= pass_step
                                new_bound= deepcopy(bounds[k])
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position de la borne trouvée en phase 1 (Si passe direct possible, cette borne est la position du Teammate)
                                new_bound[0] += step*math.cos(math.radians(theta))
                                new_bound[1] += step*math.sin(math.radians(theta))
                                    
                                dist_pass= abs(utils.distance(new_bound, carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible= is_pass_possible_v(carrier_pos, tm, ops_pos, new_bound)
                                dist_tm_bound= utils.distance(tm, new_bound)
                                dist_carrier_bound= utils.distance(carrier_pos, new_bound)
                                #Si on peut faire la passe ET que le nouveau point de passe (theorique) est dans les limites du terrain ET que le point de passe(Theorique ou reel) est plus proche du Teammate que du Carrier
                                    #|On va stocker la nouvelle position de borne (position valide pour une passe)
                                if pass_possible == True and abs(new_bound[0]) < FIELD_WIDTH/2 and abs(new_bound[1]) < FIELD_HEIGHT/2 and dist_tm_bound < dist_carrier_bound:
                                    bounds[k]= new_bound  
                                #Si on ne peut plus faire la passe au niveau de la nouvelle position
                                #OU qu'on peut faire la passe ET que la dernière position stockée est dans les limites du terrain
                                #       (ATTENTION: Comme le if précédent est FAUX, ça veut dire que la nouvelle position(new_bound) est hors terrain, mais que la dernière position stockée est valide pour une passe)
                                # | On va ajouter la dernière position stockée dans les passes possibles (borne de zone de passe la plus éloignée du Teammate)
                                elif pass_possible == False or \
                                    pass_possible == True and (abs(bounds[k][0]) < FIELD_WIDTH/2 and abs(bounds[k][1]) < FIELD_HEIGHT/2):
                                    bound= bounds[k].tolist()
                                    if bound not in pass_pos:
                                        pass_pos.append(deepcopy(bound))
                                    stop_search[k]= True
                                

    return np.array(pass_pos), None, None, None

#VERSION AVEC CONNAISSANCE DES VITESSES DES TM ET OPS + CALCUL DE DYNAMIQUE DE BALLE | VERSION ITERATIVE
def compute_possibles_pass_it(carrier_pos, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_lines= 11, pass_step= 0.5, cf= 0.3, dt= 1/50):
    #ALGO EN 2 PHASES:
        #1 -> Recherche des bornes des zones de passes les plus proches du Teammate
        #2 -> Recherche des bornes des zones de passes les plus éloignées du Teammate
    pass_pos= []
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = utils.modulo_2Pi(theta_direct + math.pi)
        dist_pass= abs(utils.distance(tm, carrier_pos))
        ball_init_speed = utils.compute_ball_v0(dist_pass)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible, real_intercept= is_pass_possible_it(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, tm, ball_init_speed, cf= cf, dt= dt)
        if pass_possible == True:
                n_phases= [2,2]
                if real_intercept is not None and real_intercept not in pass_pos:
                    pass_pos.append(real_intercept)
                elif tm not in pass_pos:
                    pass_pos.append(deepcopy(tm.tolist()))
        # --> PASSE EN MOUVEMENT <--
        theta_tm= [math.degrees(utils.modulo_2Pi(math.radians(theta))) for theta in range(int(math.degrees(theta_tm_direct - math.pi / 2)), int(math.degrees(theta_tm_direct + math.pi/2)), int(math.degrees(math.pi/pass_lines)))]
        for theta in theta_tm:
            phases= deepcopy(n_phases)
            stop_search= [False, False]
            bounds= np.array([deepcopy(tm), deepcopy(tm)])
            real_intercept_p2= None
            while stop_search.count(True) < 2:
                for k in range(len(phases)):
                    if stop_search[k] == False:
                        match phases[k]:
                            #Positionner les bornes des zones de passe les plus proches du Teammate (Notament quand passe directe impossible)
                            case 1:
                                step= pass_step
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position du Teammate
                                bounds[k,0] += step*math.cos(math.radians(theta))
                                bounds[k,1] += step*math.sin(math.radians(theta))
                                dist_pass= abs(utils.distance(bounds[k], carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept= is_pass_possible_it(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, bounds[k], ball_init_speed, cf= cf, dt= dt)
                                dist_tm_bound= utils.distance(tm, bounds[k])
                                dist_carrier_bound= utils.distance(carrier_pos, bounds[k])
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si la position de passe est hors terrain OU que la position de passe est plus proche du Carrier que du Teammate | On stoppe la recherche. Pas de passes possibles pour cet angle, dans cette direction
                                if (abs(bounds[k][0]) > FIELD_WIDTH/2 or abs(bounds[k][1]) > FIELD_HEIGHT/2) or (pass_possible == True and dist_carrier_bound < dist_tm_bound):
                                    stop_search[k]= True
                                #Sinon, si passe possible | On a trouver la borne la plus proche du Teammate -> Passage à la phase 2, dans cette direction
                                elif pass_possible == True:
                                    phases[k]= 2
                                    bound= bounds[k].tolist()
                                    if real_intercept is not None and real_intercept not in pass_pos:
                                        pass_pos.append(real_intercept)
                                    elif bound not in pass_pos and utils.distance(bounds[k], tm) < utils.distance(bounds[k], carrier_pos):
                                        pass_pos.append(deepcopy(bound))  
                            #Positionner les limites de zones de passes les plus éloignées du Teammate
                            case 2:
                                step= pass_step
                                new_bound= deepcopy(bounds[k])
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position de la borne trouvée en phase 1 (Si passe direct possible, cette borne est la position du Teammate)
                                new_bound[0] += step*math.cos(math.radians(theta))
                                new_bound[1] += step*math.sin(math.radians(theta))
                                    
                                dist_pass= abs(utils.distance(new_bound, carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept= is_pass_possible_it(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, new_bound, ball_init_speed, cf= cf, dt= dt)
                                dist_tm_bound= utils.distance(tm, new_bound)
                                dist_carrier_bound= utils.distance(carrier_pos, new_bound)
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si on peut faire la passe ET que le nouveau point de passe (theorique) est dans les limites du terrain ET que le point de passe(Theorique ou reel) est plus proche du Teammate que du Carrier
                                    #|On va stocker la nouvelle position de borne (position valide pour une passe)
                                if pass_possible == True and abs(new_bound[0]) < FIELD_WIDTH/2 and abs(new_bound[1]) < FIELD_HEIGHT/2 and dist_tm_bound < dist_carrier_bound:
                                    bounds[k]= new_bound  
                                    real_intercept_p2= real_intercept
                                #Si on ne peut plus faire la passe au niveau de la nouvelle position
                                #OU qu'on peut faire la passe ET que la dernière position stockée est dans les limites du terrain
                                #       (ATTENTION: Comme le if précédent est FAUX, ça veut dire que la nouvelle position(new_bound) est hors terrain, mais que la dernière position stockée est valide pour une passe)
                                # | On va ajouter la dernière position stockée dans les passes possibles (borne de zone de passe la plus éloignée du Teammate)
                                elif pass_possible == False or \
                                    pass_possible == True and (abs(bounds[k][0]) < FIELD_WIDTH/2 and abs(bounds[k][1]) < FIELD_HEIGHT/2):
                                    bound= bounds[k].tolist()
                                    if real_intercept_p2 is not None and real_intercept_p2 not in pass_pos:
                                        pass_pos.append(real_intercept_p2)
                                    elif bound not in pass_pos:
                                        pass_pos.append(deepcopy(bound))
                                    stop_search[k]= True
                                

    return np.array(pass_pos), None, None, None

#VERSION AVEC CONNAISSANCE DES VITESSES DES TM ET OPS + CALCUL DE DYNAMIQUE DE BALLE | VERSION ANALYTIQUE
def compute_possibles_pass_a(carrier_pos, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_lines= 21, pass_step= 0.25, cf= 0.3, dt= 1/50):
    #ALGO EN 2 PHASES:
        #1 -> Recherche des bornes des zones de passes les plus proches du Teammate
        #2 -> Recherche des bornes des zones de passes les plus éloignées du Teammate
    pass_pos= []
    pass_no= []
    pass_zone= []
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = utils.modulo_2Pi(theta_direct + math.pi)
        dist_pass= abs(utils.distance(tm, carrier_pos))
        ball_init_speed = utils.compute_ball_v0(dist_pass)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible, real_intercept= is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, tm, ball_init_speed, cf= cf)
        if pass_possible == True:
                n_phases= [2,2]
                if real_intercept is not None and real_intercept not in pass_pos:
                    pass_pos.append(real_intercept)
                elif tm not in pass_pos:
                    pass_pos.append(deepcopy(tm.tolist()))
        # --> PASSE EN MOUVEMENT <--
        theta_tm= [math.degrees(utils.modulo_2Pi(math.radians(theta))) for theta in range(int(math.degrees(theta_tm_direct - math.pi / 2)), int(math.degrees(theta_tm_direct + math.pi/2)), int(math.degrees(math.pi/pass_lines)))]
        for theta in theta_tm:
            phases= deepcopy(n_phases)
            stop_search= [False, False]
            bounds= np.array([deepcopy(tm), deepcopy(tm)])
            real_intercept_p2= None
            while stop_search.count(True) < 2:
                for k in range(len(phases)):
                    if stop_search[k] == False:
                        match phases[k]:
                            #Positionner les bornes des zones de passe les plus proches du Teammate (Notament quand passe directe impossible)
                            case 1:
                                step= pass_step
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position du Teammate
                                bounds[k,0] += step*math.cos(math.radians(theta))
                                bounds[k,1] += step*math.sin(math.radians(theta))
                                dist_pass= abs(utils.distance(bounds[k], carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept= is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, bounds[k], ball_init_speed, cf= cf)
                                dist_tm_bound= utils.distance(tm, bounds[k])
                                dist_carrier_bound= utils.distance(carrier_pos, bounds[k])
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si la position de passe est hors terrain OU que la position de passe est plus proche du Carrier que du Teammate | On stoppe la recherche. Pas de passes possibles pour cet angle, dans cette direction
                                if (abs(bounds[k][0]) > FIELD_WIDTH/2 or abs(bounds[k][1]) > FIELD_HEIGHT/2) or (pass_possible == True and dist_carrier_bound < dist_tm_bound):
                                    stop_search[k]= True
                                    pass_no.append(bounds[k].tolist())
                                #Sinon, si passe possible | On a trouver la borne la plus proche du Teammate -> Passage à la phase 2, dans cette direction
                                elif pass_possible == True:
                                    phases[k]= 2
                                    bound= bounds[k].tolist()
                                    if real_intercept is not None and real_intercept not in pass_pos:
                                        pass_pos.append(real_intercept)
                                    elif bound not in pass_pos and utils.distance(bounds[k], tm) < utils.distance(bounds[k], carrier_pos):
                                        pass_pos.append(deepcopy(bound))  
                                else:
                                    pass_no.append(bounds[k].tolist())
                            #Positionner les limites de zones de passes les plus éloignées du Teammate
                            case 2:
                                step= pass_step
                                new_bound= deepcopy(bounds[k])
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position de la borne trouvée en phase 1 (Si passe direct possible, cette borne est la position du Teammate)
                                new_bound[0] += step*math.cos(math.radians(theta))
                                new_bound[1] += step*math.sin(math.radians(theta))
                                    
                                dist_pass= abs(utils.distance(new_bound, carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept= is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, new_bound, ball_init_speed, cf= cf)
                                dist_tm_bound= utils.distance(tm, new_bound)
                                dist_carrier_bound= utils.distance(carrier_pos, new_bound)
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si on peut faire la passe ET que le nouveau point de passe (theorique) est dans les limites du terrain ET que le point de passe(Theorique ou reel) est plus proche du Teammate que du Carrier
                                    #|On va stocker la nouvelle position de borne (position valide pour une passe)
                                if pass_possible == True and abs(new_bound[0]) < FIELD_WIDTH/2 and abs(new_bound[1]) < FIELD_HEIGHT/2 and dist_tm_bound < dist_carrier_bound:
                                    bounds[k]= new_bound  
                                    real_intercept_p2= real_intercept
                                    if real_intercept_p2 is not None and (len(pass_zone) == 0 or real_intercept_p2 not in np.array(pass_zone)):
                                        pass_zone.append(real_intercept_p2)
                                    if (len(pass_zone) == 0 or new_bound not in np.array(pass_zone)):
                                        pass_zone.append(new_bound)
                                #Si on ne peut plus faire la passe au niveau de la nouvelle position
                                #OU qu'on peut faire la passe ET que la dernière position stockée est dans les limites du terrain
                                #       (ATTENTION: Comme le if précédent est FAUX, ça veut dire que la nouvelle position(new_bound) est hors terrain, mais que la dernière position stockée est valide pour une passe)
                                # | On va ajouter la dernière position stockée dans les passes possibles (borne de zone de passe la plus éloignée du Teammate)
                                elif pass_possible == False or \
                                    pass_possible == True and (abs(bounds[k][0]) < FIELD_WIDTH/2 and abs(bounds[k][1]) < FIELD_HEIGHT/2):
                                    bound= bounds[k].tolist()
                                    if real_intercept_p2 is not None and real_intercept_p2 not in pass_pos:
                                        pass_pos.append(real_intercept_p2)
                                    elif bound not in pass_pos:
                                        pass_pos.append(deepcopy(bound))
                                    real_intercept_p2= None
                                    stop_search[k]= True
                                else:
                                    pass_no.append(bounds[k].tolist())
                                

    return np.array(pass_pos), None, np.array(pass_no), np.array(pass_zone)

#VERSION ANALYTIQUE AVEC POSSIBILITE DE BLOCAGE DE L'ADVERSAIRE POUR AVOIR PLUS D'OPPORTUNITES DE PASSES
def compute_possibles_pass_block(carrier_pos, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_lines= 51, pass_step= 0.2, cf= 0.3, dt= 1/50):
    #ALGO EN 2 PHASES:
        #1 -> Recherche des bornes des zones de passes les plus proches du Teammate
        #2 -> Recherche des bornes des zones de passes les plus éloignées du Teammate
    pass_pos= []
    block_path= []
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = utils.modulo_2Pi(theta_direct + math.pi)
        dist_pass= abs(utils.distance(tm, carrier_pos))
        ball_init_speed = utils.compute_ball_v0(dist_pass)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible, real_intercept, block_pos= is_pass_possible_opti(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, tm, ball_init_speed, cf= cf)
        if pass_possible == True:
                n_phases= [2,2]
                if real_intercept is not None and (len(pass_pos) == 0 or real_intercept not in np.array(pass_pos)):
                    
                    pass_pos.append(real_intercept)
                elif (len(pass_pos) == 0 or tm not in np.array(pass_pos)):
                    pass_pos.append(deepcopy(tm.tolist()))
                if block_pos is not None:
                    if real_intercept is not None:
                        block_path.append([tm, block_pos, real_intercept])
                    else:
                        block_path.append([tm, block_pos, deepcopy(tm.tolist())])
        # --> PASSE EN MOUVEMENT <--
        theta_tm= [math.degrees(utils.modulo_2Pi(math.radians(theta))) for theta in range(int(math.degrees(theta_tm_direct - math.pi / 2)), int(math.degrees(theta_tm_direct + math.pi/2)), int(math.degrees(math.pi/pass_lines)))]
        for theta in theta_tm:
            phases= deepcopy(n_phases)
            stop_search= [False, False]
            bounds= np.array([deepcopy(tm), deepcopy(tm)])
            real_intercept_p2= None
            block_pos_p2= None
            while stop_search.count(True) < 2:
                for k in range(len(phases)):
                    if stop_search[k] == False:
                        match phases[k]:
                            #Positionner les bornes des zones de passe les plus proches du Teammate (Notament quand passe directe impossible)
                            case 1:
                                step= pass_step
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position du Teammate
                                bounds[k,0] += step*math.cos(math.radians(theta))
                                bounds[k,1] += step*math.sin(math.radians(theta))
                                dist_pass= abs(utils.distance(bounds[k], carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept, block_pos= is_pass_possible_opti(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, bounds[k], ball_init_speed, cf= cf)
                                dist_tm_bound= utils.distance(tm, bounds[k])
                                dist_carrier_bound= utils.distance(carrier_pos, bounds[k])
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si la position de passe est hors terrain OU que la position de passe est plus proche du Carrier que du Teammate | On stoppe la recherche. Pas de passes possibles pour cet angle, dans cette direction
                                if (abs(bounds[k][0]) > FIELD_WIDTH/2 or abs(bounds[k][1]) > FIELD_HEIGHT/2) or (pass_possible == True and dist_carrier_bound < dist_tm_bound):
                                    stop_search[k]= True
                                #Sinon, si passe possible | On a trouver la borne la plus proche du Teammate -> Passage à la phase 2, dans cette direction
                                elif pass_possible == True:
                                    phases[k]= 2
                                    bound= bounds[k].tolist()
                                    if real_intercept is not None and real_intercept not in pass_pos:
                                        pass_pos.append(real_intercept)
                                    elif bound not in pass_pos and utils.distance(bounds[k], tm) < utils.distance(bounds[k], carrier_pos):
                                        pass_pos.append(deepcopy(bound))  
                                    if block_pos is not None:
                                        if real_intercept is not None:
                                            block_path.append([tm, block_pos, real_intercept])
                                        else:
                                            block_path.append([tm, block_pos, deepcopy(tm.tolist())])
                            #Positionner les limites de zones de passes les plus éloignées du Teammate
                            case 2:
                                step= pass_step
                                new_bound= deepcopy(bounds[k])
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position de la borne trouvée en phase 1 (Si passe direct possible, cette borne est la position du Teammate)
                                new_bound[0] += step*math.cos(math.radians(theta))
                                new_bound[1] += step*math.sin(math.radians(theta))
                                    
                                dist_pass= abs(utils.distance(new_bound, carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept, block_pos= is_pass_possible_opti(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, new_bound, ball_init_speed, cf= cf)
                                dist_tm_bound= utils.distance(tm, new_bound)
                                dist_carrier_bound= utils.distance(carrier_pos, new_bound)
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si on peut faire la passe ET que le nouveau point de passe (theorique) est dans les limites du terrain ET que le point de passe(Theorique ou reel) est plus proche du Teammate que du Carrier
                                    #|On va stocker la nouvelle position de borne (position valide pour une passe)
                                if pass_possible == True and abs(new_bound[0]) < FIELD_WIDTH/2 and abs(new_bound[1]) < FIELD_HEIGHT/2 and dist_tm_bound < dist_carrier_bound:
                                    bounds[k]= new_bound  
                                    real_intercept_p2= real_intercept
                                    block_pos_p2= block_pos
                                #Si on ne peut plus faire la passe au niveau de la nouvelle position
                                #OU qu'on peut faire la passe ET que la dernière position stockée est dans les limites du terrain
                                #       (ATTENTION: Comme le if précédent est FAUX, ça veut dire que la nouvelle position(new_bound) est hors terrain, mais que la dernière position stockée est valide pour une passe)
                                # | On va ajouter la dernière position stockée dans les passes possibles (borne de zone de passe la plus éloignée du Teammate)
                                elif pass_possible == False or \
                                    pass_possible == True and (abs(bounds[k][0]) < FIELD_WIDTH/2 and abs(bounds[k][1]) < FIELD_HEIGHT/2):
                                    bound= bounds[k].tolist()
                                    if real_intercept_p2 is not None:
                                        pass_pos.append(real_intercept_p2)
                                    elif bound not in np.array(pass_pos):
                                        pass_pos.append(deepcopy(bound))
                                    if block_pos_p2 is not None:
                                        if real_intercept is not None:
                                            block_path.append([tm, block_pos_p2, real_intercept_p2])
                                        else:
                                            block_path.append([tm, block_pos_p2, deepcopy(tm.tolist())])
                                    block_pos_p2= None
                                    real_intercept_p2= None
                                    stop_search[k]= True
                                

    return np.array(pass_pos), np.array(block_path), None, None

#VERSION ANALYTIQUE ROBUSTE AUX OCCULTATIONS (Zone de passe coupée par le cône d'occultation de l'adversaire)
def compute_possibles_pass_occult(carrier_pos, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speeds, ops_max_speeds, pass_lines= 51, pass_step= 0.2, cf= 0.3, dt= 1/50):
    #ALGO EN 2 PHASES:
        #1 -> Recherche des bornes des zones de passes les plus proches du Teammate
        #2 -> Recherche des bornes des zones de passes les plus éloignées du Teammate
    pass_pos= []
    pass_no= []
    pass_zone= []
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = utils.modulo_2Pi(theta_direct + math.pi)
        dist_pass= abs(utils.distance(tm, carrier_pos))
        ball_init_speed = utils.compute_ball_v0(dist_pass)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible, real_intercept= is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, tm, ball_init_speed, cf= cf)
        if pass_possible == True:
                n_phases= [2,2]
                if real_intercept is not None and real_intercept not in pass_pos:
                    pass_pos.append(real_intercept)
                elif len(pass_pos) == 0 or tm not in np.array(pass_pos):
                    pass_pos.append(deepcopy(tm.tolist()))
        # --> PASSE EN MOUVEMENT <--
        theta_tm= [math.degrees(utils.modulo_2Pi(math.radians(theta))) for theta in range(int(math.degrees(theta_tm_direct - math.pi / 2)), int(math.degrees(theta_tm_direct + math.pi/2)), int(math.degrees(math.pi/pass_lines)))]
        for theta in theta_tm:
            phases= deepcopy(n_phases)
            stop_search= [False, False]
            bounds= np.array([deepcopy(tm), deepcopy(tm)])
            real_intercept_p2= None
            while stop_search.count(True) < 2:
                for k in range(len(phases)):
                    if stop_search[k] == False:
                        match phases[k]:
                            #Positionner les bornes des zones de passe les plus proches du Teammate (Notament quand passe directe impossible)
                            case 1:
                                step= pass_step
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position du Teammate
                                bounds[k,0] += step*math.cos(math.radians(theta))
                                bounds[k,1] += step*math.sin(math.radians(theta))
                                dist_pass= abs(utils.distance(bounds[k], carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept= is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, bounds[k], ball_init_speed, cf= cf)
                                dist_tm_bound= utils.distance(tm, bounds[k])
                                dist_carrier_bound= utils.distance(carrier_pos, bounds[k])
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si la position de passe est hors terrain OU que la position de passe est plus proche du Carrier que du Teammate | On stoppe la recherche. Pas de passes possibles pour cet angle, dans cette direction
                                if (abs(bounds[k][0]) > FIELD_WIDTH/2 or abs(bounds[k][1]) > FIELD_HEIGHT/2) or (pass_possible == True and dist_carrier_bound < dist_tm_bound):
                                    stop_search[k]= True
                                    pass_no.append(bounds[k].tolist())
                                #Sinon, si passe possible | On a trouver la borne la plus proche du Teammate -> Passage à la phase 2, dans cette direction
                                elif pass_possible == True:
                                    phases[k]= 2
                                    bound= bounds[k].tolist()
                                    if real_intercept is not None and real_intercept not in pass_pos:
                                        pass_pos.append(real_intercept)
                                    elif bound not in pass_pos and utils.distance(bounds[k], tm) < utils.distance(bounds[k], carrier_pos):
                                        pass_pos.append(deepcopy(bound))  
                                else:
                                    pass_no.append(bounds[k].tolist())
                            #Positionner les limites de zones de passes les plus éloignées du Teammate
                            case 2:
                                step= pass_step
                                new_bound= deepcopy(bounds[k])
                                if k == 1:
                                    step= -pass_step
                                #Eloignement des bornes par rapport à la position de la borne trouvée en phase 1 (Si passe direct possible, cette borne est la position du Teammate)
                                new_bound[0] += step*math.cos(math.radians(theta))
                                new_bound[1] += step*math.sin(math.radians(theta))
                                    
                                dist_pass= abs(utils.distance(new_bound, carrier_pos))
                                ball_init_speed = utils.compute_ball_v0(dist_pass)
                                pass_possible, real_intercept= is_pass_possible_a(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speeds, ops_max_speeds, new_bound, ball_init_speed, cf= cf)
                                dist_tm_bound= utils.distance(tm, new_bound)
                                dist_carrier_bound= utils.distance(carrier_pos, new_bound)
                                if real_intercept is not None:
                                    dist_carrier_bound= utils.distance(carrier_pos, real_intercept)
                                    dist_tm_bound= utils.distance(tm, real_intercept)
                                #Si on peut faire la passe ET que le nouveau point de passe (theorique) est dans les limites du terrain ET que le point de passe(Theorique ou reel) est plus proche du Teammate que du Carrier
                                    #|On va stocker la nouvelle position de borne (position valide pour une passe)
                                if pass_possible == True and abs(new_bound[0]) < FIELD_WIDTH/2 and abs(new_bound[1]) < FIELD_HEIGHT/2 and dist_tm_bound < dist_carrier_bound:
                                    bounds[k]= new_bound  
                                    real_intercept_p2= real_intercept
                                    if real_intercept_p2 is not None and (len(pass_zone) == 0 or real_intercept_p2 not in np.array(pass_zone)):
                                        pass_zone.append(real_intercept_p2)
                                    if (len(pass_zone) == 0 or new_bound not in np.array(pass_zone)):
                                        pass_zone.append(new_bound)
                                #Si on ne peut plus faire la passe au niveau de la nouvelle position
                                #OU qu'on peut faire la passe ET que la dernière position stockée est dans les limites du terrain
                                #       (ATTENTION: Comme le if précédent est FAUX, ça veut dire que la nouvelle position(new_bound) est hors terrain, mais que la dernière position stockée est valide pour une passe)
                                # | On va ajouter la dernière position stockée dans les passes possibles (borne de zone de passe la plus éloignée du Teammate)
                                elif pass_possible == False or \
                                    pass_possible == True and (abs(bounds[k][0]) < FIELD_WIDTH/2 and abs(bounds[k][1]) < FIELD_HEIGHT/2):
                                    bound= bounds[k].tolist()
                                    if real_intercept_p2 is not None and real_intercept_p2 not in pass_pos:
                                        pass_pos.append(real_intercept_p2)
                                    elif bound not in pass_pos:
                                        pass_pos.append(deepcopy(bound))
                                    real_intercept_p2= None
                                    #Si la new_bound est dans le cône d'occultation de l'adversaire | On va retourner en phase 1 pour chercher la limite de la nouvelle zone de passe
                                    #Pour le detecter, on rentre dans ce cône quand new_bound est plus proche de l'adversaire que bounds[k] (Dernière position de zone de passe valide)
                                    closest_ops= None
                                    ops_dist= math.inf
                                    for ops in ops_pos:
                                        dist= utils.distance(ops, new_bound)
                                        if dist < ops_dist:
                                            ops_dist= dist
                                            closest_ops= ops
                                    if utils.distance(new_bound, closest_ops) < utils.distance(bounds[k], closest_ops):
                                        phases[k]= 1
                                    else:
                                        stop_search[k]= True
                                else:
                                    pass_no.append(bounds[k].tolist())
                                

    return np.array(pass_pos), None, np.array(pass_no), np.array(pass_zone)

#Fonction de calcul de passes possibles par segmentation de l'espace de jeu | Permet de comparer et valider les zones de passes trouvées par les algorithmes
def pass_zone_validator(ax, pass_verif_algorithm, field, carrier, tms, ops, title, segmentation_step= 0.25, cf= 0.3, tm_init_speeds= None, tm_max_speeds= None, ops_init_speedss= None, ops_max_speedss= None):
    pass_ok= []
    pass_no= []
    p = Polygon(field, color= (0, 0.7, 0, 0.4))
    ax.add_patch(p)
    ax.set_xlim([-(FIELD_WIDTH/2) + 0.25,(FIELD_WIDTH/2) + 0.25])
    ax.set_ylim([-(FIELD_HEIGHT/2) + 0.25, (FIELD_HEIGHT/2) + 0.25])
    #Test de la possibilité de faire une passe pour chaque position de l'espace de jeu
    for y in np.arange(-utils.FIELD_HEIGHT/2 + segmentation_step/2, (utils.FIELD_HEIGHT + segmentation_step)/2 - segmentation_step/2, segmentation_step):
        for x in np.arange(-utils.FIELD_WIDTH/2 + segmentation_step/2, (utils.FIELD_WIDTH + segmentation_step)/2 - segmentation_step/2, segmentation_step):
            for i, tm in enumerate(tms):
                dist_pass= abs(utils.distance([x, y], carrier))
                ball_init_speed = utils.compute_ball_v0(dist_pass)
                pass_possible,physic_stop_pos= pass_verif_algorithm(carrier, tm, ops, tm_init_speeds[i], tm_max_speeds[i], ops_init_speedss, ops_max_speedss, [x, y], ball_init_speed, cf= cf)
                #On calcul la ditance entre le porteur de balle et la position de passe
                carrier_pass_dist= utils.distance([x, y], carrier)
                tm_pass_dist= utils.distance([x, y], tm)
                if physic_stop_pos is not None:
                    carrier_pass_dist= utils.distance(physic_stop_pos, carrier)
                    tm_pass_dist= utils.distance(physic_stop_pos, tm)
                #Si la passe est possible et que la position de passe est plus proche du Teammate que du Carrier (sinon la passe est inutile)
                if pass_possible == True and (carrier_pass_dist > tm_pass_dist):
                    pass_ok.append([x, y])
                else:
                    pass_no.append([x, y])
    if len(pass_no) > 0:
        pass_no= np.array(pass_no)
        ax.scatter(pass_no[:,0], pass_no[:,1], color= "red",label= "Passes impossibles")
    if len(pass_ok) > 0:
        pass_ok= np.array(pass_ok)
        ax.scatter(pass_ok[:,0], pass_ok[:,1], color= "blue", label= "Passes possibles")
    ax.scatter([carrier[0]], [carrier[1]], marker='o', color= "black", label= "Carrier")
    ax.scatter(tms[:,0], tms[:,1], marker= "o", color= "purple", label= "Teammates")
    ax.scatter(ops[:,0], ops[:,1], marker= "o", color= "orange", label= "Opponents")
    ax.legend()
        
#Fonction générique pour plotter les situations de jeu avec n'importe quel algorithme de calcul de passes possibles
#A condition que la fonction prenne les mêmes arguments
def plot_situation_problem(ax, pass_algorithm, field, carrier, tms, ops, title, tm_init_speeds= None, tm_max_speeds= None, ops_init_speedss= None, ops_max_speedss= None):
    p = Polygon(field, color= (0, 0.7, 0, 0.4))
    start= perf_counter()
    pass_pos, block_path, pass_no, zones_passes= pass_algorithm(carrier, tms, ops, tm_init_speeds, tm_max_speeds, ops_init_speedss, ops_max_speedss)
    elapse_time= perf_counter() - start
    ax.add_patch(p)
    ax.set_xlim([-(FIELD_WIDTH/2) + 0.25,(FIELD_WIDTH/2) + 0.25])
    ax.set_ylim([-(FIELD_HEIGHT/2) + 0.25, (FIELD_HEIGHT/2) + 0.25])
    if block_path is not None:
        for i, path in (enumerate(block_path)):
            if i == 0:
                ax.plot(path[:,0], path[:,1], color= "pink", linestyle= "--", label= "Block Path")
            else:
                ax.plot(path[:,0], path[:,1], color= "pink", linestyle= "--")
    if pass_no is not None and pass_no.shape[0] > 0:
        ax.scatter(pass_no[:,0], pass_no[:,1], color= "red", label= "Passes impossibles")
    if zones_passes is not None:
        ax.scatter(zones_passes[:,0], zones_passes[:,1], color= "green", label= "Zones de passes")
    if pass_pos.shape[0] > 0:
        ax.scatter(pass_pos[:,0], pass_pos[:,1], label= "Passes possibles")
    ax.scatter([carrier[0]], [carrier[1]], marker='o', color= "black", label= "Carrier")
    ax.scatter(tms[:,0], tms[:,1], marker= "o", color= "purple", label= "Teammates")
    ax.scatter(ops[:,0], ops[:,1], marker= "o", color= "orange", label= "Opponents")
    ax.legend()
    ax.set_title(title + "\n Temps d'exécution= " + str(elapse_time) + "s")

if __name__ == "__main__":
    #Dimensions du terrain
    field = np.array([[FIELD_WIDTH/2, -FIELD_HEIGHT/2], 
                        [FIELD_WIDTH/2, FIELD_HEIGHT/2], 
                        [-FIELD_WIDTH/2, FIELD_HEIGHT/2], 
                        [-FIELD_WIDTH/2, -FIELD_HEIGHT/2]], dtype= float)
    carrier = np.array([0, -5], dtype= float)
    tms= np.array([[-4, 0], [-3, 6], [3, -2], [10, 0]], dtype= float)
    ops= np.array([[-2, -5], [2, -5], [0, -3], [-7, 0], [-10, 0]], dtype= float)
    tms_init_speed= np.array([0, 0, 0, 0], dtype= float)
    tms_max_speed= np.array([2.5, 2.5, 2.5, 1])
    ops_init_speeds= np.array([0, 0, 0, 0, 0], dtype= float)
    ops_max_speeds= np.array([2.5, 2.5, 2.5, 2.5, 1], dtype= float)

    fig,ax = plt.subplots(2,1)
    """plot_situation_problem(ax[0], compute_possibles_pass_it, field, carrier, tms, ops, "Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s\n Version ITERATIVE",
                            tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)"""
    plot_situation_problem(ax[0], compute_possibles_pass_occult, field, carrier, tms, ops, "Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s\n Version ANALYTIQUE",
                            tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)
    pass_zone_validator(ax[1], is_pass_possible_a, field, carrier, tms, ops, "Validation des zones de passes possibles\n vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s",
                                0.25, 0.3, tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)
    """plot_situation_problem(ax[1], compute_possibles_pass_block, field, carrier, tms, ops, "VERSION ANALYTIQUE Avec positions de blocages de l'Adversaire\n vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speeds) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speeds) + "m/s",
                            tms_init_speed, tms_max_speed, ops_init_speeds, ops_max_speeds)"""
    
    plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, top=0.9, wspace=0.2, hspace=0.55)
    plt.show()