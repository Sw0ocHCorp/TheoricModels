import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from copy import deepcopy

FIELD_WIDTH= 22
FIELD_HEIGHT= 14
# VERSION ITERATIVE |Lourd en calcul (Surtout en Python)
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

def distance(pt1, pt2):
     if (pt1 is not None and pt2 is not None):
         return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
     else:
         return math.inf

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

def compute_time_to_position(starting_pos, currentSpeed, maxSpeed, finish_pos):
    dist= abs(distance(starting_pos, finish_pos))
    #Pour l'instant on considère que l'accélération = vitesse max |en partant à vitesse initiale nulle, on met 1s pour atteindre la vitesse max
    accel= maxSpeed
    timeToMax= (maxSpeed - currentSpeed) / accel
    maxSpeedDist= 0.5*accel*(timeToMax**2)
    if maxSpeedDist >= dist:
        return math.sqrt((2*dist)/accel)
    else:
        remainDist= dist - maxSpeedDist
        return timeToMax + (remainDist / maxSpeed)

def limit_to_interval(value, lowLimit, highLimit):
    if (value > highLimit):
        return highLimit
    elif (value < lowLimit):
        return lowLimit
    else:
        return value


def linear_in_interval(value, borneBasse, borneHaute, valeurBorneBasse, valeurBorneHaute):
    if (valeurBorneHaute >= valeurBorneBasse):
        return limit_to_interval((value - borneBasse) / (borneHaute - borneBasse) * (valeurBorneHaute - valeurBorneBasse) + valeurBorneBasse, valeurBorneBasse, valeurBorneHaute)

    else:
        return limit_to_interval((value - borneBasse) / (borneHaute - borneBasse) * (valeurBorneHaute - valeurBorneBasse) + valeurBorneBasse, valeurBorneHaute, valeurBorneBasse)

#VERSION VANILLA (Pas d'infos sur les vitesses et dynamiques de balle)
def is_pass_possible_v(carrier_pos, tm_pos, ops_pos, pass_pos):
    #Position d'interception de la balle par le Teammate
    tm_intercept= find_closest_point_on_segment(tm_pos, carrier_pos, pass_pos)
    #Ops
    closestDist= math.inf
    closestOps= None
    ops_intercept= None
    opsIndex=0
    for j, ops in enumerate(ops_pos):
        intercept= find_closest_point_on_segment(ops, carrier_pos, pass_pos)
        if (distance(ops, intercept) < closestDist):
            closestDist= distance(ops, intercept)
            closestOps= ops
            ops_intercept= intercept
            opsIndex= j
    #Distance entre la position d'interception du Teammate sa position actuelle
    dist_tm_intercept= abs(distance(tm_pos, tm_intercept))
    #Distance entre la position d'interception du Teammate et la position du porteur de balle
    dist_ball_tm= abs(distance(tm_intercept, carrier_pos))
    #Distance entre la position d'interception de l'adversaire et sa position actuelle
    dist_ops_intercept= abs(distance(closestOps, ops_intercept)) *0.8
    #Distance entre la position d'interception de l'adversaire et la position du porteur de balle
    dist_ball_ops= abs(distance(ops_intercept, carrier_pos))
    #SI le coéquipier atteint sa position d'interception avant la balle ET que la balle atteint la position d'interception de l'adversaire avant lui
        # | Passe valide
    if ((dist_tm_intercept < dist_ball_tm) and (dist_ops_intercept > dist_ball_ops)):
        return True
    #SINON | Passe pas possible
    else:
        return False

#VERSION VANILLA (Pas d'infos sur les vitesses et dynamiques de balle)
def compute_possibles_pass_v(carrier_pos, tm_pos, ops_pos, pass_lines= 11, pass_step= 0.5):
    pass_pos= []
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = modulo_2Pi(theta_direct + math.pi)
        #dist_pass= abs(distance(tm, carrier_pos))
        #ball_init_speed = linear_in_interval(dist_pass, 0, 10, 2.0, 8)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible= is_pass_possible_v(carrier_pos, tm, ops_pos, tm)
        if pass_possible == True:
                n_phases= [2,2]
                if tm not in pass_pos:
                    pass_pos.append(deepcopy(tm.tolist()))
        # --> PASSE EN MOUVEMENT <--
        theta_tm= [math.degrees(modulo_2Pi(math.radians(theta))) for theta in range(int(math.degrees(theta_tm_direct - math.pi / 2)), int(math.degrees(theta_tm_direct + math.pi/2)), int(math.degrees(math.pi/pass_lines)))]
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
                                dist_pass= abs(distance(bounds[k], carrier_pos))
                                #ball_init_speed = linear_in_interval(dist_pass, 0, 10, 2.0, 8)
                                pass_possible= is_pass_possible_v(carrier_pos, tm, ops_pos, bounds[k])
                                distTmBound= distance(tm, bounds[k])
                                distCarrierBound= distance(carrier_pos, bounds[k])
                                #Si la position de passe est hors terrain OU que la position de passe est plus proche du Carrier que du Teammate | On stoppe la recherche. Pas de passes possibles pour cet angle, dans cette direction
                                if (abs(bounds[k][0]) > FIELD_WIDTH/2 or abs(bounds[k][1]) > FIELD_HEIGHT/2) or (pass_possible == True and distCarrierBound < distTmBound):
                                    stop_search[k]= True
                                #Sinon, si passe possible | On a trouver la borne la plus proche du Teammate -> Passage à la phase 2, dans cette direction
                                elif pass_possible == True:
                                    phases[k]= 2
                                    bound= bounds[k].tolist()
                                    if bound not in pass_pos and distance(bounds[k], tm) < distance(bounds[k], carrier_pos):
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
                                    
                                dist_pass= abs(distance(new_bound, carrier_pos))
                                ball_init_speed = linear_in_interval(dist_pass, 0, 10, 2.0, 8)
                                pass_possible= is_pass_possible_v(carrier_pos, tm, ops_pos, new_bound)
                                distTmBound= distance(tm, new_bound)
                                distCarrierBound= distance(carrier_pos, new_bound)
                                #Si on peut faire la passe ET que le nouveau point de passe (theorique) est dans les limites du terrain ET que le point de passe(Theorique ou reel) est plus proche du Teammate que du Carrier
                                    #|On va stocker la nouvelle position de borne (position valide pour une passe)
                                if pass_possible == True and abs(new_bound[0]) < FIELD_WIDTH/2 and abs(new_bound[1]) < FIELD_HEIGHT/2 and distTmBound < distCarrierBound:
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
                                

    return np.array(pass_pos)

#VERSION AVEC CONNAISSANCE DES VITESSES DES TM ET OPS + CALCUL DE DYNAMIQUE DE BALLE
def is_pass_possible(carrier, tm_pos, ops_pos, tm_init_speed, tm_max_speed, ops_init_speed, ops_max_speed, pass_pos, ball_speed,
                        cf, dt):
    real_intercept= None
    theta= math.atan2(pass_pos[1] - carrier[1], pass_pos[0] - carrier[0])
    pass_dist= abs(distance(carrier, pass_pos))
    ball_pos= _it(carrier, theta, ball_speed, cf, dt)
    #Calcul des temps du Teammate / Ops / Ball
    #Teammate
    real_ball_dist = distance(ball_pos[0], ball_pos[-1])
    teammate= tm_pos.tolist()
    tmIntercept= sorted(ball_pos, key= lambda pos: abs(distance(teammate, pos)))[0]
    #Temps que va mettre la balle pour atteindre la position d'interception du Teammate
    timeBallTm= ball_pos.index(tmIntercept)*dt
    #Temps que va mettre le Teammate pour atteindre la position d'interception de la balle
    timeTm= compute_time_to_position(tm_pos, tm_init_speed, tm_max_speed, tmIntercept)
    #Ops
    closestDist= math.inf
    closestOps= None
    opsIntercept= None
    opsIndex=0
    for j, ops in enumerate(ops_pos):
        intercept= sorted(ball_pos, key= lambda pos: abs(distance(ops.tolist(), pos)))[0]
        if (distance(ops, intercept) < closestDist):
            closestDist= distance(ops, intercept)
            closestOps= ops
            opsIntercept= intercept
            opsIndex= j
    #Temps que va mettre la balle pour atteindre la position d'interception de l'adversaire
    timeBallOps= ball_pos.index(opsIntercept)*dt
    #Temps que va mettre l'adversaire pour atteindre la position d'interception de la balle
    timeOps= compute_time_to_position(closestOps, ops_init_speed[opsIndex], ops_max_speed[opsIndex], opsIntercept) * 0.8
    #Temps que va mettre le Teammate pour atteindre la position d'arrêt de la balle
    timeLastPosTm= compute_time_to_position(tm_pos, tm_init_speed, tm_max_speed, ball_pos[-1])
    #Temps que va mettre l'adversaire pour atteindre la position d'arrêt de la balle
    timeLastPosOps= compute_time_to_position(closestOps, ops_init_speed[opsIndex], ops_max_speed[opsIndex], ball_pos[-1])
    #Si la position d'interception du Teammate se trouve dans les limites du Terrain ==> TOUJOURS VRAI
    #ET que l'adversaire ne peut pas intercepter le ballon à la position la plus proche de lui (temps de trajet de l'adversaire > temps de trajet de la balle) ==> TOUJOURS VRAI
    #ET que le Teammate peut intercepter la balle (temps de trajet du Teammate < temps de trajet de la balle) OU que le Teammate mets moins de temps que l'Adversaire pour atteindre la position d'arrêt de la balle (dans les limites du terrain)
    if (abs(tmIntercept[0]) < FIELD_WIDTH/2 and abs(tmIntercept[1]) < FIELD_HEIGHT/2 and (timeOps > timeBallOps)) and \
        (timeTm < timeBallTm or ((abs(ball_pos[-1][0]) < FIELD_WIDTH/2 and abs(ball_pos[-1][1]) < FIELD_HEIGHT/2) and (timeLastPosTm < timeLastPosOps))):
        if ((abs(ball_pos[-1][0]) < FIELD_WIDTH/2 and abs(ball_pos[-1][1]) < FIELD_HEIGHT/2) and (timeLastPosTm < timeLastPosOps)):
            real_intercept= ball_pos[-1]
        return True, real_intercept
    else:
        return False, None

#VERSION AVEC CONNAISSANCE DES VITESSES DES TM ET OPS + CALCUL DE DYNAMIQUE DE BALLE
def compute_possibles_pass(carrier_pos, tm_pos, tm_init_speed, tm_max_speed, ops_pos, ops_init_speed, ops_max_speed, pass_lines= 11, pass_step= 0.5, cf= 0.3, dt= 1/50):
    #ALGO EN 2 PHASES:
        #1 -> Recherche des bornes des zones de passes les plus proches du Teammate
        #2 -> Recherche des bornes des zones de passes les plus éloignées du Teammate
    pass_pos= []
    for i, tm in enumerate(tm_pos):
        theta_direct= math.atan2(tm[1] - carrier_pos[1], tm[0] - carrier_pos[0])
        theta_tm_direct = modulo_2Pi(theta_direct + math.pi)
        dist_pass= abs(distance(tm, carrier_pos))
        ball_init_speed = linear_in_interval(dist_pass, 0, 10, 2.0, 8)
        n_phases= [1, 1]
        #Si on peut faire une passe directe
        #Skip Phase 1 -> Recherche limite de zone de passe plus proche du Teammate
        pass_possible, real_intercept= is_pass_possible(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speed, ops_max_speed, tm, ball_init_speed, cf= cf, dt= dt)
        if pass_possible == True:
                n_phases= [2,2]
                if real_intercept is not None and real_intercept not in pass_pos:
                    pass_pos.append(real_intercept)
                elif tm not in pass_pos:
                    pass_pos.append(deepcopy(tm.tolist()))
        # --> PASSE EN MOUVEMENT <--
        theta_tm= [math.degrees(modulo_2Pi(math.radians(theta))) for theta in range(int(math.degrees(theta_tm_direct - math.pi / 2)), int(math.degrees(theta_tm_direct + math.pi/2)), int(math.degrees(math.pi/pass_lines)))]
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
                                dist_pass= abs(distance(bounds[k], carrier_pos))
                                ball_init_speed = linear_in_interval(dist_pass, 0, 10, 2.0, 8)
                                pass_possible, real_intercept= is_pass_possible(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speed, ops_max_speed, bounds[k], ball_init_speed, cf= cf, dt= dt)
                                distTmBound= distance(tm, bounds[k])
                                distCarrierBound= distance(carrier_pos, bounds[k])
                                if real_intercept is not None:
                                    distCarrierBound= distance(carrier_pos, real_intercept)
                                    distTmBound= distance(tm, real_intercept)
                                #Si la position de passe est hors terrain OU que la position de passe est plus proche du Carrier que du Teammate | On stoppe la recherche. Pas de passes possibles pour cet angle, dans cette direction
                                if (abs(bounds[k][0]) > FIELD_WIDTH/2 or abs(bounds[k][1]) > FIELD_HEIGHT/2) or (pass_possible == True and distCarrierBound < distTmBound):
                                    stop_search[k]= True
                                #Sinon, si passe possible | On a trouver la borne la plus proche du Teammate -> Passage à la phase 2, dans cette direction
                                elif pass_possible == True:
                                    phases[k]= 2
                                    bound= bounds[k].tolist()
                                    if real_intercept is not None and real_intercept not in pass_pos:
                                        pass_pos.append(real_intercept)
                                    elif bound not in pass_pos and distance(bounds[k], tm) < distance(bounds[k], carrier_pos):
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
                                    
                                dist_pass= abs(distance(new_bound, carrier_pos))
                                ball_init_speed = linear_in_interval(dist_pass, 0, 10, 2.0, 8)
                                pass_possible, real_intercept= is_pass_possible(carrier_pos, tm, ops_pos, tm_init_speed[i], tm_max_speed[i], ops_init_speed, ops_max_speed, new_bound, ball_init_speed, cf= cf, dt= dt)
                                distTmBound= distance(tm, new_bound)
                                distCarrierBound= distance(carrier_pos, new_bound)
                                if real_intercept is not None:
                                    distCarrierBound= distance(carrier_pos, real_intercept)
                                    distTmBound= distance(tm, real_intercept)
                                #Si on peut faire la passe ET que le nouveau point de passe (theorique) est dans les limites du terrain ET que le point de passe(Theorique ou reel) est plus proche du Teammate que du Carrier
                                    #|On va stocker la nouvelle position de borne (position valide pour une passe)
                                if pass_possible == True and abs(new_bound[0]) < FIELD_WIDTH/2 and abs(new_bound[1]) < FIELD_HEIGHT/2 and distTmBound < distCarrierBound:
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
                                

    return np.array(pass_pos)

if __name__ == "__main__":
    #Dimensions du terrain
    field = np.array([[FIELD_WIDTH/2, -FIELD_HEIGHT/2], 
                        [FIELD_WIDTH/2, FIELD_HEIGHT/2], 
                        [-FIELD_WIDTH/2, FIELD_HEIGHT/2], 
                        [-FIELD_WIDTH/2, -FIELD_HEIGHT/2]], dtype= float)
    #Position du porteur de balle
    carrier = np.array([0, -5], dtype= float)
    #Position du / des coéquipiers (pour en ajouter, ajoute des lignes de la forme [x, y] dans le tableau)
        #Exemple: tms= np.array([[0, 5], [5, 5]], dtype= float) pour 2 coéquipiers
    tms= np.array([[0, 5]], dtype= float)
    #Position du / des adversaires (pour en ajouter, ajoute des lignes de la forme [x, y] dans le tableau)
        #Exemple: ops= np.array([[0, 0], [5, 0]], dtype= float) pour 2 adversaires
    ops= np.array([[8, 0]], dtype= float)
    
    fig,ax = plt.subplots(2,1)
    #Zone du terrain
    p1 = Polygon(field, color= (0, 0.7, 0, 0.4))
    ax[0].add_patch(p1)
    ax[0].scatter([carrier[0]], [carrier[1]], marker='o', color= "black", label= "Carrier")
    ax[0].scatter(tms[:,0], tms[:,1], marker= "o", color= "purple", label= "Teammates")
    ax[0].scatter(ops[:,0], ops[:,1], marker= "o", color= "red", label= "Opponents")
    ax[0].set_xlim([-(FIELD_WIDTH/2) + 0.25,(FIELD_WIDTH/2) + 0.25])
    ax[0].set_ylim([-(FIELD_HEIGHT/2) + 0.25, (FIELD_HEIGHT/2) + 0.25])
    #Vitesse initiale et maximale des coéquipiers et adversaires (en m/s)
    tms_init_speed= np.array([0], dtype= float)
    tms_max_speed= np.array([2.5])
    ops_init_speed= np.array([0], dtype= float)
    ops_max_speed= np.array([2.5], dtype= float)
    pass_pos= compute_possibles_pass(carrier, tms, tms_init_speed, tms_max_speed, ops, ops_init_speed, ops_max_speed,)
    if pass_pos.shape[0] > 0:
        ax[0].scatter(pass_pos[:,0], pass_pos[:,1], label= "Passes possibles")
    ax[0].legend()
    ax[0].set_title("Passes possibles avec vitesse courrante Teammate= " + str(tms_init_speed) + "m/s et vitesse courrante Opponent= " + str(ops_init_speed) + "m/s\n Vitesse maximale Teammates= " + str(tms_max_speed) + "m/s et vitesse maximale Opponents= " + str(ops_max_speed) + "m/s")

    p2 = Polygon(field, color= (0, 0.7, 0, 0.4))
    ax[1].add_patch(p2)
    pass_pos2= compute_possibles_pass_v(carrier, tms, ops)
    if pass_pos2.shape[0] > 0:
        ax[1].scatter(pass_pos2[:,0], pass_pos2[:,1], label= "Passes possibles")
    ax[1].scatter([carrier[0]], [carrier[1]], marker='o', color= "black", label= "Carrier")
    ax[1].scatter(tms[:,0], tms[:,1], marker= "o", color= "purple", label= "Teammates")
    ax[1].scatter(ops[:,0], ops[:,1], marker= "o", color= "red", label= "Opponents")
    ax[1].set_xlim([-(FIELD_WIDTH/2) + 0.25,(FIELD_WIDTH/2) + 0.25])
    ax[1].set_ylim([-(FIELD_HEIGHT/2) + 0.25, (FIELD_HEIGHT/2) + 0.25])
    ax[1].legend()
    ax[1].set_title("Passes possibles sans avoir les données de Vitesses Initiales / Courrantes des Robots et Dynamique de Balle (Utilisation Unique des distances)")
    plt.show()