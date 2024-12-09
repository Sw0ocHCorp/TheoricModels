import math
from copy import deepcopy

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
        ball_speed -= cf*9.8*dt
        vx= ball_speed*math.cos(theta)
        vy= ball_speed*math.sin(theta)
        pos_history.append(deepcopy(ball_pos))
        if (vx < 0.05 and vy < 0.05):
            break
    return pos_history

def calculer_temps_distance(v0, theta, d, cf):
    """
    Calcule le temps nécessaire pour qu'un ballon parcoure une distance donnée
    dans un plan en tenant compte de la décélération due aux frottements.

    :param v0: Vitesse initiale du ballon (m/s)
    :param theta: Angle initial (rad)
    :param d: Distance à parcourir (m)
    :param cf: Coefficient de frottement (sans unité)
    :return: Temps nécessaire pour parcourir la distance (s)
    """
    # Accélération due aux frottements
    a_f = cf * 9.8  # m/s^2

    # Temps nécessaire pour que la vitesse atteigne zéro
    t_arret = v0 / a_f

    # Distance totale parcourue jusqu'à l'arrêt
    d_arret = (v0 ** 2) / (2 * a_f)

    if d > d_arret:
        raise ValueError("La distance donnée dépasse la distance maximale que le ballon peut parcourir.")

    # Équation de la distance en fonction du temps
    # d(t) = v0 * t - 0.5 * a_f * t^2
    # On réarrange pour résoudre d = v0 * t - 0.5 * a_f * t^2
    # 0.5 * a_f * t^2 - v0 * t + d = 0
    # Forme quadratique : a * t^2 + b * t + c = 0
    a = 0.5 * a_f
    b = -v0
    c = d

    # Résolution de l'équation quadratique
    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0:
        raise ValueError("Erreur dans le calcul : discriminant négatif.")

    # Les solutions de l'équation quadratique
    t1 = (-b - math.sqrt(discriminant)) / (2 * a)
    t2 = (-b + math.sqrt(discriminant)) / (2 * a)

    # On retourne la solution positive et réaliste
    if t1 >= 0:
        return t1
    elif t2 >= 0:
        return t2
    else:
        raise ValueError("Aucune solution temporelle valide trouvée.")

def calculer_temps_et_vitesse(position, acceleration, vitesse_initiale, vitesse_max):
    """
    Calcule le temps pour atteindre une position avec contrainte de vitesse maximale.
    
    :param position: Position finale (en mètres)
    :param acceleration: Accélération (en m/s²)
    :param vitesse_max: Vitesse maximale autorisée (en m/s)
    :return: Temps total et vitesse finale
    """
    # Temps pour atteindre la vitesse maximale
    t_max = (vitesse_max - vitesse_initiale) / acceleration
    
    # Distance parcourue jusqu'à la vitesse maximale
    x_max = 0.5 * acceleration * (t_max ** 2)
    
    if x_max >= position:
        # On n'atteint pas la vitesse maximale
        temps = math.sqrt((2 * position) / acceleration)
        vitesse_finale = acceleration * temps
    else:
        # On atteint la vitesse maximale
        t1 = t_max  # Temps pour atteindre la vitesse maximale
        x1 = x_max  # Distance parcourue jusqu'à la vitesse maximale
        
        # Temps supplémentaire à vitesse constante
        t2 = (position - x1) / vitesse_max
        
        temps = t1 + t2
        vitesse_finale = vitesse_max
    
    return temps, vitesse_finale

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

def distance(pt1, pt2):
     if (pt1 is not None and pt2 is not None):
         return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
     else:
         return math.inf

# Exemple d'utilisation
position = 10  # mètres
acceleration = 10  # m/s²
vitesse_initiale = 5  # m/s
vitesse_max = 10  # m/s

v0 = 50  # Vitesse initiale en m/s
theta = math.radians(45)  # Angle initial en radians
cf = 0.3  # Coefficient de frottement
dt= 1/50
pos_history= compute_ball_dynamics([0, 0], theta, v0, cf, dt)
dist= distance(pos_history[0], pos_history[-1])
print(f"Distance parcourue : {dist:.2f} mètres en {len(pos_history)*dt} secondes")
temps = calculer_temps_distance(v0, theta, int(dist), cf)
print(f"Temps pour parcourir {int(dist)} mètres : {temps:.2f} secondes")

temps, vitesse = calculer_temps_et_vitesse(position, acceleration, vitesse_initiale, vitesse_max)
print(f"Temps total : {temps:.2f} secondes")
print(f"Vitesse finale : {vitesse:.2f} m/s")
print(compute_time_to_position([math.cos(math.pi/4), math.sin(math.pi/4)], 5, 10, [11*math.cos(math.pi/4), 11*math.sin(math.pi/4)]))