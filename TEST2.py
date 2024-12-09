import math

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

temps, vitesse = calculer_temps_et_vitesse(position, acceleration, vitesse_initiale, vitesse_max)
print(f"Temps total : {temps:.2f} secondes")
print(f"Vitesse finale : {vitesse:.2f} m/s")
print(compute_time_to_position([math.cos(math.pi/4), math.sin(math.pi/4)], 5, 10, [11*math.cos(math.pi/4), 11*math.sin(math.pi/4)]))