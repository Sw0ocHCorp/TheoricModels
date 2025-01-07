import numpy as np
import matplotlib.pyplot as plt
import math

# Points de contrôle et vecteurs tangents
P0 = np.array([0, 0])
P1 = np.array([1.0, 1.0])
T0 = np.array([math.cos(math.pi/4), math.sin(math.pi/4)])
T1 = np.array([math.cos(-math.pi/4), math.sin(-math.pi/4)])

# Fonction pour calculer P(t)
def hermite_cubic_spline(t, P0, P1, T0, T1):
    h00 = 2*t**3 - 3*t**2 + 1
    h10 = t**3 - 2*t**2 + t
    h01 = -2*t**3 + 3*t**2
    h11 = t**3 - t**2
    return h00 * P0 + h10 * T0 + h01 * P1 + h11 * T1

# Paramètres
delta_t = 1/50
t = 0.0

# Liste pour stocker les points
points = []
point= P0
# Itération
while np.sqrt((P1[0] - point[0])**2 + (P1[1] - point[1])**2) > 1e-6:
    point = hermite_cubic_spline(t, P0, P1, T0, T1)
    points.append(point)
    t += delta_t

# Conversion en array pour le tracé
points = np.array(points)

# Tracé de la spline
plt.plot(points[:, 0], points[:, 1], '-o', label="Spline Hermite Cubique")
plt.scatter([P0[0], P1[0]], [P0[1], P1[1]], color='red', label="Points de contrôle")
plt.quiver(P0[0], P0[1], T0[0], T0[1], angles='xy', scale_units='xy', scale=1, color='green', label="Tangente en P0")
plt.quiver(P1[0], P1[1], T1[0], T1[1], angles='xy', scale_units='xy', scale=1, color='blue', label="Tangente en P1")
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Spline Hermite Cubique')
plt.grid(True)
plt.show()