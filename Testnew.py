import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline

def generate_velocity_profile(spline, v0, vf, vmax, amax, num_points=1000):
    # Calcul longueur trajectoire
    D = spline.integrate(spline.x[0], spline.x[-1])

    # Calcul des phases du mouvement
    ta = (vmax - v0) / amax
    td = (vmax - vf) / amax
    Da = v0 * ta + 0.5 * amax * ta**2
    Dd = vf * td + 0.5 * amax * td**2
    Dc = D - (Da + Dd)

    if Dc > 0:
        tc = Dc / vmax
        T = ta + tc + td
    else:
        # Ajustement de la vitesse max
        vmax = np.sqrt(2 * amax * D + (v0**2 + vf**2) / 2)
        ta = (vmax - v0) / amax
        td = (vmax - vf) / amax
        T = ta + td

    # Profil de vitesse :333 kawaiii desune
    t_profile = np.linspace(0, T, num_points)
    v_profile = np.piecewise(t_profile,
                             [t_profile < ta,
                              (ta <= t_profile) & (t_profile < T - td),
                              t_profile >= T - td],
                             [lambda t: v0 + amax * t,
                              lambda t: vmax,
                              lambda t: vf + amax * (T - t)])

    return t_profile, v_profile

# Waypoints
x = np.array([0, 1, 2, 3])
y = np.array([0, 2, 1, 3])
dydx = np.array([1, 0, -1, 2])  # Dérivées aux waypoints

# Coucou la spline c'est davidlafargepokemon
spline = CubicHermiteSpline(x, y, dydx)

# Pramètres de la spline
v0 = 0.5  # vitesse initiale
vf = 0.3  # vitesse finale
vmax = 2.0  # vitesse maximale
amax = 1.0  # accélération maximale

# Génération du profil de vitesse
t, v = generate_velocity_profile(spline, v0, vf, vmax, amax)

# Visualissation des résultats (merci GPT je sais pas plot avec matplotlib)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

x_new = np.linspace(x[0], x[-1], 100)
y_new = spline(x_new)
ax1.plot(x, y, 'ro', label='Points de contrôle')
ax1.plot(x_new, y_new, 'b-', label='CubicHermiteSpline')
ax1.set_title('CubicHermiteSpline')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.legend()
ax1.grid(True)

ax2.plot(t, v, 'r-')
ax2.set_title('Profil de vitesse')
ax2.set_xlabel('Temps')
ax2.set_ylabel('Vitesse')
ax2.grid(True)

plt.tight_layout()
plt.show()
