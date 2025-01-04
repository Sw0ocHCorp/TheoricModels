import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

x = np.linspace(0, 10, 100)
y = np.sin(x)
z = x  # valeurs pour la colormap

points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

norm = plt.Normalize(z.min(), z.max())
lc = LineCollection(segments, cmap='viridis', norm=norm)
lc.set_array(z)

fig, ax = plt.subplots()
ax.add_collection(lc)
ax.set_xlim(x.min(), x.max())
ax.set_ylim(y.min(), y.max())
plt.colorbar(lc)
plt.show()