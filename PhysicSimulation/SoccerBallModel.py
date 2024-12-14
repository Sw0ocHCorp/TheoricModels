import numpy as np
import matplotlib.pyplot as plt
import math
from copy import deepcopy

RHO= 1.2
A= 0.0333

def modelizeBallKick(p, theta0, v0, circonference= 0.7, m= 0.45, g=-9.81, restitution= 0.6, drag= 0.3,dt= 1/50):
    pos= [p]
    vi= [v0]
    r= circonference/(2*np.pi)
    vx= v0*math.cos(theta0)
    vy= v0*math.sin(theta0)
    v= v0
    x= p[0]
    y= p[1]
    theta= theta0
    i=0
    while v > 0.1:

        x+= vx*dt
        y+= vy*dt + (1/2)*g*(dt**2)
        if vy != 0:
            vy+= g*dt
        theta = math.atan2(y - pos[i][1], x - pos[i][0])
        #Rebound or Rolling
        if y <= 0:
            y= 0
            #Rebound
            if vy != 0:
                v-= ((1/2)*RHO*A*(v**2)*drag)*dt
                theta *= -1
                v *= restitution
                vx= v * math.cos(theta)
                vy= v * math.sin(theta)
            #Rolling
            if vy == 0 and vx != 0:
                v-= (0.3*(-g))*dt
                theta= 0
                vx= v * math.cos(theta)
        #Ball in the Air
        else:
            v= vx / math.cos(theta)
        pos.append([x, y]) 
        vi.append(v)
        i+=1
    return pos, vi

if __name__ == "__main__":
    init_pos= [0, 0]
    theta= 0
    init_speed= 6
    pos, vi= modelizeBallKick(init_pos, theta, init_speed)
    pos= np.array(pos)
    fig= plt.figure()
    ax= fig.add_subplot(121)
    ax.plot(pos[:, 0], pos[:, 1])
    ax.set_title(f"Ball Trajectory: Initial angle:{theta} rad | Initial speed:{init_speed} m/s")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax= fig.add_subplot(122)
    ax.plot([i for i in range(len(vi))], vi)
    ax.set_title("Ball Speed")
    ax.set_xlabel("TimeSteps")
    plt.show()
    
