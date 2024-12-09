import numpy as np
import matplotlib.pyplot as plt

def bouncing_ball_simulation(m, C_r, e, dt=0.01, h_initial=10):
    g = 9.81  # acceleration due to gravity in m/s^2
    v = 0  # initial vertical velocity
    y = h_initial  # initial height
    t = 0  # initial time

    # Lists to store the trajectory points
    times = []
    heights = []
    velocities = []
    i= 0
    # Loop until the ball comes to rest
    while i < 500:
        # Update time and position
        t += dt
        # Update vertical velocity
        v += -g * dt  # falling down due to gravity

        # Update height
        y += v * dt

        # Check for bounce
        if y <= 0:
            # Ball has hit the ground
            #y = 0  # Reset height to ground level
            v = e * v  # Reverse velocity and apply coefficient of restitution

            # After the bounce, the ball starts rolling
            # Calculate the rolling resistance deceleration
            """a_rolling = -C_r * g
            
            # Simulate rolling for a short duration after the bounce
            while v > 0:  # Continue until the ball stops rolling
                # Store the results
                times.append(t)
                heights.append(y)
                velocities.append(v)

                # Update position and velocity while rolling
                y = 0  # Height remains at ground level while rolling
                v += a_rolling * dt  # Apply rolling resistance
                t += dt"""
        i+= 1
        # Store the results
        times.append(t)
        heights.append(y)
        velocities.append(v)

    return times, heights, velocities

def plot_trajectory(times, heights, velocities):
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.plot(times, heights)
    plt.title('Height of the Ball Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Height (m)')
    plt.grid()

    plt.subplot(1, 2, 2)
    plt.plot(times, velocities)
    plt.title('Velocity of the Ball Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid()

    plt.tight_layout()
    plt.show()

# Example parameters
mass = 0.43  # mass of a soccer ball in kg
C_r = 0.02  # coefficient of rolling resistance
e = 0.6  # coefficient of restitution (bounciness)
initial_height = 10  # initial height in meters

# Run the simulation
times, heights, velocities = bouncing_ball_simulation(mass, C_r, e, h_initial=initial_height)

# Plot the results
plot_trajectory(times, heights, velocities)