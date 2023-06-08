import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import correlate
from tabulate import tabulate


def calculate_phase_error(speed1, speed2, c, r):
    # calculate cross-correlation
    cross_correlation = correlate(speed1, speed2)

    # find the shift that yields maximum cross-correlation
    n_star = np.argmax(cross_correlation) - len(speed1) + 1

    # calculate phase error using the provided formula
    phase_err = np.exp((n_star - c) / r)

    return phase_err, n_star


# Read the data from CSV files
carla_heading = pd.read_csv('ego_heading_carla.csv')
carla_speed = pd.read_csv('ego_speed_carla.csv')
carla_traj = pd.read_csv('ego_trajec_carla.csv')
sumo = pd.read_csv('sumo_logged.csv')
cosim = pd.read_csv('cosim_log.csv')

# Correct the offset in the Carla data
corrected_carla_heading = []
for idx, val in enumerate(carla_heading.heading.values):
    if -90 < val < 180:
        corrected_carla_heading.append(val + 90)
    else:
        corrected_carla_heading.append(val + 450)
corrected_carla_heading = np.array(corrected_carla_heading)

corrected_cosim_heading = []
for idx, val in enumerate(cosim.heading.values):
    if -90 < val < 180:
        corrected_cosim_heading.append(val + 90)
    else:
        corrected_cosim_heading.append(val + 450)
corrected_cosim_heading = np.array(corrected_cosim_heading)

sumo_x = sumo.x.values
carla_x = carla_traj.loc_x.values + 503.02
sumo_y = sumo.y.values
carla_y = -carla_traj.loc_y.values + 423.76
cosim_x = cosim.loc_x.values + 503.02
cosim_y = -cosim.loc_y.values + 423.76

carla_timestep = 0.23

# Create time arrays for Carla and SUMO data
carla_time = np.arange(0, len(carla_speed) * carla_timestep, carla_timestep)
sumo_time = sumo.time.values
cosim_time = np.linspace(carla_time[0], carla_time[-1], len(cosim))

# Interpolate Carla and Cosim data to match SUMO time points
# speed
interpolator_speed = interp1d(carla_time, carla_speed.velocity)
interpolated_carla_speed = interpolator_speed(sumo_time)
interpolator_speed_cosim = interp1d(cosim_time, cosim.velocity)
interpolated_cosim_speed = interpolator_speed_cosim(sumo_time)

# heading
interpolator_heading = interp1d(carla_time, corrected_carla_heading)
interpolated_carla_heading = interpolator_heading(sumo_time)
interpolator_heading_cosim = interp1d(cosim_time, corrected_cosim_heading)
interpolated_cosim_heading = interpolator_heading_cosim(sumo_time)

# X
interpolator_x = interp1d(carla_time, carla_x)
interpolated_carla_x = interpolator_x(sumo_time)
interpolator_x_cosim = interp1d(cosim_time, cosim_x)
interpolated_cosim_x = interpolator_x_cosim(sumo_time)

# Y
interpolator_y = interp1d(carla_time, carla_y)
interpolated_carla_y = interpolator_y(sumo_time)
interpolator_y_cosim = interp1d(cosim_time, cosim_y)
interpolated_cosim_y = interpolator_y_cosim(sumo_time)

# Plotting speed
plt.plot(sumo_time, sumo.speed, color='black', label='SUMO Speed')
plt.plot(sumo_time, interpolated_carla_speed, color='darkorange', label='Carla Speed')
plt.plot(sumo_time, interpolated_cosim_speed, color='darkred', label='Co-simulation Speed')
plt.xlabel('Time [s]')
plt.ylabel('Speed [m/s]')
plt.legend()
plt.title('Comparison of Speed in Simulations')
plt.grid(True)
plt.show()

# Plotting heading
plt.plot(sumo_time, sumo.heading, color='black', label='SUMO Heading')
plt.plot(sumo_time, interpolated_carla_heading, color='darkorange', label='Carla Heading')
plt.plot(sumo_time, interpolated_cosim_heading, color='darkred', label='Co-simulation Heading')
plt.xlabel('Time [s]')
plt.ylabel('Heading angle [deg]')
plt.legend()
plt.title('Comparison of Heading in Simulations')
plt.grid(True)
plt.show()

# Plotting trajectory
plt.plot(sumo_x, sumo_y, 'k--', label='SUMO')
plt.plot(carla_x, carla_y, color='darkorange', linestyle='--', label='CARLA')
plt.plot(cosim_x, cosim_y, color='darkred', linestyle='--', label='CO_SIMULATION')
plt.legend()
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Car Trajectories')
plt.grid(True)
plt.show()


# Initialize the ranges for c and r
c_range = np.linspace(0, 100, 500)
r_range = np.linspace(0.001, 100, 500)


# Initialize dictionaries to store the phase errors for cosim, sumo, and Carla data
phase_errors_sumoVScosim = {}
phase_errors_sumoVSCarla = {}
phase_errors_carlaVSCosim = {}

# Compute the phase error for each combination of c and r for cosim data
for c in c_range:
    for r in r_range:
        phase_error_sumoVScosim, n_shift_cosim = calculate_phase_error(sumo.speed.values, interpolated_cosim_speed, c, r)
        phase_errors_sumoVScosim[(c, r)] = phase_error_sumoVScosim

# Compute the phase error for each combination of c and r for sumo data
for c in c_range:
    for r in r_range:
        phase_error_sumoVSCarla, n_shift_sumo = calculate_phase_error(sumo.speed.values, interpolated_carla_speed, c, r)
        phase_errors_sumoVSCarla[(c, r)] = phase_error_sumoVSCarla

# Compute the phase error for each combination of c and r for Carla data
for c in c_range:
    for r in r_range:
        phase_error_carlaVSCosim, n_shift_carla = calculate_phase_error(interpolated_cosim_speed, interpolated_carla_speed, c, r)
        phase_errors_carlaVSCosim[(c, r)] = phase_error_carlaVSCosim

# Find the (c, r) pair with the smallest phase error for cosim, sumo, and Carla data
best_c_sumoVScosim, best_r_sumoVScosim = min(phase_errors_sumoVScosim, key=phase_errors_sumoVScosim.get)
best_c_sumoVSCarla, best_r_sumoVSCarla = min(phase_errors_sumoVSCarla, key=phase_errors_sumoVSCarla.get)
best_c_carlaVSCosim, best_r_carlaVSCosim = min(phase_errors_carlaVSCosim, key=phase_errors_carlaVSCosim.get)

# Create a table to store the results
table = [
    ["Method", "C", "R", "n*", "Phase Error"],
    ["SUMO VS CARLA", best_c_sumoVSCarla, best_r_sumoVSCarla, n_shift_sumo, phase_errors_sumoVSCarla[(best_c_sumoVSCarla, best_r_sumoVSCarla)]],
    ["SUMO VS COSIM", best_c_sumoVScosim, best_r_sumoVScosim, n_shift_cosim, phase_errors_sumoVScosim[(best_c_sumoVScosim, best_r_sumoVScosim)]],
    ["CARLA VS COSIM", best_c_carlaVSCosim, best_r_carlaVSCosim, n_shift_carla, phase_errors_carlaVSCosim[(best_c_carlaVSCosim, best_r_carlaVSCosim)]]
]

# Print the table
print(tabulate(table, headers="firstrow"))

# Initialize dictionaries to store the phase errors for cosim, sumo, and Carla data
phase_errors_sumo_headingVScosim = {}
phase_errors_sumo_headingVSCarla = {}
phase_errors_carla_headingVSCosim = {}

# Compute the phase error for each combination of c and r for cosim data (heading values)
for c in c_range:
    for r in r_range:
        phase_error_sumo_headingVScosim, n_shift_cosim_heading = calculate_phase_error(sumo.heading.values, interpolated_cosim_heading, c, r)
        phase_errors_sumo_headingVScosim[(c, r)] = phase_error_sumo_headingVScosim

# Compute the phase error for each combination of c and r for sumo data (heading values)
for c in c_range:
    for r in r_range:
        phase_error_sumo_headingVSCarla, n_shift_sumo_heading = calculate_phase_error(sumo.heading.values, interpolated_carla_heading, c, r)
        phase_errors_sumo_headingVSCarla[(c, r)] = phase_error_sumo_headingVSCarla

# Compute the phase error for each combination of c and r for Carla data (heading values)
for c in c_range:
    for r in r_range:
        phase_error_carla_headingVSCosim, n_shift_carla_heading = calculate_phase_error(interpolated_cosim_heading, interpolated_carla_heading, c, r)
        phase_errors_carla_headingVSCosim[(c, r)] = phase_error_carla_headingVSCosim

# Find the (c, r) pair with the smallest phase error for cosim, sumo, and Carla data (heading values)
best_c_sumo_headingVScosim, best_r_sumo_headingVScosim = min(phase_errors_sumo_headingVScosim, key=phase_errors_sumo_headingVScosim.get)
best_c_sumo_headingVSCarla, best_r_sumo_headingVSCarla = min(phase_errors_sumo_headingVSCarla, key=phase_errors_sumo_headingVSCarla.get)
best_c_carla_headingVSCosim, best_r_carla_headingVSCosim = min(phase_errors_carla_headingVSCosim, key=phase_errors_carla_headingVSCosim.get)

# Create a table to store the results (heading values)
table_heading = [
    ["Method", "C", "R", "n*", "Phase Error"],
    ["SUMO VS CARLA", best_c_sumo_headingVSCarla, best_r_sumo_headingVSCarla, n_shift_sumo_heading, phase_errors_sumo_headingVSCarla[(best_c_sumo_headingVSCarla, best_r_sumo_headingVSCarla)]],
    ["SUMO VS COSIM", best_c_sumo_headingVScosim, best_r_sumo_headingVScosim, n_shift_cosim_heading, phase_errors_sumo_headingVScosim[(best_c_sumo_headingVScosim, best_r_sumo_headingVScosim)]],
    ["CARLA VS COSIM", best_c_carla_headingVSCosim, best_r_carla_headingVSCosim, n_shift_carla_heading, phase_errors_carla_headingVSCosim[(best_c_carla_headingVSCosim, best_r_carla_headingVSCosim)]]
]

# Print the table (heading values)
print(tabulate(table_heading, headers="firstrow"))

# Initialize dictionaries to store the phase errors for cosim, sumo, and Carla data
phase_errors_sumo_xVScosim_x = {}
phase_errors_sumo_xVSCarla_x = {}
phase_errors_carla_xVSCosim_x = {}

phase_errors_sumo_yVScosim_y = {}
phase_errors_sumo_yVSCarla_y = {}
phase_errors_carla_yVSCosim_y = {}

# Compute the phase error for each combination of c and r for cosim data (x locations)
for c in c_range:
    for r in r_range:
        phase_error_sumo_xVScosim_x, n_shift_cosim_x = calculate_phase_error(sumo_x, interpolated_cosim_x, c, r)
        phase_errors_sumo_xVScosim_x[(c, r)] = phase_error_sumo_xVScosim_x

# Compute the phase error for each combination of c and r for sumo data (x locations)
for c in c_range:
    for r in r_range:
        phase_error_sumo_xVSCarla_x, n_shift_sumo_x = calculate_phase_error(sumo_x, interpolated_carla_x, c, r)
        phase_errors_sumo_xVSCarla_x[(c, r)] = phase_error_sumo_xVSCarla_x

# Compute the phase error for each combination of c and r for Carla data (x locations)
for c in c_range:
    for r in r_range:
        phase_error_carla_xVSCosim_x, n_shift_carla_x = calculate_phase_error(interpolated_cosim_x, interpolated_carla_x, c, r)
        phase_errors_carla_xVSCosim_x[(c, r)] = phase_error_carla_xVSCosim_x

# Compute the phase error for each combination of c and r for cosim data (y locations)
for c in c_range:
    for r in r_range:
        phase_error_sumo_yVScosim_y, n_shift_cosim_y = calculate_phase_error(sumo_y, interpolated_cosim_y, c, r)
        phase_errors_sumo_yVScosim_y[(c, r)] = phase_error_sumo_yVScosim_y

# Compute the phase error for each combination of c and r for sumo data (y locations)
for c in c_range:
    for r in r_range:
        phase_error_sumo_yVSCarla_y, n_shift_sumo_y = calculate_phase_error(sumo_y, interpolated_carla_y, c, r)
        phase_errors_sumo_yVSCarla_y[(c, r)] = phase_error_sumo_yVSCarla_y

# Compute the phase error for each combination of c and r for Carla data (y locations)
for c in c_range:
    for r in r_range:
        phase_error_carla_yVSCosim_y, n_shift_carla_y = calculate_phase_error(interpolated_cosim_y, interpolated_carla_y, c, r)
        phase_errors_carla_yVSCosim_y[(c, r)] = phase_error_carla_yVSCosim_y

# Find the (c, r) pair with the smallest phase error for cosim, sumo, and Carla data (x locations)
best_c_sumo_xVScosim_x, best_r_sumo_xVScosim_x = min(phase_errors_sumo_xVScosim_x, key=phase_errors_sumo_xVScosim_x.get)
best_c_sumo_xVSCarla_x, best_r_sumo_xVSCarla_x = min(phase_errors_sumo_xVSCarla_x, key=phase_errors_sumo_xVSCarla_x.get)
best_c_carla_xVSCosim_x, best_r_carla_xVSCosim_x = min(phase_errors_carla_xVSCosim_x, key=phase_errors_carla_xVSCosim_x.get)

# Find the (c, r) pair with the smallest phase error for cosim, sumo, and Carla data (y locations)
best_c_sumo_yVScosim_y, best_r_sumo_yVScosim_y = min(phase_errors_sumo_yVScosim_y, key=phase_errors_sumo_yVScosim_y.get)
best_c_sumo_yVSCarla_y, best_r_sumo_yVSCarla_y = min(phase_errors_sumo_yVSCarla_y, key=phase_errors_sumo_yVSCarla_y.get)
best_c_carla_yVSCosim_y, best_r_carla_yVSCosim_y = min(phase_errors_carla_yVSCosim_y, key=phase_errors_carla_yVSCosim_y.get)

# Create a table to store the results (x locations)
table_x = [
    ["Method", "C", "R", "n*", "Phase Error"],
    ["SUMO VS CARLA", best_c_sumo_xVSCarla_x, best_r_sumo_xVSCarla_x, n_shift_sumo_x, phase_errors_sumo_xVSCarla_x[(best_c_sumo_xVSCarla_x, best_r_sumo_xVSCarla_x)]],
    ["SUMO VS COSIM", best_c_sumo_xVScosim_x, best_r_sumo_xVScosim_x, n_shift_cosim_x, phase_errors_sumo_xVScosim_x[(best_c_sumo_xVScosim_x, best_r_sumo_xVScosim_x)]],
    ["CARLA VS COSIM", best_c_carla_xVSCosim_x, best_r_carla_xVSCosim_x, n_shift_carla_x, phase_errors_carla_xVSCosim_x[(best_c_carla_xVSCosim_x, best_r_carla_xVSCosim_x)]]
]

# Print the table (x locations)
print(tabulate(table_x, headers="firstrow"))

# Create a table to store the results (y locations)
table_y = [
    ["Method", "C", "R", "n*", "Phase Error"],
    ["SUMO VS CARLA", best_c_sumo_yVSCarla_y, best_r_sumo_yVSCarla_y, n_shift_sumo_y, phase_errors_sumo_yVSCarla_y[(best_c_sumo_yVSCarla_y, best_r_sumo_yVSCarla_y)]],
    ["SUMO VS COSIM", best_c_sumo_yVScosim_y, best_r_sumo_yVScosim_y, n_shift_cosim_y, phase_errors_sumo_yVScosim_y[(best_c_sumo_yVScosim_y, best_r_sumo_yVScosim_y)]],
    ["CARLA VS COSIM", best_c_carla_yVSCosim_y, best_r_carla_yVSCosim_y, n_shift_carla_y, phase_errors_carla_yVSCosim_y[(best_c_carla_yVSCosim_y, best_r_carla_yVSCosim_y)]]
]

# Print the table (y locations)
print(tabulate(table_y, headers="firstrow"))
