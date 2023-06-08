import numpy as np
from scipy.signal import correlate
from scipy.interpolate import interp1d
from scipy.ndimage import convolve1d
import matplotlib.pyplot as plt
import pandas as pd
from tabulate import tabulate



def dtw(s, t, cost_function):
    n, m = len(s), len(t)
    dtw_matrix = np.zeros((n+1, m+1))
    for i in range(n+1):
        for j in range(m+1):
            dtw_matrix[i, j] = np.inf
    dtw_matrix[0, 0] = 0

    for i in range(1, n+1):
        for j in range(1, m+1):
            cost = cost_function(s[i-1], t[j-1], i, j, derivative_s[i-1], derivative_ca[j - 1])
            dtw_matrix[i, j] = cost + min(dtw_matrix[i-1, j], dtw_matrix[i, j-1], dtw_matrix[i-1, j-1])

    # Backtrack to find the optimal warping path
    i, j = n, m
    path = []
    while i > 0 and j > 0:
        path.append((i-1, j-1))
        if dtw_matrix[i-1, j-1] == min(dtw_matrix[i-1, j-1], dtw_matrix[i-1, j], dtw_matrix[i, j-1]):
            i, j = i-1, j-1
        elif dtw_matrix[i-1, j] == min(dtw_matrix[i-1, j-1], dtw_matrix[i-1, j], dtw_matrix[i, j-1]):
            i -= 1
        else:
            j -= 1
    path.append((0, 0))

    return dtw_matrix[-1, -1], path[::-1]  # Reverse the path so it goes from start to end


def calculate_magnitude_error(Ats, Bts, warped_path):
    Atsw = [Ats[i] for i, _ in warped_path]
    Btsw = [Bts[i] for _, i in warped_path]

    magnitude_error = np.sum(np.abs(np.array(Atsw) - np.array(Btsw))) / np.sum(np.abs(np.array(Btsw)))

    return magnitude_error


def cost_function(ai, bi, ti, tj, dAts_dt_i, dBts_dt_i):
    return ((ai - bi)**2 + (ti - tj)**2) * abs(dAts_dt_i - dBts_dt_i)


def calculate_topology_error(Ats, Bts, warped_path):
    Ats_derivative = np.gradient(Ats)
    Bts_derivative = np.gradient(Bts)

    Atsw = [Ats_derivative[i] for i, _ in warped_path]
    Btsw = [Bts_derivative[i] for _, i in warped_path]

    topology_error = np.sum(np.abs(np.array(Atsw) - np.array(Btsw))) / np.sum(np.abs(np.array(Btsw)))

    return topology_error


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

# Calculate the derivatives
derivative_s = np.gradient(sumo.speed.values)
derivative_ca = np.gradient(interpolated_carla_speed)
derivative_co = np.gradient(interpolated_cosim_speed)
derivative_s_h = np.gradient(sumo.heading.values)
derivative_ca_h = np.gradient(interpolated_carla_heading)
derivative_co_h = np.gradient(interpolated_cosim_heading)
derivative_s_x = np.gradient(sumo_x)
derivative_ca_x = np.gradient(interpolated_carla_x)
derivative_co_x = np.gradient(interpolated_cosim_x)
derivative_s_y = np.gradient(sumo_y)
derivative_ca_y = np.gradient(interpolated_carla_y)
derivative_co_y = np.gradient(interpolated_cosim_y)

# DTW for speed
sumoVScarla_dtw_distance, sVca_path = dtw(sumo.speed.values, interpolated_carla_speed, cost_function)
sumoVScosim_dtw_distance, sVco_path = dtw(sumo.speed.values, interpolated_cosim_speed, cost_function)
carlaVScosim_dtw_distance, caVco_path = dtw(interpolated_carla_speed, interpolated_cosim_speed, cost_function)

# DTW for heading
sumoVScarla_dtw_distance_heading, sVca_path_heading = dtw(sumo.heading.values, interpolated_carla_heading, cost_function)
sumoVScosim_dtw_distance_heading, sVco_path_heading = dtw(sumo.heading.values, interpolated_cosim_heading, cost_function)
carlaVScosim_dtw_distance_heading, caVco_path_heading = dtw(interpolated_carla_heading, interpolated_cosim_heading, cost_function)

# DTW for x locations
sumoVScarla_dtw_distance_x, sVca_path_x = dtw(sumo_x, carla_x, cost_function)
sumoVScosim_dtw_distance_x, sVco_path_x = dtw(sumo_x, cosim_x, cost_function)
carlaVScosim_dtw_distance_x, caVco_path_x = dtw(carla_x, cosim_x, cost_function)

# DTW for y locations
sumoVScarla_dtw_distance_y, sVca_path_y = dtw(sumo_y, carla_y, cost_function)
sumoVScosim_dtw_distance_y, sVco_path_y = dtw(sumo_y, cosim_y, cost_function)
carlaVScosim_dtw_distance_y, caVco_path_y = dtw(carla_y, cosim_y, cost_function)


# Magnitude error speed
magnitude_error_sVca = calculate_magnitude_error(sumo.speed.values, interpolated_carla_speed, sVca_path)
magnitude_error_sVco = calculate_magnitude_error(sumo.speed.values, interpolated_cosim_speed, sVco_path)
magnitude_error_caVco = calculate_magnitude_error(interpolated_carla_speed, interpolated_cosim_speed, caVco_path)

# Magnitude error for heading
magnitude_error_sVca_heading = calculate_magnitude_error(sumo.heading.values, interpolated_carla_heading, sVca_path_heading)
magnitude_error_sVco_heading = calculate_magnitude_error(sumo.heading.values, interpolated_cosim_heading, sVco_path_heading)
magnitude_error_caVco_heading = calculate_magnitude_error(interpolated_carla_heading, interpolated_cosim_heading, caVco_path_heading)

# Magnitude error for x locations
magnitude_error_sVca_x = calculate_magnitude_error(sumo_x, carla_x, sVca_path_x)
magnitude_error_sVco_x = calculate_magnitude_error(sumo_x, cosim_x, sVco_path_x)
magnitude_error_caVco_x = calculate_magnitude_error(carla_x, cosim_x, caVco_path_x)

# Magnitude error for y locations
magnitude_error_sVca_y = calculate_magnitude_error(sumo_y, carla_y, sVca_path_y)
magnitude_error_sVco_y = calculate_magnitude_error(sumo_y, cosim_y, sVco_path_y)
magnitude_error_caVco_y = calculate_magnitude_error(carla_y, cosim_y, caVco_path_y)


# Topology error for speed
topology_error_sVca_speed = calculate_topology_error(sumo.speed.values, interpolated_carla_speed, sVca_path)
topology_error_sVco_speed = calculate_topology_error(sumo.speed.values, interpolated_cosim_speed, sVco_path)
topology_error_caVco_speed = calculate_topology_error(interpolated_carla_speed, interpolated_cosim_speed, caVco_path)

# Topology error for heading
topology_error_sVca_heading = calculate_topology_error(sumo.heading.values, interpolated_carla_heading, sVca_path_heading)
topology_error_sVco_heading = calculate_topology_error(sumo.heading.values, interpolated_cosim_heading, sVco_path_heading)
topology_error_caVco_heading = calculate_topology_error(interpolated_carla_heading, interpolated_cosim_heading, caVco_path_heading)

# Topology error for x locations
topology_error_sVca_x = calculate_topology_error(sumo_x, carla_x, sVca_path_x)
topology_error_sVco_x = calculate_topology_error(sumo_x, cosim_x, sVco_path_x)
topology_error_caVco_x = calculate_topology_error(carla_x, cosim_x, caVco_path_x)

# Topology error for y locations
topology_error_sVca_y = calculate_topology_error(sumo_y, carla_y, sVca_path_y)
topology_error_sVco_y = calculate_topology_error(sumo_y, cosim_y, sVco_path_y)
topology_error_caVco_y = calculate_topology_error(carla_y, cosim_y, caVco_path_y)

# Create a table to store the results
table = [
    ["Simulations", "DTW Distance", "Magnitude Error", "Topology Error"],
    ["Speed: SUMO vs Carla", sumoVScarla_dtw_distance, magnitude_error_sVca, topology_error_sVca_speed],
    ["Speed: SUMO vs Co-simulation", sumoVScosim_dtw_distance, magnitude_error_sVco, topology_error_sVco_speed],
    ["Speed: Carla vs Co-simulation", carlaVScosim_dtw_distance, magnitude_error_caVco, topology_error_caVco_speed],
    ["Heading: SUMO vs Carla", sumoVScarla_dtw_distance_heading, magnitude_error_sVca_heading, topology_error_sVca_heading],
    ["Heading: SUMO vs Co-simulation", sumoVScosim_dtw_distance_heading, magnitude_error_sVco_heading, topology_error_sVco_heading],
    ["Heading: Carla vs Co-simulation", carlaVScosim_dtw_distance_heading, magnitude_error_caVco_heading, topology_error_caVco_heading],
    ["X: SUMO vs Carla", sumoVScarla_dtw_distance_x, magnitude_error_sVca_x, topology_error_sVca_x],
    ["X: SUMO vs Co-simulation", sumoVScosim_dtw_distance_x, magnitude_error_sVco_x, topology_error_sVco_x],
    ["X: Carla vs Co-simulation", carlaVScosim_dtw_distance_x, magnitude_error_caVco_x, topology_error_caVco_x],
    ["Y: SUMO vs Carla", sumoVScarla_dtw_distance_y, magnitude_error_sVca_y, topology_error_sVca_y],
    ["Y: SUMO vs Co-simulation", sumoVScosim_dtw_distance_y, magnitude_error_sVco_y, topology_error_sVco_y],
    ["Y: Carla vs Co-simulation", carlaVScosim_dtw_distance_y, magnitude_error_caVco_y, topology_error_caVco_y]
]

# Print the table
print(tabulate(table, headers="firstrow"))
