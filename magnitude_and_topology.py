import numpy as np
from scipy.signal import correlate
from scipy.interpolate import interp1d
from scipy.ndimage import convolve1d
import matplotlib.pyplot as plt
import pandas as pd


def dtw(s, t, cost_function):
    n, m = len(s), len(t)
    dtw_matrix = np.zeros((n+1, m+1))
    for i in range(n+1):
        for j in range(m+1):
            dtw_matrix[i, j] = np.inf
    dtw_matrix[0, 0] = 0

    for i in range(1, n+1):
        for j in range(1, m+1):
            cost = cost_function(s[i-1], t[j-1], i, j, derivative_s[i-1], derivative_t[j-1])
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
# Correct the offset in the Carla data
corrected_carla_heading = []
for idx, val in enumerate(carla_heading.heading.values):
    if -90 < val < 180:
        corrected_carla_heading.append(val + 90)
    else:
        corrected_carla_heading.append(val + 450)
corrected_carla_heading = np.array(corrected_carla_heading)


sumo_x = sumo.x.values
carla_x = carla_traj.loc_x.values + 503.02
sumo_y = sumo.y.values
carla_y = -carla_traj.loc_y.values + 423.76

carla_timestep = 0.23

# Create time arrays for Carla and SUMO data
carla_time = np.arange(0, len(carla_speed) * carla_timestep, carla_timestep)
sumo_time = sumo.time.values

# Adjust Carla data to match the time range of SUMO data
carla_time_adjusted = np.linspace(carla_time[0], carla_time[-1], len(sumo_time))

# Interpolate Carla data to match SUMO time points
# speed
interpolator_speed = interp1d(carla_time, carla_speed.velocity)
interpolated_carla_speed = interpolator_speed(carla_time_adjusted)
# heading
interpolator_heading = interp1d(carla_time, corrected_carla_heading)
interpolated_carla_heading = interpolator_heading(carla_time_adjusted)
# X
interpolator_heading = interp1d(carla_time, carla_x)
interpolated_carla_x = interpolator_heading(carla_time_adjusted)
# Y
interpolator_heading = interp1d(carla_time, carla_y)
interpolated_carla_y = interpolator_heading(carla_time_adjusted)

# Calculate the derivatives
derivative_s = np.gradient(sumo.speed.values)
derivative_t = np.gradient(interpolated_carla_speed)
derivative_s_h = np.gradient(sumo.heading.values)
derivative_t_h = np.gradient(interpolated_carla_heading)
derivative_s_x = np.gradient(sumo_x)
derivative_t_x = np.gradient(interpolated_carla_x)
derivative_s_y = np.gradient(sumo_y)
derivative_t_y = np.gradient(interpolated_carla_y)

# Compute the DTW distance and the optimal warping path
dtw_distance, path = dtw(sumo.speed.values, interpolated_carla_speed, cost_function)
dtw_distance_h, path_h = dtw(sumo.heading.values, interpolated_carla_heading, cost_function)
dtw_distance_x, path_x = dtw(sumo_x, interpolated_carla_x, cost_function)
dtw_distance_y, path_y = dtw(sumo_y, interpolated_carla_y, cost_function)

print("SPEED")
print('DTW distance: ', dtw_distance)

# Compute the magnitude error
magnitude_error = calculate_magnitude_error(sumo.speed.values, interpolated_carla_speed, path)

print('Magnitude error: ', magnitude_error)

# Compute the topology error
topology_error = calculate_topology_error(sumo.speed.values, interpolated_carla_speed, path)

print('Topology error: ', topology_error)

print("HEADING")
print('DTW distance: ', dtw_distance_h)

# Compute the magnitude error
magnitude_error_h = calculate_magnitude_error(sumo.heading.values, interpolated_carla_heading, path_h)

print('Magnitude error: ', magnitude_error_h)

# Compute the topology error
topology_error_h = calculate_topology_error(sumo.heading.values, interpolated_carla_heading, path_h)

print('Topology error: ', topology_error_h)

print("X")
print('DTW distance: ', dtw_distance_h)

# Compute the magnitude error
magnitude_error_x = calculate_magnitude_error(sumo_x, interpolated_carla_x, path_x)

print('Magnitude error: ', magnitude_error_x)

# Compute the topology error
topology_error_x = calculate_topology_error(sumo_x, interpolated_carla_x, path_x)

print('Topology error: ', topology_error_x)

print("Y")
print('DTW distance: ', dtw_distance_y)

# Compute the magnitude error
magnitude_error_y = calculate_magnitude_error(sumo_y, interpolated_carla_y, path_y)

print('Magnitude error: ', magnitude_error_y)

# Compute the topology error
topology_error_y = calculate_topology_error(sumo_y, interpolated_carla_y, path_y)

print('Topology error: ', topology_error_y)
