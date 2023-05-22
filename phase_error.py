import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from scipy.interpolate import interp1d
from scipy.signal import correlate


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
# Correct the offset in the Carla data
# corrected_carla_heading = np.where(carla_heading.heading.values < 0, carla_heading.heading.values + 360,
#                                    carla_heading.heading.values)

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

# Plotting speed
plt.plot(sumo_time, sumo.speed, label='SUMO Speed')
plt.plot(carla_time_adjusted, interpolated_carla_speed, label='Carla Speed')
plt.xlabel('Time')
plt.ylabel('Speed')
plt.legend()
plt.title('Comparison of SUMO and Carla Speed')
plt.show()

sumo_trace = go.Scatter(x=sumo_time, y=sumo.speed, name='SUMO', mode='lines', line={'dash':'dash'})
carla_trace = go.Scatter(x=carla_time_adjusted, y=interpolated_carla_speed, name='CARLA', mode='lines')
fig = go.Figure(data=[sumo_trace, carla_trace],
                layout=go.Layout(title='Comparison between Sumo and Carla velocity',
                                 xaxis={'title': 'Time'},
                                 yaxis={'title': 'Velocity'}))
fig.show()

# Plotting heading
plt.plot(sumo_time, sumo.heading, label='SUMO heading')
plt.plot(carla_time_adjusted, interpolated_carla_heading, label='Carla heading')
plt.xlabel('Time')
plt.ylabel('Heading angle')
plt.legend()
plt.title('Comparison of SUMO and Carla Heading')
plt.show()

sumo_trace = go.Scatter(x=sumo_time, y=sumo.heading, name='SUMO', mode='lines', line={'dash':'dash'})
carla_trace = go.Scatter(x=carla_time_adjusted, y=interpolated_carla_heading, name='CARLA', mode='lines')
fig = go.Figure(data=[sumo_trace, carla_trace],
                layout=go.Layout(title='Comparison between Sumo and Carla heading angle',
                                 xaxis={'title': 'Time'},
                                 yaxis={'title': 'heading angle'}))
fig.show()

# Plotting trajectory
# plot car1's trajectory, 'b-' means blue line
plt.plot(sumo_x, sumo_y, 'b-', label='SUMO')
# plot car2's trajectory, 'r--' means red dashed line
plt.plot(carla_x, carla_y, 'r--', label='CARLA')
plt.legend()  # display labels in a legend
plt.xlabel('X Coordinate')  # x-axis label
plt.ylabel('Y Coordinate')  # y-axis label
plt.title('Car Trajectories')  # plot title
plt.show()  # display the plot

sumo_trace = go.Scatter(x=sumo_x, y=sumo_y, name='SUMO', mode='lines', line={'dash':'dash'})
carla_trace = go.Scatter(x=carla_x, y=carla_y, name='CARLA', mode='lines')
fig = go.Figure(data=[sumo_trace, carla_trace],
                layout=go.Layout(title='Comparison between Sumo and Carla trajectory',
                                 xaxis={'title': 'X'},
                                 yaxis={'title': 'Y'}))
fig.show()

# Initialize the ranges for c and r
c_range = np.linspace(1, 60, 100)
r_range = np.linspace(1, 60, 100)

# Initialize a dictionary to store the phase errors
phase_errors = {}
# Compute the phase error for each combination of c and r
for c in c_range:
    for r in r_range:
        phase_error, n_shift = calculate_phase_error(sumo.speed.values, interpolated_carla_speed, c, r)
        phase_errors[(c, r)] = phase_error
phase_errors_h = {}
# for c in c_range:
#     for r in r_range:
#         phase_error_h, n_shift_h = calculate_phase_error(sumo.heading.values, interpolated_carla_heading, c, r)
#         phase_errors_h[(c, r)] = phase_error_h
# phase_errors_x = {}
# for c in c_range:
#     for r in r_range:
#         phase_error_x, n_shift_h = calculate_phase_error(sumo_x, interpolated_carla_x, c, r)
#         phase_errors_x[(c, r)] = phase_error_x
# phase_errors_y = {}
# for c in c_range:
#     for r in r_range:
#         phase_error_y, n_shift_y = calculate_phase_error(sumo_y, interpolated_carla_y, c, r)
#         phase_errors_y[(c, r)] = phase_error_y
#
# # Find the (c, r) pair with the smallest phase error
# best_c, best_r = min(phase_errors, key=phase_errors.get)
# best_c_h, best_r_h = min(phase_errors_h, key=phase_errors_h.get)
# best_c_x, best_r_x = min(phase_errors_x, key=phase_errors_x.get)
# best_c_y, best_r_y = min(phase_errors_y, key=phase_errors_y.get)
best_c = 20
best_r = 10

best_c_h = 20
best_r_h = 10

best_c_x = 20
best_r_x = 10

best_c_y = 20
best_r_y = 10

phase_error, n_shift = calculate_phase_error(sumo.speed, interpolated_carla_speed, best_c, best_r)
phase_error_h, n_shift_h = calculate_phase_error(sumo.heading, interpolated_carla_heading, best_c_h, best_r_h)
phase_error_x, n_shift_x = calculate_phase_error(sumo_x, interpolated_carla_x, best_c_x, best_r_x)
phase_error_y, n_shift_y = calculate_phase_error(sumo_y, interpolated_carla_y, best_c_y, best_r_y)

print("SPEED")
print("C: " + str(best_c))
print("R: " + str(best_r))
print("n* : " + str(n_shift))
print("Phase error: " + str(phase_error))
print()
print("HEADING")
print("C: " + str(best_c_h))
print("R: " + str(best_r_h))
print("n* : " + str(n_shift_h))
print("Phase error: " + str(phase_error_h))
print("X")
print("C: " + str(best_c_x))
print("R: " + str(best_r_x))
print("n* : " + str(n_shift_x))
print("Phase error: " + str(phase_error_x))
print("Y")
print("C: " + str(best_c_y))
print("R: " + str(best_r_y))
print("n* : " + str(n_shift_y))
print("Phase error: " + str(phase_error_y))
