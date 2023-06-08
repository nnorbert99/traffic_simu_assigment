from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import pandas as pd
import time

from scenario_descriptor import ScenarioDescription as sc
from scenario_descriptor import conv2sumoLoc

import carla
import pygame

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

actor_list = []
metrics_df = pd.DataFrame(
    columns=['actor_type', 'sumo_id', 'velocity', 'travelled_distance', 'throttle', 'steer', 'brake', 'acceleration',
             'acc_x', 'acc_y', 'loc_x', 'loc_y', 'heading', 'time_step'])

def log_metrics(path: str = None, last_tick = False):
    global metrics_df
    if path is None:
        path = os.getcwd()
    path = os.path.join(path, 'logs')
    if not os.path.exists(path):
        os.mkdir(path)
    filepath = os.path.join(path, f'logged_metrics_{time.time()}.csv')
    for ac in actor_list:
        sc_actor = [sc_ac for sc_ac in sc.all_actor if sc_ac.type == ac.type_id][0]
        actor_type = ac.type_id
        sumo_id = sc_actor.sumo_id
        vel = ac.get_velocity().length()
        time_step = world.get_snapshot().timestamp.delta_seconds
        travelled_dis = vel * time_step
        try:
            type_df = metrics_df.loc[metrics_df['actor_type'] == ac.type_id]
            travelled_dis = travelled_dis + type_df.iloc[-1,3]
        except IndexError:
            pass
        control = ac.get_control()
        throttle = control.throttle
        steer = control.steer
        brake = control.brake
        acc_x = ac.get_acceleration().x
        acc_y = ac.get_acceleration().y
        acceleration = (acc_x ** 2 + acc_y ** 2) ** (1 / 2)
        loc_x = ac.get_location().x
        loc_y = ac.get_location().y
        heading = ac.get_transform().rotation.yaw
        new_row = {'actor_type': actor_type, 'sumo_id': sumo_id, 'velocity': vel, 'travelled_distance': travelled_dis,
                   'throttle': throttle, 'steer': steer, 'brake': brake, 'acceleration': acceleration, 'acc_x': acc_x,
                   'acc_y': acc_y,'loc_x':loc_x,'loc_y':loc_y,'heading':heading,'time_step':time_step }
        new_row_df = pd.DataFrame([new_row])
        metrics_df = pd.concat([metrics_df, new_row_df], ignore_index=True)

        if last_tick:
            metrics_df.to_csv(filepath, index=False)


# Set up SUMO simulation
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


sumoBinary = checkBinary('sumo-gui')

traci.start([sumoBinary, '-c', 'C:\WindowsNoEditor\Co-Simulation\Sumo\examples\Town04.sumocfg',
                 '--step-length', '0.023'])



# Set up CARLA simulation
client = carla.Client("localhost", 2000)  # Connect to CARLA server
client.set_timeout(20.0)  # Set client timeout
client.load_world('Town04')

world = client.get_world()
map = world.get_map()
default_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.023
world.apply_settings(settings)
world.set_weather(carla.WeatherParameters.ClearNoon)
blueprint_library = world.get_blueprint_library()
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', str(CAMERA_WIDTH))
camera_bp.set_attribute('image_size_y', str(CAMERA_HEIGHT))


# Spawn vehicles in CARLA

ego_vehicle_id = sc.ego_actor.sumo_id
sumo_vehicle_ids = []

# Spawn SUMO-controlled vehicles in CARLA
carla_vehicles = []

for actor in sc.other_actors:
    vehicle_location = carla.Transform(carla.Location(*actor.pos), carla.Rotation(*actor.rot))
    blueprint = world.get_blueprint_library().find(actor.type)
    carla_vehicle = world.try_spawn_actor(blueprint, vehicle_location)
    carla_vehicles.append(carla_vehicle)
    vehicle_id = actor.sumo_id
    sumo_vehicle_ids.append(vehicle_id)


# Spawn the ego vehicle

ego_bp = blueprint_library.find(sc.ego_actor.type)
ego_vehicle_location = carla.Transform(carla.Location(*sc.ego_actor.pos), carla.Rotation(*sc.ego_actor.rot))
ego_vehicle = world.try_spawn_actor(ego_bp, ego_vehicle_location)
ego_vehicle.apply_control(carla.VehicleControl(*sc.ego_actor.control))
actor_list.append(ego_vehicle)

# Spectator
spectator = world.get_spectator()
spectator_location = carla.Transform(
    carla.Location(x=-445.9, y=24.23, z=44.5), carla.Rotation(pitch=-47.0, yaw=0.0, roll=0.0))
spectator.set_transform(spectator_location)

running = True

# Simulation loop
distance = 0.0
step = 0

vehic_num = 0
start_edge_ego, _, start_lane_ego = traci.simulation.convertRoad(*conv2sumoLoc(sc.ego_actor.pos))
des_edge_ego, _, des_lane_ego = traci.simulation.convertRoad(*conv2sumoLoc(sc.ego_actor.destination_pos))
traci.route.add(f'trip{vehic_num}', [f'{start_edge_ego}', f'{des_edge_ego}'])
traci.vehicle.add(sc.ego_actor.sumo_id, f'trip{vehic_num}', typeID=sc.ego_actor.type.split('_')[0])
traci.vehicle.moveToXY(sc.ego_actor.sumo_id, f'{start_edge_ego}', f'{start_lane_ego}',
                       *conv2sumoLoc(sc.ego_actor.pos), keepRoute=1)
vehic_num += 1
for actor in sc.other_actors:
    start_edge_actor, _, start_lane_actor = traci.simulation.convertRoad(*conv2sumoLoc(actor.pos))
    des_edge_actor, _, des_lane_actor = traci.simulation.convertRoad(*conv2sumoLoc(actor.destination_pos))
    traci.route.add(f'trip{vehic_num}', [f'{start_edge_actor}', f'{des_edge_actor}'])
    traci.vehicle.add(actor.sumo_id, f'trip{vehic_num}', typeID=actor.type)
    traci.vehicle.moveToXY(actor.sumo_id, f'{start_edge_actor}', f'{start_lane_actor}',
                           *conv2sumoLoc(actor.pos), keepRoute=1)
    vehic_num += 1

running = True

while step < 401:
    if step == 400:
        running = False


    traci.simulationStep()  # Advance SUMO simulation by one time step
    world.tick()
    
    x_offset = 503.02
    y_offset = 423.76

    # Update SUMO-controlled vehicles positions and velocities based on SUMO states
    for vehicle, vehicle_id in zip(carla_vehicles, sumo_vehicle_ids):
        sumo_vehicle_pos = traci.vehicle.getPosition(vehicle_id)
        sumo_vehicle_ori = traci.vehicle.getAngle(vehicle_id)
        sumo_vehicle_speed = traci.vehicle.getSpeed(vehicle_id)
        sumo_vehicle_acc = traci.vehicle.getAcceleration(vehicle_id)

        vehicle.set_transform(
            carla.Transform(
                carla.Location(sumo_vehicle_pos[0] - x_offset, y_offset - sumo_vehicle_pos[1], 0),
                carla.Rotation(0, sumo_vehicle_ori - 90, 0)
            )
        )
        vehicle.set_target_velocity(
            carla.Vector3D(sumo_vehicle_speed * 3.6, 0, 0)  # Convert SUMO speed to CARLA velocity (km/h)
        )

    # Get CARLA-controlled ego vehicle state and update SUMO state
    ego_vehicle_speed = ego_vehicle.get_velocity().x / 3.6  # Convert CARLA velocity to SUMO speed (m/s)
    ego_vehicle_angle = ego_vehicle.get_transform().rotation.yaw + 90

    sumo_x = ego_vehicle_location.location.x + x_offset
    sumo_y = y_offset - ego_vehicle_location.location.y
    edgeID, _, lane = traci.simulation.convertRoad(sumo_x, sumo_y)
    traci.vehicle.moveToXY(ego_vehicle_id, edgeID, lane, sumo_x, sumo_y, ego_vehicle_angle)
    traci.vehicle.setSpeed(ego_vehicle_id, ego_vehicle_speed)

    #------------------------------------------------------------------------------
    # EGO control segment : START
    current_location = ego_vehicle.get_transform().location
    distance += ego_vehicle_location.location.distance(current_location)
    ego_vehicle_location.location = current_location
    # set Traffic Lights
    actors = world.get_actors().filter('traffic.traffic_light')
    for actor in actors:
        state = carla.TrafficLightState.Red
        light_dis = actor.get_location().distance(current_location)
        if light_dis < 55:
            state = carla.TrafficLightState.Yellow
        if light_dis < 45:
            state = carla.TrafficLightState.Green
        actor.set_state(state)

    if distance > 20:
        ego_vehicle.apply_control(carla.VehicleControl(brake=0.8, steer=0.0))
    if distance > 34:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-0.3))
    if distance > 67:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=4.0, steer=0.0))
    if distance > 83:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.07))
    if distance > 87:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))

    # EGO control segment : END
    #------------------------------------------------------------------------------

    log_metrics(last_tick=not running)

    step += 1
# Cleanup
traci.close()
client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list])
