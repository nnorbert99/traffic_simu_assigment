import os
import time

import carla
import numpy as np
import pandas as pd
import pygame

from scenario_descriptor import ScenarioDescription as sc

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Pygame Setup
pygame.init()
screen = pygame.display.set_mode((CAMERA_WIDTH, CAMERA_HEIGHT))

actor_list = []
metrics_df = pd.DataFrame(
    columns=['actor_type', 'sumo_id', 'velocity', 'travelled_distance', 'throttle', 'steer', 'brake', 'acceleration',
             'acc_x', 'acc_y', 'loc_x', 'loc_y', 'heading', 'time_step'])


def visualize_image(image):
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    screen.blit(surface, (0, 0))


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
        metrics_df = metrics_df.append(new_row,ignore_index=True)

        if last_tick:
            metrics_df.to_csv(filepath,index=False)



try:
    # Client
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    client.load_world('Town04')

    # World
    world = client.get_world()
    default_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.023
    world.apply_settings(settings)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    blueprint_library = world.get_blueprint_library()

    # Ego vehicle with camera attached to it's back

    vehicle_bp = blueprint_library.find(sc.ego_actor.type)
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(CAMERA_WIDTH))
    camera_bp.set_attribute('image_size_y', str(CAMERA_HEIGHT))
    ego_vehicle_location = carla.Transform(carla.Location(*sc.ego_actor.pos), carla.Rotation(*sc.ego_actor.rot))
    ego_vehicle = world.try_spawn_actor(vehicle_bp, ego_vehicle_location)
    ego_vehicle.apply_control(carla.VehicleControl(*sc.ego_actor.control))
    camera_relative_loc = carla.Transform(carla.Location(*sc.ego_actor.sens_rel_loc), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_relative_loc, attach_to=ego_vehicle)
    camera.listen(lambda data: visualize_image(data))
    actor_list.append(ego_vehicle)

    for actor in sc.other_actors:
        vehicle_bp = blueprint_library.find(actor.type)
        vehicle_location = carla.Transform(carla.Location(*actor.pos), carla.Rotation(*actor.rot))
        vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
        vehicle.apply_control(carla.VehicleControl(*actor.control))
        actor_list.append(vehicle)

    world.tick()

    # Spectator
    spectator = world.get_spectator()
    spectator_location = carla.Transform(
        carla.Location(x=-445.9, y=24.23, z=44.5), carla.Rotation(pitch=-47.0, yaw=0.0, roll=0.0))
    spectator.set_transform(spectator_location)

    running = True
    screen.fill((0, 0, 0))
    pygame.display.flip()

    distance = 0.0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        world.tick()
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
            ego_vehicle.apply_control(carla.VehicleControl(brake=0.2, steer=0.0))
            [a for a in actor_list if a.type_id == 'vehicle.audi.a2'][0].apply_control(
                carla.VehicleControl(throttle=1.0, steer=0.0))
        if distance > 34:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-0.3))
        if distance > 67:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=4.0, steer=0.0))
        if distance > 83:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.07))
        if distance > 87:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        pygame.display.flip()
        log_metrics(last_tick=not running)

finally:
    world.apply_settings(default_settings)
    world.tick()
    for actor in actor_list:
        actor.destroy()
    camera.destroy()
    pygame.quit()
