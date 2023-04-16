import carla
import numpy as np
import pygame

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Pygame Setup
pygame.init()
screen = pygame.display.set_mode((CAMERA_WIDTH, CAMERA_HEIGHT))

actor_list = []


def visualize_image(image):
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    screen.blit(surface, (0, 0))


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

    # VUT vehicle with camera attached to it's back
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.mercedes.coupe_2020')
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(CAMERA_WIDTH))
    camera_bp.set_attribute('image_size_y', str(CAMERA_HEIGHT))

    ego_vehicle_location = carla.Transform(carla.Location(-426.5, 30.4, 0.5))
    ego_vehicle = world.try_spawn_actor(vehicle_bp, ego_vehicle_location)
    ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    camera_relative_loc = carla.Transform(carla.Location(x=-5.7, z=3.7), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_relative_loc, attach_to=ego_vehicle)
    camera.listen(lambda data: visualize_image(data))
    actor_list.append(ego_vehicle)

    vehicle_bp = blueprint_library.find('vehicle.audi.a2')
    vehicle_location = carla.Transform(carla.Location(-426.4, 26.9, 0.5))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0))
    actor_list.append(vehicle)

    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    vehicle_location = carla.Transform(carla.Location(-447.2, 37.4, 0.5))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    actor_list.append(vehicle)

    vehicle_bp = blueprint_library.find('vehicle.nissan.micra')
    vehicle_location = carla.Transform(carla.Location(-391.5, 33.8, 0.5))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    actor_list.append(vehicle)

    vehicle_bp = blueprint_library.find('vehicle.audi.etron')
    vehicle_location = carla.Transform(carla.Location(-298.2, 5.4, 3),carla.Rotation(yaw=180))
    vehicle = world.spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    actor_list.append(vehicle)

    vehicle_bp = blueprint_library.find('vehicle.tesla.cybertruck')
    vehicle_location = carla.Transform(carla.Location(-315.9, 12.7, 3),carla.Rotation(yaw=180))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    actor_list.append(vehicle)

    vehicle_bp = blueprint_library.find('vehicle.dodge.charger_police')
    vehicle_location = carla.Transform(carla.Location(-347.7, 16.1, 3), carla.Rotation(yaw=180))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    actor_list.append(vehicle)

    vehicle_bp = blueprint_library.find('vehicle.ford.mustang')
    vehicle_location = carla.Transform(carla.Location(-384.5, -8.5, 3), carla.Rotation(yaw=90))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))
    actor_list.append(vehicle)

    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    vehicle_location = carla.Transform(carla.Location(-380.7, -7.4, 3), carla.Rotation(yaw=90))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))
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

        # print(distance)
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

finally:
    world.apply_settings(default_settings)
    world.tick()
    vehicle.destroy()
    camera.destroy()
    pygame.quit()
