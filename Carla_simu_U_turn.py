import carla
import numpy as np
import pygame

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Pygame Setup
pygame.init()
screen = pygame.display.set_mode((CAMERA_WIDTH, CAMERA_HEIGHT))


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

    vehicle_location = carla.Transform(carla.Location(-426.5, 30.4, 0.5))
    vehicle = world.try_spawn_actor(vehicle_bp, vehicle_location)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    camera_relative_loc = carla.Transform(carla.Location(x=-5.7, z=3.7), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_relative_loc, attach_to=vehicle)
    camera.listen(lambda data: visualize_image(data))

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
        current_location = vehicle.get_transform().location
        distance += vehicle_location.location.distance(current_location)
        vehicle_location.location = current_location

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
            vehicle.apply_control(carla.VehicleControl(brake=0.2, steer=0.0))
        if distance > 34:
            vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-0.3))
        if distance > 67:
            vehicle.apply_control(carla.VehicleControl(throttle=4.0, steer=0.0))
        if distance > 83:
            vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.07))
        if distance > 87:
            vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        pygame.display.flip()

finally:
    world.apply_settings(default_settings)
    world.tick()
    vehicle.destroy()
    camera.destroy()
    pygame.quit()
