class Actor:
    sumo_index = 0

    def __init__(self, pos: tuple, rot: tuple, destionation_pos: tuple, type: str, control: tuple, sensor: bool = False,
                 sens_rel_loc: tuple = None):
        self.pos = pos
        self.destination_pos = destionation_pos
        self.rot = rot
        self.type = type
        self.sumo_id = f'vehicle{Actor.sumo_index}'
        Actor.sumo_index += 1
        self.control = control
        self.sensor = sensor
        self.sens_rel_loc = sens_rel_loc

    def __str__(self):
        print(
            f'Actor type: {self.type}\n '
            f'Actor position: x:{self.pos[0]},y:{self.pos[1]},z:{self.pos[2]}\n '
            f'Actor rotation: pitch:{self.rot[0]},yaw:{self.rot[1]},roll:{self.rot[2]}\n '
            f'Actor control: throttle:{self.control[0]},steer:{self.control[1]},brake:{self.control[2]}')


class ScenarioDescription:
    ego_actor = Actor(
        (-426.5, 30.4, 0.5),
        (0, 0, 0),
        (-438.57, 10.17, -0.00),
        'vehicle.mercedes.coupe_2020',
        (1.0, 0.0, 0.0),
        sensor=True,
        sens_rel_loc=(-5.7, 0, 3.7))
    other_actors = [
        Actor(
            (-426.4, 26.9, 0.5),
            (0, 0, 0),
            (-326.72, 26.91, 0.22),
            'vehicle.audi.a2',
            (0.6, 0.0, 0.0)),
        Actor(
            (-447.2, 37.4, 0.5),
            (0, 0, 0),
            (-220.10, 37.37, 3.68),
            'vehicle.tesla.model3',
            (1.0, 0.0, 0.0)),
        Actor(
            (-298.2, 5.4, 3),
            (0, 180, 0),
            (-446.78, 6.99, -0.02),
            'vehicle.audi.etron',
            (1.0, 0.0, 0.0)),
        Actor(
            (-315.9, 12.7, 3),
            (0, 180, 0),
            (-464.65, 14.54, -0.01),
            'vehicle.tesla.cybertruck',
            (1.0, 0.0, 0.0)),
        Actor(
            (-337.7, 16.1, 3),
            (0, 180, 0),
            (-470.86, 17.06, -0.03),
            'vehicle.dodge.charger_police',
            (1.0, 0.0, 0.0)),
    ]
    all_actor = [ego_actor]
    all_actor.extend(other_actors)


def conv2sumoLoc(loc: tuple) -> tuple:
    sumo_loc = (loc[0] + 503.02, -loc[1] + 423.76)
    return sumo_loc
