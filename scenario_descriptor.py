class Actor:
    def __init__(self, pos: tuple,destination_pos:tuple, rot: tuple, type: str, control: tuple, sensor: bool = False,
                 sens_rel_loc: tuple = None):
        self.pos = pos
        self.destination_pos = destination_pos
        self.rot = rot
        self.type = type
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
        'vehicle.mercedes.coupe_2020',
        (1.0, 0.0, 0.0),
        sensor=True,
        sens_rel_loc=(-5.7, 0, 3.7))
    other_actors = [
        Actor(
            (-426.4, 26.9, 0.5),
            (0, 0, 0),
            'vehicle.audi.a2',
            (0.6, 0.0, 0.0)),
        Actor(
            (-447.2, 37.4, 0.5),
            (0, 0, 0),
            'vehicle.tesla.model3',
            (1.0, 0.0, 0.0)),
        Actor(
            (-298.2, 5.4, 3),
            (0, 180, 0),
            'vehicle.audi.etron',
            (1.0, 0.0, 0.0)),
        Actor(
            (-315.9, 12.7, 3),
            (0, 180, 0),
            'vehicle.tesla.cybertruck',
            (1.0, 0.0, 0.0)),
        Actor(
            (-347.7, 16.1, 3),
            (0, 180, 0),
            'vehicle.dodge.charger_police',
            (1.0, 0.0, 0.0)),
        Actor(
            (-384.5, -8.5, 3),
            (0, 90, 0),
            'vehicle.ford.mustang',
            (0.0, 0.0, 0.0)),
        Actor(
            (-380.7, -7.4, 3),
            (0, 90, 0),
            'vehicle.tesla.model3',
            (0.0, 0.0, 0.0)),
    ]


def conv2sumoLoc(loc: tuple) -> tuple:
    sumo_loc = (loc[0] + 503.02, loc[1] + 423.76)
    return sumo_loc
