from __future__ import absolute_import
from __future__ import print_function

import os
import sys

from scenario_descriptor import ScenarioDescription as sc
from scenario_descriptor import conv2sumoLoc

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def run():
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
    while step < 1000:
        traci.simulationStep()
        step += 1
    traci.close()


if __name__ == '__main__':
    sumoBinary = checkBinary('sumo-gui')

    traci.start([sumoBinary, '-c', 'D:\Carla_bullshit\WindowsNoEditor\Co-Simulation\Sumo\examples\Town04.sumocfg',
                 '--step-length', '0.023'])
    run()
