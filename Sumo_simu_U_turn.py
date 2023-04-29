from __future__ import absolute_import
from __future__ import print_function

import os
import sys

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
    traci.route.add('trip01',['-41.0.00','45.0.00'])
    traci.vehicle.add('vehicle01', 'trip01',typeID='vehicle.mercedes.coupe')
    traci.vehicle.moveToXY('vehicle01', '-41', '-41_3',448.26,52.29, keepRoute=1)
    while step < 1000:

        traci.simulationStep()
        step += 1
    traci.close()


if __name__ == '__main__':
    sumoBinary = checkBinary('sumo-gui')

    traci.start([sumoBinary, '-c', 'D:\Carla_bullshit\WindowsNoEditor\Co-Simulation\Sumo\examples\Town04.sumocfg'])
    run()
