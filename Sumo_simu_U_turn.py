from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import pandas as pd
import plotly.express as px

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

def extract_speed_values(df:pd.DataFrame,actors:list)->dict:
    speed_dict = {}
    for actor in actors:
        sumo_id = actor.sumo_id
        sumo_type_df = df.loc[df['sumo_id']==sumo_id]
        velocities = sumo_type_df['velocity'].tolist()
        speed_dict.update({actor.sumo_id:velocities})
    return speed_dict

def save_plots(df:pd.DataFrame)->None:

    act_types = df['actor_type'].tolist()[0:8]
    # for col in df.columns:
    #     for act_type in act_types:
    #         df_type = df.loc[df['actor_type']==act_type]
    #         fig = px.line(df_type,x='time',y=col,title = act_type)
    #         fig.write_html(f'./figs/{col}__{act_type}.html')
    for act_type in act_types:
        df_type = df.loc[df['actor_type']==act_type]
        fig = px.line(df_type,x='loc_x',y='loc_y',title = f'Trajectory {act_type}')
        fig.write_html(f'./figs/Trajectory__{act_type}.html')
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
    path = os.path.join(os.getcwd(), 'logs')
    df = None
    for file in os.listdir(path):
        filepath = os.path.join(path, file)
        if os.path.isfile(filepath) and file.endswith('.csv'):
            df = pd.read_csv(filepath)
            break
    if df is None:
        raise Exception(f'No log (cvs) file found at {path}')
    speeds = extract_speed_values(df,sc.all_actor)
    #save_plots(df)
    sumo_df = pd.DataFrame(columns=['speed','x','y','heading','time'])
    while step < 1000:
        try:
            traci.simulationStep()
            s_speed = traci.vehicle.getSpeed('vehicle0')
            s_trajectory = traci.vehicle.getPosition('vehicle0')
            s_heading = traci.vehicle.getAngle('vehicle0')
            try:
                s_time = sumo_df.iloc[-1,-1] + 0.115
            except:
                s_time = 0
            sumo_dict = {'speed':s_speed, 'x':s_trajectory[0],'y':s_trajectory[1], 'heading':s_heading,'time':s_time}
            sumo_df = sumo_df.append(sumo_dict, ignore_index=True)
            for sumo_id,vel in speeds.items():
                traci.vehicle.setSpeed(sumo_id,vel[step])
            step += 1
        except:
            figure = px.line(sumo_df, x='x', y='y',title='SUMO Trajectory ego')
            figure.write_html('./sumo_trajectory.html')
            figure = px.line(sumo_df, x='time', y='speed', title='SUMO velocity ego')
            figure.write_html('./sumo_vel.html')
            figure = px.line(sumo_df, x='time', y='heading', title='SUMO heading ego')
            figure.write_html('./sumo_heading.html')
            sumo_df.to_csv('./logs/sumo_logged.csv')
            traci.close()


if __name__ == '__main__':
    sumoBinary = checkBinary('sumo-gui')

    traci.start([sumoBinary, '-c', 'D:\Carla_bullshit\WindowsNoEditor\Co-Simulation\Sumo\examples\Town04.sumocfg',
                 '--step-length', '0.023'])
    run()
