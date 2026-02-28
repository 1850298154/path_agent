import json
import os
import numpy as np
import geometry
import plot
import pickle
import SET
import zstatistics as zs
SET.initialize_set()
# 加载并读取存储的UAV对象列表
# 002_nt\2023-09-05_11-32-58\print.txt
# F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-05_14-46-21
# 2023-09-05_22-18-09
path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-08-31_16-35-45\agent100\agent_list_100.pkl'
path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-05_11-32-58\agent100\agent_list_100.pkl'
path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-05_22-18-09\agent100\agent_list_100.pkl'
path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-09_22-46-35\agent100\agent_list_100.pkl'
path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-09_22-46-37\agent100\agent_list_100.pkl'
path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-20_22-31-06\agent100\agent_list_100.pkl'
path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC\OB_2D\002_nt\2023-09-27_16-36-23---进OB了\agent100\agent_list_100.pkl'
path = r'/home/bnw/MPC/OB_2D/004/2023-10-24_10-22-40/agent100/agent_list_100.pkl'
# path = r''
with open(path, "rb") as f:
    agent_list = pickle.load(f)

new_tractive_list = [
    
    for i in len(agent_list[42].data)
]
agent_list[42].data

# path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-08-31_16-35-45\agent100\agent_list_100.pkl'
# with open(path, "rb") as f:
#     agent_list = pickle.load(f)
#  plot_all_pre_traj(agent_list, obstacle_list, show, episodes)

# 获取当前文件所在的目录路径
# current_dir = os.path.dirname(os.path.abspath(__file__))
current_dir = os.path.dirname(os.path.abspath(path))

# 拼接目标文件的路径
# target_file_path = os.path.join(current_dir, "obstacles.json") # 不是真JSON
target_file_path = os.path.join(current_dir, "obstacles.pkl")

# 打印目标文件的路径
print(target_file_path)
# 打开目标文件并读取内容
with open(target_file_path, 'rb') as file:
    obstacle_list = pickle.load(file)

# import zrand as zr
# zr.obstacles = obstacle_list
# zr.extend_obstacles = zr.make_extend_obstacles(obstacles=obstacle_list,radius=zr.radius)
# zs.fstatistics(agent_list=agent_list) 

# obstacle_list = [(19.90483245259833, 9.229841051752082, 45)]
# obstacle_list = [(44.463889765000935, 32.48971580562379, 54.772255750516614)]
# obstacle_list = [(21.12259173571769, 65.15159520730612, 10.0), (53.52742539855494, 13.68940234358442, 10.0), (52.4277664889509, 63.07710991990868, 10.0), (79.91558613833139, 22.556996592618816, 10.0), (18.592853279185046, 6.876560067110321, 10.0), (85.26212992449648, 62.67576679048956, 10.0), (33.2741318041034, 40.26060502212709, 10.0), (42.08998663990984, 86.16697613946859, 10.0), (64.45661583061617, 44.207786769369406, 10.0), (11.44162005480852, 27.900621850883223, 10.0),
#                  (35.99403057148203, 20.660475534795147, 10.0), (80.92475673826564, 3.3878470669396474, 10.0), (12.15955268294461, 46.55051705513637, 10.0), (5.617269395501804, 84.95293777568217, 10.0), (60.057880590908155, 88.44080277010175, 10.0), (82.7933201324779, 82.9742753902054, 10.0), (23.76854162023156, 88.69267440537402, 10.0), (0.49316893855936406, 10.459152574540045, 10.0), (82.8833837383154, 44.12664970388627, 10.0), (35.82404217445927, 2.9817098558649233, 10.0)]
# obstacle_list=[(89.56644424595373, 69.21995573677518, 10.0), (2.590521899087488, 10.163675184522035, 10.0), (27.50605581887015, 67.14281577713933, 10.0), (51.478236518303355, 18.98171349872495, 10.0), (60.354916044256846, 79.95933759103265, 10.0), (43.16116149872769, 42.25147588223885, 10.0), (87.5257734838176, 31.778078828629724, 10.0), (50.417065906675084, 0.8642864178953458, 10.0), (12.615614532961358, 43.45491811644925, 10.0), (82.03523797415782, 9.811518109063996, 10.0), (67.26061228994337, 61.575398850807694, 10.0), (39.80208931872106, 88.88255332157624, 10.0), (16.540306744814764, 85.21257901835892, 10.0), (25.618671542360143, 0.5785090051634578, 10.0), (85.66279664168052, 88.52508497320206, 10.0), (22.59990515189366, 25.92307035199542, 10.0), (1.5113752119183121, 65.15018280612992, 10.0), (64.72485762275298, 40.98758696597912, 10.0), (48.53781994519814, 62.672591868241874, 10.0), (86.3746850891212, 50.68845885968511, 10.0)]
# obstacle_list=[(6.608854845881571, 24.037648025501053, 54.772255750516614)]

# exit()

# ini_obstacle_list = [
ini_obstacle_list = [
    geometry.rectangle(np.array([ob[0], ob[1]]),
                       ob[2],
                       ob[2],
                       0.0)
    for ob in obstacle_list
]
obstacle_list = [
    geometry.rectangle(np.array([ob[0], obe[1]]),
                        ob[2],
                        ob[2],
                        SET.ExtendWidth)
    for ob in zr.obstacles
]
plot.plot_all_pre_traj(agent_list, ini_obstacle_list, obstacle_list, True, -1)
