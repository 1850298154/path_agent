# zrand.py
print('OB_2D  mode :: zrand 模块初始化...') # 在导入所有【自定义】和【标准】库之后，再执行这句打印
import sys
import os
# sys.path.append(  os.path.dirname(os.path.abspath(__file__))   +'\..')  # 将当前路径目录的再上一级目录添加到Python搜索路径中
sys.path.append(  os.path.dirname(os.path.abspath(__file__))   +'/..')  # 将当前路径目录的再上一级目录添加到Python搜索路径中

import shared_util.io_filename as iof
import shared_util.sys_argument as ag
import random
import matplotlib.pyplot as plt
import math
import output_filename as of
import numpy as np
import zyaml as zy
import yaml
import sys

m = zy.parameters['avoid.m']
# m = 10

# 智能体参数
radius = zy.parameters['radius']  # 通过
# radius = 0.01  # tag1 通过
# radius = 0.3  # 通过
# radius = 1  # 失败
# radius = 0.6  # 失败
# radius = 0.45  # 失败
# radius = 0.37  # 失败
# radius = 3
Num = zy.parameters['Num']      # the number of agents
# Num = 100     # the number of agents
# Num = 15     # the number of agents

# 地图范围
set_xlim = zy.parameters['zr.set_xlim']
set_ylim = zy.parameters['zr.set_ylim']
# x_range = 100
# y_range = 100
# x_range = 75
# y_range = 75
# x_range = 50
# y_range = 50
# x_range = 25
# y_range = 25
# x_range = 10
# y_range = 10
map_area = set_xlim * set_ylim  # 平面面积

ob_rate = zy.parameters['zr.ob_rate']
# ob_rate = 0.2
# rate = 0.01
# obstacles_area_std = map_area * (ob_rate-0.001)  # 障碍物面积要求, -0.001是为了消除误差。 小于0的时候，不会生成障碍物
obstacles_area_std = map_area * (ob_rate-1e-6)  # 障碍物面积要求, -0.001是为了消除误差。 小于0的时候，不会生成障碍物
# 环境参数
lower_limit_Square_side_length = zy.parameters['zr.lower_limit_Square_side_length']
upper_limit_Square_side_length = zy.parameters['zr.upper_limit_Square_side_length']
# lower_limit_Square_side_length = 45
# upper_limit_Square_side_length = 45
# lower_limit_Square_side_length = 1
# upper_limit_Square_side_length = 5

buffer = zy.parameters['buffer']
# buffer = 0.03

bug_step_size = zy.parameters['bug.bug_step_size']
# bug_step_size = 0.5

inflated_size = zy.parameters['bug.inflated_size']
# inflated_size = radius*5
# inflated_size = 0.2

more_inflated_size = zy.parameters['zr.more_inflated_size']
# more_inflated_size = (zy.parameters['zr.more_inflated_size']
#                       + zy.parameters['bug.num_tracks']*2*zy.parameters['radius'])
# more_inflated_size = 0.3
# more_inflated_size = 0.1
# more_inflated_size = 0


# 程序迭代次数
episodes = zy.parameters['episodes']
# episodes = int(set_xlim*4 / 0.15)
# episodes = 70*2+30
# episodes = 2
# print('episodes')
# print(episodes)
# h=0.15

# 障碍物、智能体起点、智能体终点
obstacles = []  # 存储障碍物的列表
# extend_obstacles = []  # 存储障碍物的列表
agents_starts = []  # 存储agent的列表
agents_ends = []  # 存储agents_ends的列表


############################################################
############     app.py 初始化障碍物(膨胀后)不能重合    ######
############################################################
# 判断两个障碍物是否重叠的函数
first = True


def is_overlap(obstacle1, obstacle2):
    x1, y1, size1 = obstacle1
    x2, y2, size2 = obstacle2
    # more_margin = buffer*2 + radius + inflated_size + more_inflated_size
    # global first
    # if first:
    #     print('buffer*2 + radius + inflated_size + more_inflated_size')
    #     print(buffer*2, radius, inflated_size, more_inflated_size)
    #     first = False
    # more_margin = buffer*2 + radius + inflated_size + more_inflated_size
    more_margin = more_inflated_size
    global first
    if first:
        print('more_margin')
        print(more_margin)
        first = False
    # 更多的膨胀是加入多个轨道， 2倍是因为，要让两个相邻障碍物 不要碰撞
    # more_margin *= 2
    x1 -= more_margin
    x2 -= more_margin
    y1 -= more_margin
    y2 -= more_margin
    size1 += more_margin*2
    size2 += more_margin*2

    # 左下角+边长 < 左下角
    if (x1 + size1+more_margin) < x2 or (x2 + size2+more_margin) < x1:
        return False

    if (y1 + size1+more_margin) < y2 or (y2 + size2+more_margin) < y1:
        return False
    return True

    # # 按照象限 右上、左上
    # xlist = [x1+size1,x1,x1,x1+size1 ]
    # ylist = [y1+size1,y1+size1,y1,y1]
    # for x,y in zip(xlist,ylist):
    #     # 落在正方形中
    #     if ((x >= x2 and x <= x2 + size2) and
    #         (y >= y2 and y <= y2 + size2)):
    #         return True
    # return False


def generate_obstacles():
    cumulative_ob_area = 0  # 记录已生成的障碍物面积之和
    cnt_ob_fail = 0
    threshold = 1000
    threshold = 100000*3
    while cumulative_ob_area < obstacles_area_std:
        cnt_ob_fail += 1
        if cnt_ob_fail > threshold:
            print('generate_obstacles')
            print('cnt_ob_fail > threshold')
            return False
        # size = random.randint(lower_limit_Square_side_length,
        size = random.uniform(lower_limit_Square_side_length,
                              upper_limit_Square_side_length)  # 障碍物边长在1到5之间随机选择
        obstacle_area = size ** 2  # 障碍物面积

        # # 你让while循环的判断干什么，别让while判断闲着
        # if total_area + obstacle_area > obstacles_area:
        #     break

        x = random.uniform(0, set_xlim - size)  # 随机选择障碍物的左下角x坐标
        y = random.uniform(0, set_ylim - size)  # 随机选择障碍物的左下角y坐标

        obstacle = (x, y, size)  # 障碍物的数据结构为元组(x坐标, y坐标, 边长)
        # obstacles.append(obstacle)
        # total_area += obstacle_area

        # 判断是否与已生成的障碍物重叠
        overlap = False
        for existing_obstacle in obstacles:
            if is_overlap(obstacle, existing_obstacle):
                overlap = True
                break

        if not overlap:  # 不重叠才添加到障碍物列表中
            obstacles.append(obstacle)
            cumulative_ob_area += obstacle_area
    # end while
    print('area                        : ', map_area)
    print('obstacles_area_std          : ', obstacles_area_std)
    print('cumulative_ob_area          : ', cumulative_ob_area)
    print('cumulative_ob_area/map_area : ', cumulative_ob_area/map_area)
    print('ob_rate                     : ', ob_rate)
    return True


############################################################
############     app.py 初始化agent起点终点不重合    #########
############################################################
def check_agents_external_collision(x, y, obstacles, more_inflated_size):
    # more_margin = buffer*2 + radius + inflated_size + more_inflated_size
    # more_margin = radius + inflated_size + more_inflated_size
    # more_margin = inflated_size + more_inflated_size
    more_margin = more_inflated_size
    # 更多的膨胀是加入多个轨道， 2倍是因为，要让两个相邻障碍物 不要碰撞
    # more_margin *= 2

    for i, obstacle in enumerate(obstacles):
        if ((
                # obstacle[0]-buffer*2-radius-inflated_size <= x <= obstacle[0]+obstacle[2]+buffer*2 + radius+inflated_size)
                obstacle[0]-more_margin <= x <= obstacle[0]+obstacle[2]+more_margin)
            and (
                # obstacle[1]-buffer*2-radius-inflated_size <= y <= obstacle[1]+obstacle[2]+buffer*2 + radius+inflated_size)):
                obstacle[1]-more_margin <= y <= obstacle[1]+obstacle[2]+more_margin)):
            return True, i
    return False, -1


# def tri_check_agents_external_collision(x, y, z, obstacles, more_inflated_size):
def tri_check_agents_external_collision(x, y, obstacles, more_inflated_size):
    # more_margin = buffer*2 + radius + inflated_size + more_inflated_size
    # more_margin = radius + inflated_size + more_inflated_size
    # more_margin = inflated_size + more_inflated_size
    more_margin = more_inflated_size
    # 更多的膨胀是加入多个轨道， 2倍是因为，要让两个相邻障碍物 不要碰撞
    # more_margin *= 2

    for i, obstacle in enumerate(obstacles):
        # six=[
        four=[
            - (obstacle[0]-more_margin - x) ,
            obstacle[0]+obstacle[-1]+more_margin - x,
            - (obstacle[1]-more_margin - y) , 
            obstacle[1]+obstacle[-1]+more_margin - y,
            # - (obstacle[2]-more_margin - z) ,
            # obstacle[2]+obstacle[3]+more_margin - z,
        ]
        # 用里面的xyz 表示正向 距离 >0
        # 如果 在 cube 里面 ，则 six都 >= 0 （就取最小的数，看是不是非负数） ; 最小值也>0 ; 且最小值表示进入cube的方向和深度 ; 最大值 是最小值另一个对应面的距离
        smin=min(four)
        # 如果 xyz 在外， 则存在 six < 0 （就取最小的数，看是不是负数）; 最小值<0 ; 且最小值表示到cube某个面的距离的负数
        smax=max(four)
        # if (
        #     np.all(np.array(six)>0)
        #     ):
        if (
            smin > 0
            ):
            return True, i, smin # 知道距离哪个面最近
    return False, -1, -1 # -1 表示没有和障碍物碰撞  -1表示不好计算agent到障碍物的最短距离



def check_agents_internal_collision(x, y, agents):
    # more_margin = 4*(radius+buffer)  # 4个半径，两个机器人。中间可以通过机器人
    # more_margin = 2*(radius+buffer)  # 4个半径，两个机器人。中间可以通过机器人
    more_margin = 6*(radius+buffer)  # 4个半径，两个机器人。中间可以通过机器人

    for agent in agents:
        # # if (agent[0][0] <= x <= agent[0][0] + 1) and (agent[0][1] <= y <= agent[0][1] + 1):
        # #     return True
        # # if (agent[1][0] <= x <= agent[1][0] + 1) and (agent[1][1] <= y <= agent[1][1] + 1):
        # #     return True

        # # 换个写法
        # # if ((
        # #         agent[0]-2*radius <= x <= agent[0] + 2*radius)
        # #     and (
        # #         agent[1]-2*radius <= y <= agent[1] + 2*radius)):
        # #     return True
        # if ((
        #         agent[0]-more_margin <= x <= agent[0] + more_margin)
        #     and (
        #         agent[1]-more_margin <= y <= agent[1] + more_margin)):
        #     return True
        if np.linalg.norm(np.array(agent)-np.array([x,y])) < more_margin:
            return True        
    return False


def generate_agents_points(agents_points):
    count_agent = 0

    cnt_agent_fail = 0
    threshold = 1000
    threshold = 1000000
    while count_agent < Num:
        cnt_agent_fail += 1
        if cnt_agent_fail > threshold:
            print('generate_agents_points')
            print('cnt_agent_fail > threshold')
            print(flush=True)
            exit()
        x = random.uniform(+radius, set_xlim-radius)  # 随机选择agent的中心x坐标
        y = random.uniform(+radius, set_ylim-radius)  # 随机选择agent的中心y坐标

        # if (not check_agents_external_collision(x, y, obstacles, more_inflated_size+3*radius)[0] 
        if (not check_agents_external_collision(x, y, obstacles, more_inflated_size+2*radius)[0] 
            and not check_agents_internal_collision(x, y, agents_points)):
            # agent = [(x, y)]  # agent的数据结构为列表，元素为起点和终点的元组[(起点), (终点)]
            agent = (x, y)  # agent的数据结构为元组(x坐标, y坐标)
            agents_points.append(agent)
            count_agent += 1


def make_extend_obstacles(obstacles, radius):
    extend_obstacles=[
        [
            ob[0]-radius,
            ob[1]-radius,
            # ob[2]-radius,
            ob[2]+radius*2,
            # ob[3]+radius*2,
        ]
        for ob in obstacles
    ]
    return extend_obstacles
    
    
# if (sys.argv[0] != 'zread_pickle.py'
#         # and sys.argv[1] == 'datetime'
#         ):
#     # print('generate_obstacles  round :   ',end='')
    
    # # UI 之前
    # print('generate_obstacles  round :   ')
    # for round in range(20):    
    #     # print(round, end=',  ')
    #     print('round')
    #     print(round)
    #     obstacles=[]
    #     ok=generate_obstacles()
    #     if ok:
    #         print("\nobstacles:")
    #         # for obstacle in obstacles:
    #         #     print(f"位置：({obstacle[0]}, {obstacle[1]})，边长：{obstacle[2]}")
    #         print(obstacles)
    #         break
    # else:
    #     print('generate_obstacles')
    #     print('cnt_ob_fail > threshold')
    #     exit()
    # # extend_obstacles=make_extend_obstacles(obstacles, radius)
    # print('generate_obstacles  ok')
    # print('len(obstacles)')
    # print(len(obstacles))
    # generate_agents_points(agents_starts)
    # print("\nAgents_start:")
    # print(agents_starts)
    # generate_agents_points(agents_ends)
    # print("\nAgents_ends:")
    # print(agents_ends)

import shared_util.io_filename as iof
import shared_util.sys_argument as ag
description = iof.DIOM(ag.get_datetime()).load()
# 障碍物、智能体起点、智能体终点
# 优先使用YAML参数中的障碍物，如果YAML中没有或为空则从description加载
import zyaml as zy
yaml_obstacles = zy.parameters.get('obstacle_list', [])
# 如果YAML中障碍物为空，则从description.json加载
if not yaml_obstacles:
    obstacles = description['obstacle_list']
else:
    obstacles = yaml_obstacles  # 存储障碍物的列表
# extend_obstacles = []  # 存储障碍物的列表
agents_starts = description['agent_start_list']  # 存储agent的列表
agents_ends = description['agent_end_list']  # 存储agents_ends的列表
UnmannedSystem_list = description['UnmannedSystem_list']  



# 打印障碍物和agents
def obstacles_agents_print():
    print("\nobstacles:")
    # for obstacle in obstacles:
    #     print(f"位置：({obstacle[0]}, {obstacle[1]})，边长：{obstacle[2]}")
    print(obstacles)

    print("\nAgents_start:")
    # for agent in agents:
    #     print(f"位置：({agent[0]}, {agent[1]})")
    print(agents_starts)

    print("\nAgents_ends:")
    # for agent in agents_ends:
    #     print(f"位置：({agent[0]}, {agent[1]})")
    print(agents_ends)


def obstacles_agents_Fig():
    # 绘制障碍物和Agent的位置图形
    fig, ax = plt.subplots()
    ax.set_xlim(0, set_xlim)
    ax.set_ylim(0, set_ylim)
    ax.set_aspect('equal')

    for obstacle in obstacles:
        x, y, size = obstacle
        rect = plt.Rectangle((x, y), size, size, facecolor='red')
        ax.add_patch(rect)

    for agent in agents_starts:
        x, y = agent
        circle = plt.Circle((x, y), radius, facecolor='blue')
        ax.add_patch(circle)

    for agent in agents_ends:
        x, y = agent
        circle = plt.Circle((x, y), radius, facecolor='green')
        ax.add_patch(circle)

    plt.show()


############################################################
############     对初始化参数进行保存，方便复现错误    #########
############################################################
# # 定义要保存为YAML的变量
# data = {
#     'episodes': episodes,
#     'x_range': set_xlim,
#     'y_range': set_ylim,
#     'area': set_xlim * set_ylim,
#     'rate': ob_rate,

#     'obstacles_area': area * ob_rate,
#     'lower_limit_Square_side_length': lower_limit_Square_side_length,
#     'upper_limit_Square_side_length': upper_limit_Square_side_length,

#     'buffer': buffer,
#     'radius': radius,
#     'bug_step_size': bug_step_size,
#     'inflated_size': inflated_size,

#     'Num': Num,
#     'obstacles': obstacles,
#     'agents': agents_starts,
#     'agents_ends': agents_ends
# }
# data = zy.parameters
# 在zyaml中存储并打印读取到的yaml数据
# 障碍物单独使用output_filename.py存
of.save_pickle(obstacles, 'obstacles')
of.saveJSON(obstacles, 'obstacles')
of.saveJSON(agents_starts, 'agents_starts')
of.saveJSON(agents_ends, 'agents_ends')
obstacles_agents_print()
# yaml 存储在带入模块的时候会自动解析执行一次
# 不要放入函数、类中
# 以上代码就是为了初始化


if __name__ == '__main__':
    obstacles_agents_Fig()
