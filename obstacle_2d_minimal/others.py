import zyaml as zy
import numpy as np
import time
from geometry import *
import output_filename as of


def get_share_data(agent_list):

    pre_traj_list = []
    type_list = []
    priority_list = []
    distance_list = []
    contest_list = []
    eta_list = []

    for agent in agent_list:
        pre_traj_list += [agent.pre_traj]
        type_list += [agent.type]
        priority_list += [agent.priority]
        distance_list += [agent.distance]
        contest_list += [agent.contest]
        eta_list += [agent.eta]

    share_data = {'pre_traj': pre_traj_list, 'type': type_list,
                  'priority': priority_list, 'distance': distance_list, 'contest': contest_list}

    print("Priority of agents: " + str(priority_list))

    return share_data


def check_reach_target(agent_list):

    REACHGOAL = True

    for agent in agent_list:

        if agent.type == 'Searcher' and agent.cost_index == 0:
            # 我们是 Obstacle 不是 searcher
            flag = True
            for agent in agent_list:
                # if np.linalg.norm(agent.pre_traj[0]-agent.pre_traj[-1]) > 1e-2:
                # if np.linalg.norm(agent.pre_traj[0]-agent.pre_traj[-1]) > zy.parameters['radius']:
                if np.linalg.norm(agent.pre_traj[0]-agent.pre_traj[-1]) > 0.01*zy.parameters['physical_radius']:
                    flag = False
                    break
            if flag:
                return True
        if not agent.cost_index == 0:
            # 检查， 只要有一个 没有到终点 ， 就终止
            return False

    return REACHGOAL


def check_deadlock(agent_list):
    for agent in agent_list:
        # 没有到达终点
        if not agent.cost_index == 0:
            # 且一动不动
            # print('agent.pre_traj_list')
            # print(type(agent.pre_traj_list))
            # print(agent.pre_traj_list)
            # print(len(agent.pre_traj_list))
            if len(agent.pre_traj_list) <= 10:
                continue
            # if np.linalg.norm(agent.pre_traj_list[-1][0] -
            #                   agent.pre_traj_list[-2][0]) < 1e-2:
            if np.linalg.norm(agent.pre_traj_list[-1][0] -
                            #   agent.pre_traj_list[0][0]) < zy.parameters['radius']:
                              agent.pre_traj_list[-10][0]) < 0.001:
                # agent.deadlock_list.append(len(agent.pre_traj_list))
                agent.deadlock_info.append(len(agent.pre_traj_list))
                agent.deadlock = True  # 小于等于半径，算到终点。 必须要脚踩目标，或这边触及目标
                
                break  # 到达终点， 就不算是死锁了。
    flag = True
    for agent in agent_list:
        # 全员 到达终点、或者死锁
        # 只要有 一个 没到终点并且也没死锁，
        # 就继续运行程序
        if ((not agent.cost_index == 0)
                and (agent.deadlock == False)):
            return False
    # print()
    return flag


def save_data(agent_list):
    return # zyt 不要输出无用东西
    data = agent_list[0].data

    for i in range(1, len(agent_list)):
        if agent_list[i].type != "Anchor":
            data = np.block([[data, agent_list[i].data]])

    filename = 'data/data.csv'
    filename = (
        of.path_dir
        + filename
    )
    of.create_file(file_path=filename)
    np.savetxt(filename, data)


def save_path(agent_list, km):

    data = agent_list[0].path

    for i in range(1, len(agent_list)):
        if agent_list[i].type != "Anchor":
            data = np.block(
                [[data], [-9999999*np.ones(2)], [agent_list[i].path]])
    filename = 'data/'+str(km)+'path.csv'
    filename = (
        of.path_dir
        + filename
    )
    of.create_file(file_path=filename)
    np.savetxt(filename, data)


def get_sample_position(Num, obstacle_list, r_min, square):

    d = round(len(square)/2)

    a = time.time()*1e8
    a = np.round(a)
    np.random.seed(a.astype(int) % 46547)

    while True:

        ini_x = []

        for i in range(Num):

            for j in range(1000):

                ini = np.array(square[0:d]) + \
                    np.random.rand(d)*np.array(square[d:2*d])
                RIGHT = True

                if detect_point_in(obstacle_list, ini):
                    RIGHT = False
                    break

                for k in range(len(ini_x)):
                    if(np.linalg.norm(ini-ini_x[k]) < r_min):
                        RIGHT = False
                        break

                if RIGHT:
                    ini_x += [ini]
                    break

        if len(ini_x) == Num:
            break

    print("Get sample positions")
    return ini_x
