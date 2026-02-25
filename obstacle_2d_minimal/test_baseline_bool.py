import sys
sys.stdout.reconfigure(encoding='utf-8')
print('设置编码为utf8...')


print('OB_2D  model :: test 模块初始化...') # 在导入所有【自定义】和【标准】库之后，再执行这句打印


import output_filename as of
import zstatistics as zs
from plot import *
import multiprocessing as mp
from trajectory import land
import shutil
import numpy as np
from uav import *
from others import *
from run import *
import SET  # Python: current File"已在运行。 是否要启动另个实例？
import os
import sys
# 、、、、、、、、、你好 # 测试 pyQT错误输出 和 编码问题

sys.path.append(os.getcwd())

# import      H_SET       as      SET
# import      forest_SET  as      SET

global agent_list

def read_pkl():
    global agent_list
    agent_list =  []
    agent_list = of.read_pickle(agent_list)
    import random
    agent_list = random.sample(agent_list, SET.Num)
    # agent_list = agent_list
    return agent_list


def initialize():

    # inilization
    global agent_list
    agent_list = []
    for i in range(SET.Num):
        agent_list += [uav2D(i, SET.ini_x[i], SET.target[i],
                             SET.type_list[i], SET.K)]  # zyt:path

    return agent_list


def Redirect_standard_output_to_a_file():
    # # 将print输出同时重定向到文件
    # with open('print.txt', 'w') as f:
    #     sys.stdout = f  # 将标准输出重定向到文件
    filename = of.path_dir + 'print.txt'
    of.create_file(file_path=filename)
    f = open(filename, 'w')
    sys.stdout = f  # 将标准输出重定向到文件

    # # 恢复标准输出
    # sys.stdout = sys.__stdout__

    # # 输出在文件中的内容
    # with open('output.txt', 'r') as f:
    #     content = f.read()
    #     print(content)


# def run(agent_list):
#     all_time = 0.0

#     # begin the main loop
#     for i in range(1, SET.episodes+1):

#         start = time.time()

#         # run one step
#         agent_list = run_one_step(agent_list, SET.obstacle_list)

#         time_interval = time.time()-start

#         # print running time in this step
#         print("Step %s have finished, running time is %s" % (i, time_interval))
#         print(" ")
#         print(flush=True)
#         all_time += time_interval
#         # plot the predetermined tarjectories in this time step
#         # plot_pre_traj(agent_list, SET.ini_obstacle_list, SET.show, i)

#         # juding whether all robots reach to their target positions
#         if check_reach_target(agent_list):
#             print('check_reach_target')
#             break

#         if check_deadlock(agent_list):
#             print('check_deadlock True')
#             break

#     print('the mean time per replanning is:'+str(all_time/i))
#     if i < SET.episodes:
#         print('End of SUCCESS')
#     elif i == SET.episodes:
#         print('End of FAIL')

def intermediate_logs(agent_list, i):
    # print('intermediate_logs:')
    # print()
    # point_list = [ np.round(agent.p, decimals=2) for agent in agent_list]
    point_list = [ np.round(agent.position[i], decimals=2) for agent in agent_list]
    point_list = np.array(point_list).tolist()
    print(f'当前{len(agent_list)}个智能体的位置信息： ', point_list)


import uav
import numpy as np

def calculate_positions_path(start, end, path, vmax, interval):
    # 将起点、路径点和终点统一转换为 NumPy 数组
    start = np.array(start)
    end = np.array(end)
    path = [np.array(p) for p in path]
    
    # 将起点、路径点和终点合并成一个完整的路径
    full_path = [start] + path + [end]
    
    positions = []
    current_position = np.array(start)
    
    for next_point in full_path[1:]:
        next_point = np.array(next_point)
        direction = (next_point - current_position) / np.linalg.norm(next_point - current_position)
        
        while np.linalg.norm(next_point - current_position) > vmax * interval:
            current_position = current_position + direction * vmax * interval
            positions.append(current_position.tolist())
        
        # 到达路径点
        current_position = next_point
        positions.append(current_position.tolist())
    
    return positions

    start = np.array(start)
    end = np.array(end)
    distance = np.linalg.norm(end - start)
    direction = (end - start) / distance
    positions = [start]

    current_position = start
    while np.linalg.norm(current_position - end) > vmax * interval:
        current_position = current_position + direction * vmax * interval
        positions.append(current_position)

    positions.append(end)
    return positions
def calculate_positions(start, end, vmax, interval):
    start = np.array(start)
    end = np.array(end)
    distance = np.linalg.norm(end - start)
    direction = (end - start) / distance
    positions = [start]

    current_position = start
    while np.linalg.norm(current_position - end) > vmax * interval:
        current_position = current_position + direction * vmax * interval
        positions.append(current_position)

    positions.append(end)
    return positions

from dead_record import pair_timeline, dead_timeline, g_dead_list, g_p_list
from typing import List

def calculate_distance_distribution(agent_list:List[uav.uav2D],maxlen):
    global g_p_list
    l = (len(agent_list))
    for step in range(maxlen):
        for i in range(l):
            agent = agent_list[i]
            for j in range(i+1, l):
                other_agent = agent_list[j]
                if agent != other_agent:
                    dis = np.linalg.norm(agent.pre_traj_list[step][1] 
                                -  other_agent.pre_traj_list[step][1]) 
                    g_p_list.append(pair_timeline(i, j, step, dis))
    sorted_g_p_list = sorted(g_p_list, key=lambda p: p.dis)
    g_p_list = sorted_g_p_list
    # print("for p in g_p_list:")
    # for p in g_p_list:
    #     print(p.dis, ' ')
    # print()
    # print('g_p_list')
    # print(g_p_list)
    return g_p_list
from typing import List

def check_for_internal_collisions_and_stop(agent_list:List[uav.uav2D], step):
    l = (len(agent_list))
    dead_ij_list = []
    for i in range(l):
        agent = agent_list[i]
        for j in range(step+1, l):
            other_agent = agent_list[j]
            if agent != other_agent:
                dis1 = np.linalg.norm(agent.pre_traj_list[step][1] 
                                - other_agent.pre_traj_list[step][1]) 
                dis2 = np.linalg.norm(agent.pre_traj_list[step][1] 
                                - other_agent.pre_traj_list[step][1]) 
                threshold = 2*2*agent.physical_r_min
                # if np.linalg.norm(agent.position[i] - other_agent.position[i]) < 2 * agent.r_max:
                if (
                        dis1 < threshold
                        or
                        dis2 < threshold
                        ):
                    # g_p_list.append(pair_timeline(i, j, step, dis))
                    from dead_record import pair_timeline, dead_timeline, g_dead_list, g_p_list
                    g_dead_list.append(
                        dead_timeline(i, j, step, dis1, 
                                      agent.position[step],
                                      other_agent.position[step],
                                      agent, other_agent))  
                    print('threshold')
                    print(threshold)
                    print('dis')
                    print(dis1)
                    print('Agent %d and Agent %d have internal collision at step %d' % (agent.index, other_agent.index, step))
                    print('Agent %d: %s' % (agent.index, agent.position[step]))
                    print('Agent %d: %s' % (other_agent.index, other_agent.position[step]))
                    print('Agent %d: next %s' % (agent.index,             agent.pre_traj_list[step][1]))
                    print('Agent %d: next %s' % (other_agent.index, other_agent.pre_traj_list[step][1]))
                    print(' ')
                    # 处理在这里      
                    dead_ij_list.append([i,j])
                    for j in range(step+1, len(agent.pre_traj_list)):
                        agent.pre_traj_list[j] = agent.pre_traj_list[j-1]
                        agent.position[j] = agent.position[j-1]
                        other_agent.pre_traj_list[j] = agent.pre_traj_list[j-1]
                        other_agent.position[j] = agent.position[j-1]
                    # print('agent.pre_traj_list')
                    # print(agent.pre_traj_list)
                    # print('agent.position')
                    # print(agent.position)
                    # return True
                    break
    # for ij in dead_ij_list:
    #     i,j = ij
    #     for k in range(step+1, len(agent.pre_traj_list)):
    #         agent_list[i].pre_traj_list[k] = agent.pre_traj_list[k-1]
    #         agent_list[i].position[k] = agent.position[k-1]
    #         agent_list[j].pre_traj_list[k] = agent.pre_traj_list[k-1]
    #         agent_list[j].position[k] = agent.position[k-1]

    from dead_record import pair_timeline, dead_timeline, g_dead_list, g_p_list
    sorted_g_dead_list = sorted(g_dead_list, key=lambda p: p.dis)
    g_dead_list = sorted_g_dead_list
    # return False


    
from typing import List

def run_baseline(agent_list:List[uav.uav2D]):
    print (' run_baseline 求解模型运行开始...')
    """
        k=3
        len ==     5
         i _ _ R
        [0,1,2,3,4,]
           i _ _ R
        [0,1,2,3,4,]
             i _ _ R
        [0,1,2,3,4,]
               i _ _ R
        [0,1,2,3,4,] 6
                 i _ _ R
        [0,1,2,3,4,] 6 7
            len  = 5
    """
    for agent in agent_list:

        agent.position = []
        agent.pre_traj_list = []
        agent.position = calculate_positions_path(
            start    = agent.ini_p ,
            end      = agent.target,
            path     = agent.path  ,
            vmax     = agent.Vmax  ,
            interval = agent.h     ,
        )
        # agent.position = calculate_positions(
        #     start    = agent.ini_p ,
        #     end      = agent.target,
        #     vmax     = agent.Vmax  ,
        #     interval = agent.h     ,
        # )
        # print('agent.position')
        # print(agent.position)
        # print(agent.pre_traj_list)
        # print('agent.pre_traj_list')
        pLen = len(agent.position)
        for i in range(pLen):
            # i + agent.K 左闭右开
            Ropen = i + agent.K
            if Ropen > pLen:
                K_positon = (
                        agent.position[i:]
                        + 
                        [agent.position[-1]]
                        * (Ropen-pLen)
                    )
            else:
                K_positon = agent.position[i : i + agent.K]
            agent.pre_traj_list.append(K_positon)

    # print(agent.position)
    # print(agent.pre_traj_list)
    # input('agent.pre_traj_list')

    maxlen = max(
        [len(agent.pre_traj_list) for agent in agent_list]
    )
    print('maxlen')
    print(maxlen)
    print('agent.pre_traj_list')
    print([len(agent.pre_traj_list) for agent in agent_list])
    print([len(agent.position) for agent in agent_list])
    
    for agent in agent_list:
        if len(agent.pre_traj_list) < maxlen:
            for i in range(maxlen - len(agent.pre_traj_list)):
                agent.pre_traj_list.append(agent.pre_traj_list[-1])
                agent.position.append(agent.position[-1])
    print('maxlen')
    print(maxlen)
    print('agent.pre_traj_list')
    print([len(agent.pre_traj_list) for agent in agent_list])
    print([len(agent.position) for agent in agent_list])
    # print(agent.pre_traj_list)
    # print(agent.position)

    for agent in agent_list:
        agent.position = np.array(agent.position)
        agent.pre_traj_list = np.array(agent.pre_traj_list)

    from dead_record import pair_timeline, dead_timeline, g_dead_list, g_p_list
    # 计算所有智能体在每一步的距离分布
    # g_p_list = calculate_distance_distribution(agent_list,maxlen)

    
    # global agent_list
    all_time = 0.0

    # begin the main loop
    # for i in range(1, SET.episodes+1):
    for i in range(1, maxlen+1):
        check_for_internal_collisions_and_stop(agent_list, i-1)

        start = time.time()

        # run one step
        # agent_list = run_one_step(agent_list, SET.obstacle_list)

        time_interval = time.time()-start

        # print running time in this step
        # print(flush=True)
        intermediate_logs(agent_list, i-1)
        print("Step %s have finished, %d agents running time is %s" % (i, SET.Num, time_interval))
        # print("agentlist",agent_list)
        print(flush=True)
        all_time += time_interval
        # plot the predetermined tarjectories in this time step
        # plot_pre_traj(agent_list, SET.ini_obstacle_list, SET.show, i)

        # juding whether all robots reach to their target positions
        # if check_reach_target(agent_list):
        #     print('check_reach_target')
        #     break
        
        if i+1 == len(agent_list[0].pre_traj_list) :
            break

        # if check_deadlock(agent_list):
        #     print('check_deadlock True')
        #     break

    print('the mean time per replanning is:'+str(all_time/i))
    # print('the mean time per replanning is:'+str(all_time))
    if i < SET.episodes:
        # print('End of SUCCESS')
        print('i < SET.episodes')
    elif i == SET.episodes:
        # print('End of FAIL')
        print('i == SET.episodes')
    # print('agent_list')
    # print(len(agent_list))
    # print(agent_list)
    # try:
    #     run(agent_list)
    # except Exception as e:
    #     print('***********************************')
    #     print("The Last Line of Defense : ", e)
    #     print('***********************************')

    # save the running data
    save_data(agent_list)

    print('轨迹结果如下：')
    for i, agent in enumerate(agent_list):
        print(i, np.array(agent.pre_traj_list)[:,0,:].tolist())
    
    # # plot trajectories in this test

    print("*list(parameters.items())")
    print(*list(zy.parameters.items()),sep='\n')
    zs.fstatistics(agent_list=agent_list)
    of.save_agent100(agent_list)# zyt 验收取消输出
    # print('agent_list')
    # print(len(agent_list))
    # print(agent_list)
    # UI test 2023.10.12
    print(' run_baseline 求解模型运行完毕， 开始存储运行结果数据...')
    plot_position(agent_list, SET.ini_obstacle_list, SET.obstacle_list, twice_base_line_bool=True)
    # if len(sys.argv)>=4:
    #     if sys.argv[3] == 'show_pict':
    #         episodes_path_list=plot_all_pre_traj(agent_list, SET.ini_obstacle_list, SET.obstacle_list, SET.show, -1)
    #         # # print(*episodes_path_list, sep='\n')
    #         # # import jpg2mp4
    #         # # jpg2mp4.images_to_video(episodes_path_list)
    #         # import shared_util.io_filename as iof
    #         # iof.VIOM(of.datetime_from_sh()).images_to_video(episodes_path_list)
    end_date_time = of.get_current_datetime_formatted_file_name()
    print('数据存储结束， 当前时间为： ', end_date_time)


if __name__ == '__main__':
    # Redirect_standard_output_to_a_file()
    # main()
    try:
        main()
    except Exception as e:
        print('***********************************')
        print("The Last Line of Defense :: ERROR is :: ", e)
        print('***********************************')
        of.save_agent100(agent_list)
        raise e
