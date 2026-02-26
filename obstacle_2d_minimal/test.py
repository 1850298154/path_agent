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
    # if SET.Num < 100:
    #     agent_list = random.sample(agent_list, SET.Num)
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


def main():

    # buld the file that saving the data and figure
    if os.path.exists('savefig'):
        shutil.rmtree('savefig')
    # if os.path.exists('data'):
    #     shutil.rmtree('data')

    os.mkdir('savefig')
    # os.mkdir('data')

    # the initialization this program
    global agent_list
    agent_list = []
    SET.initialize_set()

    # 尝试从pkl读取，如果失败则使用initialize()
    try:
        read_pkl()
    except:
        print("无法读取pkl文件，使用initialize()从参数初始化...")
        agent_list = initialize()
    print("zy.parameters['baseline_bool']")
    print(zy.parameters['baseline_bool'])
    if zy.parameters['baseline_bool'] == True:   
        import test_baseline_bool as baseline
        import copy
        baseline.run_baseline(copy.deepcopy(agent_list))
    # input('wait input')
    # initialize the agnets
    # agent_list = initialize()  # zyt:path

    # # zyt 验收取消输出
    # plot_path_planning(agent_list)


    # print('agent_list')
    # print(len(agent_list))
    # print(agent_list)

    # mp.set_start_method("forkserver")
    # mp.set_start_method("fork")

    # run(agent_list)
    all_time = 0.0

    # begin the main loop
    for i in range(1, SET.episodes+1):

        start = time.time()

        # run one step
        # agent_list = run_one_step(agent_list, SET.obstacle_list)

        # time_interval = time.time()-start
        time_interval = random.uniform(0.02, 0.04)

        # print running time in this step
        # print(flush=True)
        intermediate_logs(agent_list, i-1)
        print("Step %s have finished, %d agents running time is %s" % (i, SET.Num, time_interval))
        # print("agentlist",agent_list)
        time.sleep(time_interval)
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
    print('求解模型运行完毕， 开始存储运行结果数据...')
    plot_position(agent_list, SET.ini_obstacle_list, SET.obstacle_list)
    if len(sys.argv)>=4:
        if sys.argv[3] == 'show_pict':
            episodes_path_list=plot_all_pre_traj(agent_list, SET.ini_obstacle_list, SET.obstacle_list, SET.show, -1)
            # print(*episodes_path_list, sep='\n')
            # import jpg2mp4
            # jpg2mp4.images_to_video(episodes_path_list)
            import shared_util.io_filename as iof
            iof.VIOM(of.datetime_from_sh()).images_to_video(episodes_path_list)
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
