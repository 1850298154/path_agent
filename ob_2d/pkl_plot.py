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
# 、、、、、、、你好 # 测试 pyQT错误输出 和 编码问题

sys.path.append(os.getcwd())

# import      H_SET       as      SET
# import      forest_SET  as      SET

global agent_list
global current_step  # 当前仿真步数

current_step = 0


def initialize():
    # initialization
    global agent_list
    agent_list = []
    for i in range(SET.Num):
        agent_list += [uav2D(i, SET.ini_x[i], SET.target[i],
                             SET.type_list[i], SET.K)]  # zyt:path
    return agent_list


def load_pkl():
    """从 pkl 文件加载 agent_list"""
    import pickle
    filename = of.path_dir + 'agent100/agent_list_100.pkl'
    with open(filename, 'rb') as f:
        agent_list = pickle.load(f)
    print(f'已加载 {len(agent_list)} 个智能体数据')
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


def main():
    # 清理旧的 savefig 目录
    if os.path.exists('savefig'):
        shutil.rmtree('savefig')
    os.mkdir('savefig')

    # 初始化 SET 参数
    SET.initialize_set()

    # 从 pkl 文件加载 agent_list
    global agent_list
    agent_list = load_pkl()

    # 画图
    print('开始画图...')
    if len(sys.argv) >= 4 and sys.argv[3] == 'show_pict':
        # 使用带任务圆球的绘图函数
        plot_position_with_tasks(agent_list, SET.ini_obstacle_list, SET.obstacle_list)
        # 如果需要原来的不带任务的版本，可以取消下面这行注释
        # plot_position(agent_list, SET.ini_obstacle_list, SET.obstacle_list)
        # episodes_path_list=plot_all_pre_traj(agent_list, SET.ini_obstacle_list, SET.obstacle_list, SET.show, -1)  # 暂时注释，先跑通 plot_position

    # 保存 pkl 文件（zyt 验收）暂时注释，避免 shared_util 错误
    # print('保存 pkl 文件...')
    # of.save_agent100(agent_list)

    end_date_time = of.get_current_datetime_formatted_file_name()
    print('画图完成， 当前时间为： ', end_date_time)


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
