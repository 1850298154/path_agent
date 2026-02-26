"""
UAV引导路径规划测试脚本
使用initialize()函数从description.json和parameters.yaml初始化智能体
"""
import sys
sys.stdout.reconfigure(encoding='utf-8')
print('设置编码为utf8...')

print('OB_2D  model :: UAV引导测试 模块初始化...')

import output_filename as of
import zstatistics as zs
from plot import *
from trajectory import land
import shutil
import numpy as np
from uav import *
from others import *
from run import *
import SET
import os
import sys

sys.path.append(os.getcwd())

# 设置输出目录
if len(sys.argv) >= 2:
    of.path_dir = sys.argv[1] + '/'

global agent_list

def initialize_from_description():
    """从description.json初始化智能体"""
    global agent_list
    import json

    # 读取description.json
    with open(of.path_dir + 'description.json', 'r') as f:
        description = json.load(f)

    agent_list = []
    for i in range(len(description['agent_start_list'])):
        start_pos = np.array(description['agent_start_list'][i])
        target_pos = np.array(description['agent_end_list'][i])
        agent_type = description['UnmannedSystem_list'][i]

        agent_list.append(uav2D(i, start_pos, target_pos, agent_type, SET.K))

    return agent_list

def intermediate_logs(agent_list, i):
    point_list = [np.round(agent.position[i], decimals=2) for agent in agent_list]
    point_list = np.array(point_list).tolist()
    print(f'当前{len(agent_list)}个智能体的位置信息： ', point_list)

def main():
    # 创建输出目录
    if os.path.exists('savefig'):
        shutil.rmtree('savefig')
    os.mkdir('savefig')

    # 初始化参数
    SET.initialize_set()

    # 从description.json初始化智能体
    global agent_list
    agent_list = initialize_from_description()

    print("已从description.json初始化智能体")
    print(f"智能体数量: {len(agent_list)}")
    print(f"障碍物数量: {len(SET.obstacle_list)}")

    # 运行主循环
    all_time = 0.0

    for i in range(1, SET.episodes + 1):
        start = time.time()

        # 运行一步
        run_one_step(agent_list, SET.obstacle_list)

        time_interval = time.time() - start

        intermediate_logs(agent_list, i - 1)
        print("Step %s have finished, %d agents running time is %s" % (i, SET.Num, time_interval))
        print(flush=True)
        all_time += time_interval

        # 检查是否所有智能体到达终点
        if check_reach_target(agent_list):
            print('所有智能体已到达终点!')
            break

        if check_deadlock(agent_list):
            print('检测到死锁!')
            break

    print('平均重新规划时间: ' + str(all_time / i))

    # 保存运行数据
    save_data(agent_list)

    print('轨迹结果如下：')
    for i, agent in enumerate(agent_list):
        print(i, np.array(agent.pre_traj_list)[:, 0, :].tolist() if agent.pre_traj_list else "无预规划路径")

    # 统计分析
    print("开始统计...")
    zs.fstatistics(agent_list=agent_list)
    of.save_agent100(agent_list)

    # 绘制结果
    print('绘制结果...')
    plot_position(agent_list, SET.ini_obstacle_list, SET.obstacle_list)

    end_date_time = of.get_current_datetime_formatted_file_name()
    print('数据存储结束， 当前时间为： ', end_date_time)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('====================================')
        print("ERROR: ", e)
        print('====================================')
        of.save_agent100(agent_list)
        raise e
