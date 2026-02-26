# 简化测试 - 3个智能体，2个障碍物
import sys
import os
import numpy as np
import pickle

# 导入必要模块
from plot import *
from uav import *

def read_pkl(pkl_path):
    with open(pkl_path, 'rb') as f:
        return pickle.load(f)

def main():
    # 直接使用参数文件所在的目录
    test_dir = os.path.dirname(sys.argv[1])

    # 初始化agents
    agent_list = []
    for i in range(3):
        agent = uav2D(i, [50.0, 50.0], 'unicycle', 10.0)
        agent_list.append(agent)

    # 运行仿真循环
    for step in range(50):
        print(f'Step {step}: agents positions', [agent.p.tolist() for agent in agent_list])

        # 绘制
        if step == 49:
            from plot import plot_position
            plot_position(agent_list, [], [])
            print('绘制轨迹图...')

if __name__ == '__main__':
    main()
