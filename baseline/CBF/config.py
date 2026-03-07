# CBF基线算法配置文件
import json
import yaml
import os
import numpy as np

class Config:
    """配置管理类"""

    def __init__(self, description_path=None, parameters_path=None):
        # 默认路径
        if description_path is None:
            description_path = r"D:\zyt\git_ln\path_agent\ob_2d\004\2023-10-31_20-42-54\description.json"
        if parameters_path is None:
            parameters_path = r"D:\zyt\git_ln\path_agent\ob_2d\004\2023-10-31_20-42-54\parameters.yaml"

        # 加载配置
        self.load_description(description_path)
        self.load_parameters(parameters_path)

        # 测试模式参数（开发时使用）
        self.test_mode = False
        self.test_num_agents = 5
        self.test_max_steps = 50

    def load_description(self, path):
        """加载场景描述文件"""
        with open(path, 'r', encoding='utf-8') as f:
            desc = json.load(f)

        self.agent_start_list = [np.array(p) for p in desc['agent_start_list']]
        self.agent_end_list = [np.array(p) for p in desc['agent_end_list']]

        # 障碍物: [x, y, side_length]
        self.obstacle_list = desc['obstacle_list']
        self.num_agents = len(self.agent_start_list)
        self.num_obstacles = len(self.obstacle_list)

    def load_parameters(self, path):
        """加载参数文件"""
        with open(path, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)

        # 智能体参数
        self.num = params.get('agent.Num', 100)
        self.umax = params.get('agent.Umax', 40.0)  # 最大加速度
        self.vmax = params.get('agent.Vmax', 3.0)   # 最大速度
        self.physical_radius = params.get('agent.physical_radius', 0.25)
        self.radius = params.get('agent.radius', 1.0)  # 安全半径

        # 地图参数
        self.map_xlim = params.get('map.set_xlim', 300)
        self.map_ylim = params.get('map.set_ylim', 300)

        # 时间参数
        self.h = 0.2  # 时间步长
        self.max_steps = 863  # 最大步数

    def set_test_mode(self, num_agents=5, max_steps=50):
        """设置测试模式"""
        self.test_mode = True
        self.test_num_agents = num_agents
        self.test_max_steps = max_steps
        print(f"[测试模式] 智能体数量: {num_agents}, 最大步数: {max_steps}")

    def get_num_agents(self):
        """获取智能体数量"""
        if self.test_mode:
            return self.test_num_agents
        return self.num_agents

    def get_max_steps(self):
        """获取最大步数"""
        if self.test_mode:
            return self.test_max_steps
        return self.max_steps


# 全局配置实例
config = Config()
