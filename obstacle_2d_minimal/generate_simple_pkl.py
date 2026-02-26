"""
生成简化的agent_list_100.pkl文件
"""
import sys
import json
import numpy as np
import pickle

# 设置输出目录（使用命令行参数或默认值）
import os
if len(sys.argv) >= 2:
    output_dir = sys.argv[1]
else:
    output_dir = "004/uav_guided_scenario"

# 读取description.json
with open(f"{output_dir}/description.json", 'r') as f:
    description = json.load(f)

# 创建简化的UAV对象列表
class SimpleUAV:
    def __init__(self, index, start, target, uav_type):
        self.index = index
        self.UnmannedSystem = uav_type
        self.ini_p = np.array(start)
        self.target = np.array(target)
        self.type = uav_type
        self.K = 10
        self.p = np.array(start)  # current position
        self.position = []  # empty list to avoid IndexError

agent_list = []
for i in range(len(description['agent_start_list'])):
    uav = SimpleUAV(
        index=i,
        start=description['agent_start_list'][i],
        target=description['agent_end_list'][i],
        uav_type=description['UnmannedSystem_list'][i]
    )
    agent_list.append(uav)

# 保存pkl文件
pkl_path = f"{output_dir}/agent100/agent_list_100.pkl"
import os
os.makedirs(f"{output_dir}/agent100", exist_ok=True)
with open(pkl_path, 'wb') as f:
    pickle.dump(agent_list, f)

print(f"已生成 {pkl_path}，包含 {len(agent_list)} 个智能体")
