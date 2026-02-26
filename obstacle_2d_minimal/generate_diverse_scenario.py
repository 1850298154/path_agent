"""
生成多样化UAV引导场景 - 手工指定起点和终点，确保路径多样性
"""
import json
import os

# 输出目录
OUTPUT_DIR = '004/2026-02-26_14-55-00'

# 地图尺寸
MAP_SIZE = 330.0
MAP_MARGIN = 30  # 地图边距

# UAV分析范围
ANALYSIS_RANGE = {
    'x_min': 0.3, 'x_max': 30.09,
    'y_min': 0.3, 'y_max': 25.71
}

# 手工配置的智能体起点和终点
# 格式: [agent_index, x, y, target_x, target_y]
MANUAL_CONFIG = [
    # Agent 0-3: 从基地1出发，不同终点
    [0, 30, 30, 45, 280],
    [1, 30, 30, 60, 290],
    [2, 30, 30, 75, 295],
    [3, 30, 30, 90, 210],
    # Agent 4-6: 从基地2出发，分散分布
    [4, 120, 30, 50, 280],
    [5, 180, 30, 55, 250],
    [6, 240, 30, 100, 220],
    # Agent 7-9: 从基地2出发，分散分布
    [7, 200, 30, 200, 200],
    [8, 260, 30, 150, 150],
    [9, 280, 30, 120, 120],
]

def scale_position(pos, x, y):
    """缩放坐标到障碍物地图"""
    usable_size = MAP_SIZE - 2 * MAP_MARGIN
    x_scale = usable_size / (ANALYSIS_RANGE['x_max'] - ANALYSIS_RANGE['x_min'])
    y_scale = usable_size / (ANALYSIS_RANGE['y_max'] - ANALYSIS_RANGE['y_min'])

    scaled_x = MAP_MARGIN + (x - ANALYSIS_RANGE['x_min']) * x_scale
    scaled_y = MAP_MARGIN + (y - ANALYSIS_RANGE['y_min']) * y_scale
    return [scaled_x, scaled_y]

def main():
    print("="*60)
    print("多样化UAV场景生成器")
    print("="*60)

    # 读取UAV分析数据（用于障碍物信息）
    uav_analysis_path = '../uav_allocation_analysis/precomputed_data.json'
    with open(uav_analysis_path, 'r') as f:
        precomputed = json.load(f)
    obstacle_list = precomputed['obstacle_list']

    # 创建输出目录
    if os.path.exists(OUTPUT_DIR):
        import shutil
        shutil.rmtree(OUTPUT_DIR)
    os.makedirs(OUTPUT_DIR)
    os.makedirs(f'{OUTPUT_DIR}/savefig')

    # 生成agent配置
    agent_start_list = []
    agent_end_list = []
    unmanned_system_list = []

    agent_types = ['unicycle', 'Mecanum', 'Quadcopter', 'patrol_missile', 'fixed_wing']

    for idx, config in enumerate(MANUAL_CONFIG):
        agent_index, start_x, start_y, target_x, target_y = config

        # 智能体类型（循环使用）
        uav_type = agent_types[idx % len(agent_types)]

        # 添加到列表
        agent_start_list.append([start_x, start_y])
        agent_end_list.append([target_x, target_y])
        unmanned_system_list.append(uav_type)

    # 生成障碍物（使用原UAV障碍物数据，缩放坐标）
    scaled_obstacle_list = []
    for obs in obstacle_list:
        # obs格式: [x, y, radius]
        scaled_pos = scale_position([obs[0], obs[1]], obs[0], obs[1])
        scaled_radius = obs[2] * (330 / 30)  # 简化缩放
        scaled_obstacle_list.append([scaled_pos[0], scaled_pos[1], scaled_radius])

    description = {
        "agent_start_list": agent_start_list,
        "agent_end_list": agent_end_list,
        "obstacle_list": scaled_obstacle_list,
        "UnmannedSystem_list": unmanned_system_list
    }

    # 保存description.json
    description_path = os.path.join(OUTPUT_DIR, 'description.json')
    with open(description_path, 'w') as f:
        json.dump(description, f, indent=2)
    print(f"  - 生成 description.json")
    print(f"    智能体数量: {len(agent_start_list)}")

    # 生成parameters.yaml
    ini_x_yaml = "    - " + "\n    - ".join([f"[{pos[0]:.2f}, {pos[1]:.2f}]" for pos in agent_start_list])
    target_yaml = "    - " + "\n    - ".join([f"[{pos[0]:.2f}, {pos[1]:.2f}]" for pos in agent_end_list])
    type_yaml = "    - " + "\n    - ".join([f'    - {t}' for t in unmanned_system_list])

    parameters = f"""# 障碍物路径规划参数
# 多样化UAV场景

# ==================== 智能体参数 ====================
agent.Num: {len(agent_start_list)}
agent.Umax: 10.0
agent.Vmax: 3.0
agent.physical_radius: 0.2
agent.radius: 0.8

# 智能体初始位置
ini_x:
{ini_x_yaml}

# 智能体目标位置
target:
{target_yaml}

# 智能体类型列表
type_list:
{type_yaml}

# 智能体控制增益
K: 1.0

# ==================== 地图参数 ====================
map.set_xlim: 330
map.set_ylim: 330
map.set_zlim: 330

# ==================== 障碍物参数 ====================
ob.apart_num: 2
ob.lower_limit_Square_side_length: 20.56194904089777
ob.num: 9
ob.rate: 0.05
ob.upper_limit_Square_side_length: 20.56194904089777

# ==================== BUG算法参数 ====================
bug.bug_step_size: 20
bug.lower_limit_inflated_size: 7.2
bug.num_tracks: 4
bug.upper_limit_inflated_size: 11.2

# ==================== 其他参数 ====================
baseline_bool: false
model.SpaceEnvironmentModel.dimension: 2
model.SpaceEnvironmentModel.withObstacles: true
model.TypicalScene.id: Random
model.UnmannedSystem.id: unmanned_vehicle
mpc.K: -1
mpc.epsilon: -1
mpc.max_episode: 2200
round.current: 1
round.only_one: false
round.total: 1
"""

    parameters_path = os.path.join(OUTPUT_DIR, 'parameters.yaml')
    with open(parameters_path, 'w') as f:
        f.write(parameters)
    print(f"  - 生成 parameters.yaml")

    # 打印配置信息
    print()
    print("=== 智能体配置 ===")
    for i in range(len(agent_start_list)):
        print(f"  Agent {i}: {unmanned_system_list[i]}")
        print(f"    起点: [{agent_start_list[i][0]:.2f}, {agent_start_list[i][1]:.2f}]")
        print(f"    终点: [{agent_end_list[i][0]:.2f}, {agent_end_list[i][1]:.2f}]")

    print()
    print("="*60)
    print("场景生成完成！")
    print(f"\\n运行命令:")
    print(f"  python test.py diverse_uav_scenario {OUTPUT_DIR}/parameters.yaml not_show")
