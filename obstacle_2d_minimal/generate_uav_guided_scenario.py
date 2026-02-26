"""
将UAV分析结果转换为障碍物路径规划场景

功能:
1. 读取uav_allocation分析的UAV路径数据
2. 选择指定数量的UAV
3. 缩放坐标到障碍物系统地图范围(330x330)
4. 生成description.json和parameters.yaml
"""
import json
import os
import shutil
import numpy as np

# ==================== 配置参数 ====================
# UAV分析数据路径
UAV_DATA_PATH = '../uav_allocation_analysis/uav_positions_over_time.json'

# 障碍物系统输出路径
OUTPUT_DIR = '005/uav_guided_scenario'

# 选择的UAV ID列表（从0-79中选择，最多10个）
# 策略：混合选择两个基地的UAV，确保路径多样性
SELECTED_UAV_IDS = [0, 1, 2, 3, 4, 10, 20, 30, 40, 50]

# 地图尺寸
MAP_SIZE = 330.0
MAP_MARGIN = 30  # 地图边距

# UAV分析范围
ANALYSIS_RANGE = {
    'x_min': 0.3, 'x_max': 30.09,
    'y_min': 0.3, 'y_max': 25.71
}

# 智能体类型
AGENT_TYPES = [
    'unicycle', 'Mecanum', 'Quadcopter', 'patrol_missile', 'fixed_wing'
]

# ==================== 坐标转换 ====================
def scale_position(pos, analysis_range, map_size, margin):
    """将UAV分析坐标缩放到障碍物地图范围"""
    x, y = pos

    # 可用地图区域
    usable_size = map_size - 2 * margin

    # 缩放因子
    x_scale = usable_size / (analysis_range['x_max'] - analysis_range['x_min'])
    y_scale = usable_size / (analysis_range['y_max'] - analysis_range['y_min'])

    # 缩放后的坐标（居中放置，保留边距）
    scaled_x = margin + (x - analysis_range['x_min']) * x_scale
    scaled_y = margin + (y - analysis_range['y_min']) * y_scale

    return [scaled_x, scaled_y]

# ==================== 数据处理 ====================
def load_uav_positions():
    """加载UAV位置数据"""
    with open(UAV_DATA_PATH, 'r') as f:
        return json.load(f)

def extract_uav_trajectory(uav_data, uav_id):
    """提取单个UAV的轨迹"""
    init_pos = uav_data['init_pos']
    times = uav_data['times']
    positions = uav_data['positions']

    # 提取起点、终点和中间关键点
    n_points = len(positions)

    # 起点
    start = positions[0]

    # 终点（最后一个有效位置）
    end = positions[-1]

    # 中间关键点（均匀采样5个点）
    middle_indices = np.linspace(0, n_points-1, 5).astype(int)
    waypoints = [positions[i] for i in middle_indices]

    return {
        'start': start,
        'end': end,
        'waypoints': waypoints,
        'full_positions': positions,
        'times': times
    }

# ==================== 场景生成 ====================
def generate_description_json(uav_data):
    """生成description.json"""
    agent_start_list = []
    agent_end_list = []
    obstacle_list = []

    # 为每个选中的UAV生成起点和终点
    for uav_id in SELECTED_UAV_IDS:
        trajectory = extract_uav_trajectory(uav_data['uavs'][str(uav_id)], uav_id)

        # 缩放坐标
        start_scaled = scale_position(trajectory['start'], ANALYSIS_RANGE, MAP_SIZE, MAP_MARGIN)
        end_scaled = scale_position(trajectory['end'], ANALYSIS_RANGE, MAP_SIZE, MAP_MARGIN)

        agent_start_list.append(start_scaled)
        agent_end_list.append(end_scaled)

    # 生成障碍物（示例：9个圆形障碍物）
    obstacle_centers = [
        [50, 100], [200, 50], [100, 200],
        [150, 150], [250, 200], [80, 250],
        [280, 100], [180, 80], [250, 280]
    ]
    obstacle_radius = 30

    for center in obstacle_centers:
        obstacle_list.append([center[0], center[1], obstacle_radius])

    # 生成智能体类型列表
    unmanned_system_list = []
    for i in range(len(SELECTED_UAV_IDS)):
        unmanned_system_list.append(AGENT_TYPES[i % len(AGENT_TYPES)])

    description = {
        "agent_start_list": agent_start_list,
        "agent_end_list": agent_end_list,
        "obstacle_list": obstacle_list,
        "UnmannedSystem_list": unmanned_system_list
    }

    return description, agent_start_list, agent_end_list

def generate_parameters_yaml(agent_start_list, agent_end_list):
    """生成parameters.yaml"""
    # 格式化起点和终点列表
    ini_x_yaml = "    - " + "\n    - ".join([f"[{pos[0]:.2f}, {pos[1]:.2f}]" for pos in agent_start_list])

    target_yaml = "    - " + "\n    - ".join([f"[{pos[0]:.2f}, {pos[1]:.2f}]" for pos in agent_end_list])

    parameters = f"""# 障碍物路径规划参数
# 从UAV分析结果生成的场景

# ==================== 智能体参数 ====================
agent.Num: {len(SELECTED_UAV_IDS)}
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
    - unicycle

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

    return parameters

def generate_uav_pkl_file(uav_data, description):
    """生成agent_list_100.pkl文件（暂不使用，依赖太复杂）"""
    # 暂不实现，使用initialize()函数代替
    pass

# ==================== 主函数 ====================
def main():
    print("="*60)
    print("UAV引导场景生成器")
    print("="*60)

    # 加载UAV数据
    print(f"\n正在加载UAV数据: {UAV_DATA_PATH}")
    uav_data = load_uav_positions()

    print(f"UAV总数: {len(uav_data['uavs'])}")
    uav0_times = uav_data['uavs']['0']['times']
    print(f"时间范围: {uav0_times[0]}s - {uav0_times[-1]}s")
    print(f"时间步长: {uav_data['time_step']}s")

    # 创建输出目录
    if os.path.exists(OUTPUT_DIR):
        shutil.rmtree(OUTPUT_DIR)
    os.makedirs(OUTPUT_DIR)
    os.makedirs(f'{OUTPUT_DIR}/savefig')

    print(f"\n正在生成场景到: {OUTPUT_DIR}")

    # 生成description.json
    description, agent_start_list, agent_end_list = generate_description_json(uav_data)
    description_path = os.path.join(OUTPUT_DIR, 'description.json')
    with open(description_path, 'w') as f:
        json.dump(description, f, indent=2)
    print(f"  - 生成 description.json")
    print(f"    智能体数量: {len(description['agent_start_list'])}")
    print(f"    障碍物数量: {len(description['obstacle_list'])}")

    # 生成parameters.yaml
    parameters = generate_parameters_yaml(agent_start_list, agent_end_list)
    parameters_path = os.path.join(OUTPUT_DIR, 'parameters.yaml')
    with open(parameters_path, 'w') as f:
        f.write(parameters)
    print(f"  - 生成 parameters.yaml")

    # 打印智能体信息
    print("\n智能体信息:")
    for i, uav_id in enumerate(SELECTED_UAV_IDS):
        trajectory = extract_uav_trajectory(uav_data['uavs'][str(uav_id)], uav_id)
        start_scaled = scale_position(trajectory['start'], ANALYSIS_RANGE, MAP_SIZE, MAP_MARGIN)
        end_scaled = scale_position(trajectory['end'], ANALYSIS_RANGE, MAP_SIZE, MAP_MARGIN)
        print(f"  Agent {i} (UAV {uav_id}):")
        print(f"    原始起点: [{trajectory['start'][0]:.2f}, {trajectory['start'][1]:.2f}]")
        print(f"    原始终点: [{trajectory['end'][0]:.2f}, {trajectory['end'][1]:.2f}]")
        print(f"    缩放起点: [{start_scaled[0]:.2f}, {start_scaled[1]:.2f}]")
        print(f"    缩放终点: [{end_scaled[0]:.2f}, {end_scaled[1]:.2f}]")

    print("\n" + "="*60)
    print("场景生成完成！")
    print("="*60)
    print(f"\n运行测试:")
    print(f"  python test.py $(basename {OUTPUT_DIR}) {OUTPUT_DIR}/parameters.yaml not_show")

if __name__ == '__main__':
    main()
