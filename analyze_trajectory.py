import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

# 设置中文字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

# 读取数据
with open('data/01_original_data/uav_positions_over_time.json', 'r', encoding='utf-8') as f:
    uav_data = json.load(f)

with open('data/01_original_data/result_criticalpath_new.json', 'r', encoding='utf-8') as f:
    schedule_data = json.load(f)

with open('data/01_original_data/precomputed_data.json', 'r', encoding='utf-8') as f:
    precomputed = json.load(f)

schedule = schedule_data['task_schedule']

print("="*80)
print("轨迹图详细分析")
print("="*80)

# 分析UAV轨迹特征
print("\n【UAV轨迹分析】")

# 计算每个UAV的移动距离
uav_distances = []
for uav_id, uav_info in uav_data['uavs'].items():
    positions = uav_info['positions']
    distance = 0
    for i in range(1, len(positions)):
        dx = positions[i][0] - positions[i-1][0]
        dy = positions[i][1] - positions[i-1][1]
        distance += np.sqrt(dx**2 + dy**2)
    uav_distances.append({
        'id': int(uav_id),
        'distance': distance,
        'init_pos': positions[0],
        'final_pos': positions[-1]
    })

uav_distances.sort(key=lambda x: x['distance'], reverse=True)

print(f"\nUAV移动距离统计:")
print(f"  平均移动距离: {np.mean([u['distance'] for u in uav_distances]):.2f}")
print(f"  最大移动距离: {uav_distances[0]['distance']:.2f} (UAV {uav_distances[0]['id']})")
print(f"  最小移动距离: {uav_distances[-1]['distance']:.2f} (UAV {uav_distances[-1]['id']})")

print(f"\n移动距离最长的5个UAV:")
for i in range(5):
    u = uav_distances[i]
    print(f"  UAV {u['id']}: {u['distance']:.2f}")
    print(f"    起点: ({u['init_pos'][0]:.2f}, {u['init_pos'][1]:.2f})")
    print(f"    终点: ({u['final_pos'][0]:.2f}, {u['final_pos'][1]:.2f})")

# 分析任务位置分布
print("\n【任务位置分析】")
task_positions = []
for task_id, pos in precomputed['task_positions'].items():
    task_positions.append({
        'id': int(task_id),
        'pos': pos
    })

xs = [t['pos'][0] for t in task_positions]
ys = [t['pos'][1] for t in task_positions]

print(f"\n任务位置分布:")
print(f"  X范围: [{min(xs):.2f}, {max(xs):.2f}]")
print(f"  Y范围: [{min(ys):.2f}, {max(ys):.2f}]")
print(f"  任务中心: ({np.mean(xs):.2f}, {np.mean(ys):.2f})")

# 计算任务间的平均距离
task_distances = []
for i in range(len(task_positions)):
    for j in range(i+1, len(task_positions)):
        p1 = task_positions[i]['pos']
        p2 = task_positions[j]['pos']
        dist = np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        task_distances.append(dist)

print(f"  任务间平均距离: {np.mean(task_distances):.2f}")
print(f"  任务间最小距离: {min(task_distances):.2f}")
print(f"  任务间最大距离: {max(task_distances):.2f}")

# 分析轨迹中的关键转折点
print("\n【轨迹关键转折点分析】")

# 找出移动距离最长的UAV的轨迹转折点
top_uav_id = str(uav_distances[0]['id'])
positions = uav_data['uavs'][top_uav_id]['positions']

# 计算方向变化
direction_changes = []
for i in range(2, len(positions)):
    # 计算前一个方向向量
    v1 = [positions[i-1][0] - positions[i-2][0],
          positions[i-1][1] - positions[i-2][1]]
    # 计算当前方向向量
    v2 = [positions[i][0] - positions[i-1][0],
          positions[i][1] - positions[i-1][1]]

    # 计算方向变化角度
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    norm1 = np.sqrt(v1[0]**2 + v1[1]**2)
    norm2 = np.sqrt(v2[0]**2 + v2[1]**2)

    if norm1 > 0.01 and norm2 > 0.01:
        cos_angle = dot / (norm1 * norm2)
        cos_angle = np.clip(cos_angle, -1, 1)
        angle = np.arccos(cos_angle) * 180 / np.pi

        if angle > 30:  # 方向变化超过30度视为转折
            direction_changes.append({
                'time': i,
                'pos': positions[i],
                'angle': angle
            })

print(f"\nUAV {uav_distances[0]['id']} 轨迹转折点 (方向变化>30°):")
print(f"  总转折次数: {len(direction_changes)}")
if len(direction_changes) > 0:
    print(f"  主要转折点:")
    for i, change in enumerate(direction_changes[:5]):
        print(f"    时间{change['time']}s: 位置({change['pos'][0]:.2f}, {change['pos'][1]:.2f}), "
              f"转向{change['angle']:.1f}°")

# 分析任务执行顺序与位置关系
print("\n【任务执行顺序与空间关系】")

# 按开始时间排序任务
sorted_tasks = sorted(schedule.items(), key=lambda x: x[1]['start'])

print(f"\n任务执行顺序与位置:")
for task_id, times in sorted_tasks[:10]:  # 显示前10个任务
    task_pos = precomputed['task_positions'][task_id]
    print(f"  任务{task_id}: {times['start']:.1f}s-{times['end']:.1f}s, "
          f"位置({task_pos[0]:.2f}, {task_pos[1]:.2f})")

# 分析空间分布密度
print("\n【空间分布密度分析】")

# 将空间划分为网格，计算每个网格的UAV密度
grid_size = 5
x_min, x_max = min(xs), max(xs)
y_min, y_max = min(ys), max(ys)

grid = {}
for task in task_positions:
    grid_x = int((task['pos'][0] - x_min) / grid_size)
    grid_y = int((task['pos'][1] - y_min) / grid_size)
    key = (grid_x, grid_y)
    if key not in grid:
        grid[key] = []
    grid[key].append(task['id'])

print(f"\n任务空间分布 (网格大小={grid_size}):")
for key, tasks in sorted(grid.items(), key=lambda x: len(x[1]), reverse=True)[:5]:
    print(f"  网格{key}: {len(tasks)}个任务 {tasks}")

# 计算任务完成后的UAV最终位置分布
print("\n【UAV最终位置分析】")
final_positions = []
for uav_id, uav_info in uav_data['uavs'].items():
    final_positions.append(uav_info['positions'][-1])

final_xs = [p[0] for p in final_positions]
final_ys = [p[1] for p in final_positions]

print(f"\nUAV最终位置分布:")
print(f"  X范围: [{min(final_xs):.2f}, {max(final_xs):.2f}]")
print(f"  Y范围: [{min(final_ys):.2f}, {max(final_ys):.2f}]")
print(f"  中心点: ({np.mean(final_xs):.2f}, {np.mean(final_ys):.2f})")

# 分析从起点到终点的移动方向
print("\n【UAV移动方向分析】")
move_directions = {
    'right': 0,
    'left': 0,
    'up': 0,
    'down': 0,
    'diagonal': 0
}

for uav in uav_distances:
    dx = uav['final_pos'][0] - uav['init_pos'][0]
    dy = uav['final_pos'][1] - uav['init_pos'][1]

    # 判断主要移动方向
    if abs(dx) > abs(dy):
        if dx > 1:
            move_directions['right'] += 1
        elif dx < -1:
            move_directions['left'] += 1
        else:
            move_directions['diagonal'] += 1
    else:
        if dy > 1:
            move_directions['up'] += 1
        elif dy < -1:
            move_directions['down'] += 1
        else:
            move_directions['diagonal'] += 1

print(f"\nUAV主要移动方向统计:")
for direction, count in move_directions.items():
    print(f"  {direction}: {count}架UAV")

print("\n" + "="*80)
