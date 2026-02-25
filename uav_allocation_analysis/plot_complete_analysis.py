import os
import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Patch, Circle, Rectangle
from matplotlib.gridspec import GridSpec
import numpy as np

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

print('正在生成完整分析图（包含位置和轨迹）...')

# 加载数据
with open('result_criticalpath_new.json', 'r') as f:
    result_data = json.load(f)

with open('precomputed_data.json', 'r') as f:
    env_data = json.load(f)

task_list = env_data['task_list']
uav_list = env_data['uav_list']

# 创建task_id到task_info的映射
task_info_map = {t['task_id']: t for t in task_list}
uav_info_map = {t['uav_id']: t for t in uav_list}

# 定义颜色
task_colors = {
    'surveillance': {'A': '#FFD700', 'B': '#FFA500'},  # 金色系
    'attack': {'A': '#FF6B6B', 'B': '#EE5A5A'},       # 红色系
    'capture': {'A': '#4ECDC4', 'B': '#44A08D'}       # 青色系
}

base_colors = {'base1': '#3498db', 'base2': '#9b59b6'}

# 创建大型综合图表
fig = plt.figure(figsize=(36, 32))
gs = GridSpec(6, 4, figure=fig, hspace=0.35, wspace=0.3)

# ========== 1. 任务位置分布图 ==========
ax1 = fig.add_subplot(gs[0, :2])
ax1.set_title('(1) 任务位置分布图', fontsize=14, fontweight='bold')
ax1.set_xlabel('X坐标', fontsize=11)
ax1.set_ylabel('Y坐标', fontsize=11)
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal')

# 绘制基地
for base_name in ['base1', 'base2']:
    base_pos = next((u['init_pos'] for u in uav_list if u['base'] == base_name), None)
    if base_pos:
        base_rect = Rectangle((base_pos[0]-3.2/2, base_pos[1]-3.2/2), 3.2, 3.2,
                           facecolor=base_colors[base_name], alpha=0.2, edgecolor=base_colors[base_name])
        ax1.add_patch(base_rect)
        ax1.text(base_pos[0], base_pos[1], base_name, ha='center', va='center',
                fontsize=10, fontweight='bold', color=base_colors[base_name])

# 绘制任务
for task in task_list:
    task_id = task['task_id']
    task_type = task['type']
    center = task['center']
    radius = task['radius']

    color = {'surveillance': '#FFD700', 'attack': '#FF6B6B', 'capture': '#4ECDC4'}[task_type]
    circle = Circle(center, radius, facecolor=color, alpha=0.4, edgecolor=color, linewidth=1.5)
    ax1.add_patch(circle)
    ax1.text(center[0], center[1], str(task_id), ha='center', va='center',
            fontsize=9, fontweight='bold', color='black')

surv_patch = Patch(facecolor='#FFD700', alpha=0.4, label='侦察')
att_patch = Patch(facecolor='#FF6B6B', alpha=0.4, label='攻击')
cap_patch = Patch(facecolor='#4ECDC4', alpha=0.4, label='捕获')
base1_patch = Patch(facecolor='#3498db', alpha=0.2, label='基地1')
base2_patch = Patch(facecolor='#9b59b6', alpha=0.2, label='基地2')
ax1.legend(handles=[surv_patch, att_patch, cap_patch, base1_patch, base2_patch], loc='upper left')

# ========== 2. UAV初始位置分布 ==========
ax2 = fig.add_subplot(gs[0, 2:])
ax2.set_title('(2) UAV初始位置分布 (80个UAV)', fontsize=14, fontweight='bold')
ax2.set_xlabel('X坐标', fontsize=11)
ax2.set_ylabel('Y坐标', fontsize=11)
ax2.grid(True, alpha=0.3)
ax2.set_aspect('equal')

# 绘制基地
for base_name in ['base1', 'base2']:
    base_pos = next((u['init_pos'] for u in uav_list if u['base'] == base_name), None)
    if base_pos:
        base_rect = Rectangle((base_pos[0]-3.2/2, base_pos[1]-3.2/2), 3.2, 3.2,
                           facecolor=base_colors[base_name], alpha=0.2, edgecolor=base_colors[base_name])
        ax2.add_patch(base_rect)

# 按类型绘制UAV
type_colors = {'uavA': '#e74c3c', 'uavB': '#3498db', 'uavC': '#2ecc71'}
for uav in uav_list:
    ax2.scatter(uav['init_pos'][0], uav['init_pos'][1],
               c=type_colors[uav['type']], s=30, alpha=0.6, edgecolor='black')

uav_a_patch2 = Patch(facecolor='#e74c3c', label='UAV A')
uav_b_patch2 = Patch(facecolor='#3498db', label='UAV B')
uav_c_patch2 = Patch(facecolor='#2ecc71', label='UAV C')
ax2.legend(handles=[uav_a_patch2, uav_b_patch2, uav_c_patch2], loc='upper right')

# ========== 3. UAV任务数量分布 ==========
ax3 = fig.add_subplot(gs[1, :])
uav_task_counts = []
uav_ids = []
for uav_id, tasks in sorted(result_data['uav_schedule'].items(), key=lambda x: int(x[0])):
    uav_ids.append(int(uav_id))
    uav_task_counts.append(len(tasks))

ax3.bar(uav_ids, uav_task_counts, color='#2ecc71', alpha=0.7, edgecolor='black')
ax3.set_xlabel('UAV ID', fontsize=11)
ax3.set_ylabel('任务数量', fontsize=11)
ax3.set_title('(3) 每个UAV分配的任务数量 (80个UAV)', fontsize=14, fontweight='bold')
ax3.grid(axis='y', alpha=0.3)
ax3.set_xlim(-1, max(uav_ids)+1)

avg_count = np.mean(uav_task_counts)
ax3.axhline(y=avg_count, color='red', linestyle='--', linewidth=2, label=f'平均: {avg_count:.1f}')
ax3.legend()

# ========== 4. 每个任务的UAV分配数量 ==========
ax4 = fig.add_subplot(gs[2, :2])
task_uav_counts = {'A': [], 'B': []}
task_ids = sorted(set(t['task'] for tasks in result_data['uav_schedule'].values() for t in tasks))

for tid in task_ids:
    uavs_a = set()
    uavs_b = set()
    for uav_id, tasks in result_data['uav_schedule'].items():
        for task in tasks:
            if task['task'] == tid:
                if task['target'] == 'A':
                    uavs_a.add(int(uav_id))
                else:
                    uavs_b.add(int(uav_id))
    task_uav_counts['A'].append(len(uavs_a))
    task_uav_counts['B'].append(len(uavs_b))

x = np.arange(len(task_ids))
width = 0.35
ax4.bar(x - width/2, task_uav_counts['A'], width, label='A阶段', color='#3498db', alpha=0.7)
ax4.bar(x + width/2, task_uav_counts['B'], width, label='B阶段', color='#e74c3c', alpha=0.7)
ax4.set_xlabel('任务ID', fontsize=11)
ax4.set_ylabel('UAV数量', fontsize=11)
ax4.set_title('(4) 每个任务分配的UAV数量 (30个任务)', fontsize=13, fontweight='bold')
ax4.set_xticks(x)
ax4.set_xticklabels(task_ids, rotation=45, ha='right')
ax4.legend()
ax4.grid(axis='y', alpha=0.3)

# ========== 5. 同时性约束组 ==========
ax5 = fig.add_subplot(gs[2, 2:])
sim_groups = env_data['sim_groups']
group_colors_list = ['#9b59b6', '#f39c12', '#1abc9c', '#e67e22']

for i, group in enumerate(sim_groups):
    ax5.text(0.5, 0.9 - i*0.25, f'组 {i}: Task {group}',
             transform=ax5.transAxes, fontsize=13,
             bbox=dict(boxstyle='round', facecolor=group_colors_list[i % len(group_colors_list)], alpha=0.3))

ax5.set_xlim(0, 1)
ax5.set_ylim(0, 1)
ax5.axis('off')
ax5.set_title('(5) 同时性约束', fontsize=13, fontweight='bold', pad=20)

# ========== 6. 任务执行时间统计 ==========
ax6 = fig.add_subplot(gs[3, :])
task_durations = []
for tid, times in result_data['task_schedule'].items():
    duration = times['end'] - times['start']
    task_durations.append((int(tid), duration, task_info_map[int(tid)]['type']))

task_durations.sort(key=lambda x: x[0])
task_ids_2 = [t[0] for t in task_durations]
durations = [t[1] for t in task_durations]
task_type_list = [t[2] for t in task_durations]

type_colors_dict = {'surveillance': '#FFD700', 'attack': '#FF6B6B', 'capture': '#4ECDC4'}
bar_colors = [type_colors_dict[t] for t in task_type_list]

ax6.bar(task_ids_2, durations, color=bar_colors, alpha=0.7, edgecolor='black')
ax6.set_xlabel('任务ID', fontsize=11)
ax6.set_ylabel('执行时间 (s)', fontsize=11)
ax6.set_title('(6) 每个任务的执行时间 (30个任务)', fontsize=13, fontweight='bold')
ax6.grid(axis='y', alpha=0.3)

sur_patch = Patch(facecolor='#FFD700', label='侦察 (surveillance)')
att_patch = Patch(facecolor='#FF6B6B', label='攻击 (attack)')
cap_patch = Patch(facecolor='#4ECDC4', label='捕获 (capture)')
ax6.legend(handles=[sur_patch, att_patch, cap_patch])

# ========== 7. UAV轨迹图 (前20个UAV) ==========
ax7 = fig.add_subplot(gs[4:6, :2])
ax7.set_title('(7) UAV移动轨迹 (前20个UAV)', fontsize=14, fontweight='bold')
ax7.set_xlabel('X坐标', fontsize=11)
ax7.set_ylabel('Y坐标', fontsize=11)
ax7.grid(True, alpha=0.3)
ax7.set_aspect('equal')

# 绘制基地和任务
for base_name in ['base1', 'base2']:
    base_pos = next((u['init_pos'] for u in uav_list if u['base'] == base_name), None)
    if base_pos:
        base_rect = Rectangle((base_pos[0]-3.2/2, base_pos[1]-3.2/2), 3.2, 3.2,
                           facecolor=base_colors[base_name], alpha=0.2, edgecolor=base_colors[base_name])
        ax7.add_patch(base_rect)
        ax7.text(base_pos[0], base_pos[1], base_name, ha='center', va='center',
                fontsize=9, fontweight='bold', color=base_colors[base_name])

for task in task_list:
    circle = Circle(task['center'], task['radius'],
                   facecolor='lightgray', alpha=0.2, edgecolor='gray')
    ax7.add_patch(circle)
    ax7.text(task['center'][0], task['center'][1], str(task['task_id']),
            ha='center', va='center', fontsize=6)

# 绘制UAV轨迹（前20个）
selected_uavs = sorted([int(k) for k in result_data['uav_schedule'].keys()])[:20]
for uav_id in selected_uavs:
    uav_id_str = str(uav_id)
    uav_info = uav_info_map[uav_id]
    tasks = result_data['uav_schedule'][uav_id_str]

    current_pos = uav_info['init_pos']
    trajectory_x = [current_pos[0]]
    trajectory_y = [current_pos[1]]

    for task in tasks:
        tid = task['task']
        task_center = task_info_map[tid]['center']

        # 绘制从当前位置到任务的线
        ax7.plot([current_pos[0], task_center[0]], [current_pos[1], task_center[1]],
                '--', alpha=0.3, linewidth=1, color=type_colors[uav_info['type']])

        trajectory_x.extend([current_pos[0], task_center[0]])
        trajectory_y.extend([current_pos[1], task_center[1]])

        current_pos = task_center

    # 绘制完整轨迹
    ax7.plot(trajectory_x, trajectory_y, '-', alpha=0.5, linewidth=1,
             color=type_colors[uav_info['type']], label=f'UAV {uav_id}' if uav_id == selected_uavs[0] else "")

# 添加图例（只显示一个）
uav_a_patch = Patch(facecolor='#e74c3c', label='UAV A')
uav_b_patch = Patch(facecolor='#3498db', label='UAV B')
uav_c_patch = Patch(facecolor='#2ecc71', label='UAV C')
from matplotlib.lines import Line2D
traj_line = Line2D([0], [0], color='gray', linestyle='--', label='移动轨迹')
ax7.legend(handles=[uav_a_patch, uav_b_patch, uav_c_patch, traj_line], loc='upper right')

# ========== 8. UAV任务甘特图 (所有80个UAV) ==========
ax8 = fig.add_subplot(gs[4:6, 2:])
ax8.set_title('(8) UAV任务执行甘特图 (80个UAV)', fontsize=14, fontweight='bold')
ax8.set_xlabel('时间 (s)', fontsize=11)

all_uav_ids = sorted([int(k) for k in result_data['uav_schedule'].keys()])

for i, uav_id in enumerate(all_uav_ids):
    uav_id_str = str(uav_id)
    tasks = result_data['uav_schedule'][uav_id_str]
    y_pos = len(all_uav_ids) - i - 1

    for task in tasks:
        tid = task['task']
        target = task['target']
        start = task['start']
        end = task['end']
        travel_start = task['travel_start']
        travel_end = task['travel_end']
        task_type = task_info_map[tid]['type']

        # 旅行时间（虚线）
        if travel_end > travel_start:
            ax8.plot([travel_start, travel_end], [y_pos, y_pos],
                    color='gray', linestyle='--', linewidth=1, alpha=0.3)

        # 执行时间
        ax8.barh(y_pos, end - start, left=start, height=0.5,
                color=task_colors[task_type][target], alpha=0.8, edgecolor='black', linewidth=0.3)

ax8.set_yticks(range(len(all_uav_ids)))
ax8.set_yticklabels([f'UAV {uav}' for uav in reversed(all_uav_ids)], fontsize=7)
ax8.grid(axis='x', alpha=0.3)

legend_elements = [
    Patch(facecolor=task_colors['surveillance']['A'], label='侦察-A'),
    Patch(facecolor=task_colors['surveillance']['B'], label='侦察-B'),
    Patch(facecolor=task_colors['attack']['A'], label='攻击-A'),
    Patch(facecolor=task_colors['attack']['B'], label='攻击-B'),
    Patch(facecolor=task_colors['capture']['A'], label='捕获-A'),
    Patch(facecolor=task_colors['capture']['B'], label='捕获-B'),
]
ax8.legend(handles=legend_elements, loc='upper right', ncol=2, fontsize=8)

# 添加总标题
fig.suptitle('CRITICALPATH算法 - 完整任务分配分析 (80个UAV, 30个任务)',
             fontsize=18, fontweight='bold', y=0.995)

# 保存图片
output_path = 'criticalpath_complete_analysis.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f'完整分析图已保存到: {os.path.abspath(output_path)}')
plt.close()

# 生成详细数据报告
with open('complete_analysis_report.txt', 'w', encoding='utf-8') as f:
    f.write('='*80 + '\n')
    f.write('CRITICALPATH算法 - 完整分析报告\n')
    f.write('='*80 + '\n\n')

    f.write('=== 核心指标 ===\n')
    f.write(f'Makespan: {result_data["makespan"]:.2f}s\n')
    f.write(f'计算时间: {result_data["computation_time"]:.3f}s\n')
    f.write(f'约束违规: {result_data["violations"]}\n')
    f.write(f'总UAV数量: {len(result_data["uav_schedule"])}\n')
    f.write(f'总任务数量: {len(result_data["task_schedule"])}\n\n')

    f.write('=== 任务位置信息 ===\n')
    for task in task_list:
        f.write(f'Task {task["task_id"]} [{task["type"]}]: 位置{task["center"]}, 半径{task["radius"]:.2f}\n')

    f.write('\n=== UAV初始位置信息 ===\n')
    bases = {}
    for uav in uav_list:
        base = uav['base']
        if base not in bases:
            bases[base] = []
        bases[base].append(f'UAV {uav["uav_id"]} ({uav["type"]})')

    for base, uavs in bases.items():
        f.write(f'{base}: {len(uavs)}个UAV\n')

print(f'详细报告已保存到: {os.path.abspath("complete_analysis_report.txt")}')
print('\n输出路径: ' + os.path.abspath('.'))
