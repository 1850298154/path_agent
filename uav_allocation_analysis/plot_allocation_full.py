import os
import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Patch
from matplotlib.gridspec import GridSpec
import numpy as np

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

# 加载CriticalPath结果
with open('result_criticalpath_new.json', 'r') as f:
    data = json.load(f)

# 加载环境数据
with open('precomputed_data.json', 'r') as f:
    env_data = json.load(f)

task_list = env_data['task_list']

# 创建task_id到task_info的映射
task_info_map = {t['task_id']: t for t in task_list}

print('正在生成UAV任务分配完整分析图...')

# 根据任务类型定义颜色
task_colors = {
    'surveillance': {'A': '#FFD700', 'B': '#FFA500'},  # 金色系
    'attack': {'A': '#FF6B6B', 'B': '#EE5A5A'},       # 红色系
    'capture': {'A': '#4ECDC4', 'B': '#44A08D'}       # 青色系
}

# 创建大型综合图表
fig = plt.figure(figsize=(32, 28))
gs = GridSpec(5, 3, figure=fig, hspace=0.35, wspace=0.3)

# ========== 1. UAV任务数量分布 (所有80个UAV) ==========
ax1 = fig.add_subplot(gs[0, :])
uav_task_counts = []
uav_ids = []
for uav_id, tasks in sorted(data['uav_schedule'].items(), key=lambda x: int(x[0])):
    uav_ids.append(int(uav_id))
    uav_task_counts.append(len(tasks))

bars = ax1.bar(uav_ids, uav_task_counts, color='#2ecc71', alpha=0.7, edgecolor='black')
ax1.set_xlabel('UAV ID', fontsize=12)
ax1.set_ylabel('任务数量', fontsize=12)
ax1.set_title('(1) 每个UAV分配的任务数量 (80个UAV)', fontsize=14, fontweight='bold')
ax1.grid(axis='y', alpha=0.3)
ax1.set_xlim(-1, max(uav_ids)+1)

# 添加平均值线
avg_count = np.mean(uav_task_counts)
ax1.axhline(y=avg_count, color='red', linestyle='--', linewidth=2, label=f'平均: {avg_count:.1f}')
ax1.legend()

# ========== 2. 每个任务的UAV分配数量 ==========
ax2 = fig.add_subplot(gs[1, :2])
task_uav_counts = {'A': [], 'B': []}
task_ids = sorted(set(t['task'] for tasks in data['uav_schedule'].values() for t in tasks))

# 统计每个任务各阶段的UAV数量
for tid in task_ids:
    uavs_a = set()
    uavs_b = set()
    for uav_id, tasks in data['uav_schedule'].items():
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
bars_a = ax2.bar(x - width/2, task_uav_counts['A'], width, label='A阶段', color='#3498db', alpha=0.7)
bars_b = ax2.bar(x + width/2, task_uav_counts['B'], width, label='B阶段', color='#e74c3c', alpha=0.7)
ax2.set_xlabel('任务ID', fontsize=11)
ax2.set_ylabel('UAV数量', fontsize=11)
ax2.set_title('(2) 每个任务分配的UAV数量 (30个任务)', fontsize=14, fontweight='bold')
ax2.set_xticks(x)
ax2.set_xticklabels(task_ids, rotation=45, ha='right')
ax2.legend()
ax2.grid(axis='y', alpha=0.3)

# ========== 3. 同时性约束组 ==========
ax3 = fig.add_subplot(gs[1, 2])
sim_groups = env_data['sim_groups']
group_colors = ['#9b59b6', '#f39c12', '#1abc9c', '#e67e22']

for i, group in enumerate(sim_groups):
    ax3.text(0.5, 0.9 - i*0.25, f'组 {i}: Task {group}',
             transform=ax3.transAxes, fontsize=13,
             bbox=dict(boxstyle='round', facecolor=group_colors[i % len(group_colors)], alpha=0.3))

ax3.set_xlim(0, 1)
ax3.set_ylim(0, 1)
ax3.axis('off')
ax3.set_title('(3) 同时性约束', fontsize=14, fontweight='bold', pad=20)

# ========== 4. 任务执行时间统计 ==========
ax4 = fig.add_subplot(gs[2, :])
task_durations = []
task_types = []
for tid, times in data['task_schedule'].items():
    duration = times['end'] - times['start']
    task_durations.append((int(tid), duration, task_info_map[int(tid)]['type']))

task_durations.sort(key=lambda x: x[0])
task_ids_2 = [t[0] for t in task_durations]
durations = [t[1] for t in task_durations]
task_type_list = [t[2] for t in task_durations]

# 按任务类型着色
type_colors = {'surveillance': '#FFD700', 'attack': '#FF6B6B', 'capture': '#4ECDC4'}
bar_colors = [type_colors[t] for t in task_type_list]

ax4.bar(task_ids_2, durations, color=bar_colors, alpha=0.7, edgecolor='black')
ax4.set_xlabel('任务ID', fontsize=12)
ax4.set_ylabel('执行时间 (s)', fontsize=12)
ax4.set_title('(4) 每个任务的执行时间 (30个任务)', fontsize=14, fontweight='bold')
ax4.grid(axis='y', alpha=0.3)

# 添加图例
sur_patch = Patch(facecolor='#FFD700', label='侦察 (surveillance)')
att_patch = Patch(facecolor='#FF6B6B', label='攻击 (attack)')
cap_patch = Patch(facecolor='#4ECDC4', label='捕获 (capture)')
ax4.legend(handles=[sur_patch, att_patch, cap_patch])

# ========== 5. UAV任务甘特图 (所有80个UAV) ==========
ax5 = fig.add_subplot(gs[3:5, :])

# 绘制所有80个UAV的甘特图
all_uav_ids = sorted([int(k) for k in data['uav_schedule'].keys()])

for i, uav_id in enumerate(all_uav_ids):
    uav_id_str = str(uav_id)
    tasks = data['uav_schedule'][uav_id_str]
    y_pos = len(all_uav_ids) - i - 1  # 反转Y轴，让UAV 0在顶部

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
            ax5.plot([travel_start, travel_end], [y_pos, y_pos],
                    color='gray', linestyle='--', linewidth=1.5, alpha=0.5)

        # 执行时间（实线，按任务类型着色）
        ax5.barh(y_pos, end - start, left=start, height=0.6,
                color=task_colors[task_type][target], alpha=0.8,
                edgecolor='black', linewidth=0.5)

        # 标注任务ID
        ax5.text(start + (end - start)/2, y_pos, f'{tid}-{target}',
                ha='center', va='center', fontsize=6, color='black', fontweight='bold')

ax5.set_yticks(range(len(all_uav_ids)))
ax5.set_yticklabels([f'UAV {uav}' for uav in reversed(all_uav_ids)])
ax5.set_xlabel('时间 (s)', fontsize=13)
ax5.set_title('(5) UAV任务执行甘特图 (80个UAV)', fontsize=14, fontweight='bold')
ax5.grid(axis='x', alpha=0.3)

# 添加图例
legend_elements = [
    Patch(facecolor=task_colors['surveillance']['A'], label='侦察-A'),
    Patch(facecolor=task_colors['surveillance']['B'], label='侦察-B'),
    Patch(facecolor=task_colors['attack']['A'], label='攻击-A'),
    Patch(facecolor=task_colors['attack']['B'], label='攻击-B'),
    Patch(facecolor=task_colors['capture']['A'], label='捕获-A'),
    Patch(facecolor=task_colors['capture']['B'], label='捕获-B'),
]
ax5.legend(handles=legend_elements, loc='upper right', ncol=2, fontsize=10)

# 添加总标题
fig.suptitle('CRITICALPATH算法 - UAV任务分配完整分析 (80个UAV, 30个任务)',
             fontsize=18, fontweight='bold', y=0.995)

# 保存图片
output_path = 'uav_allocation_analysis/criticalpath_allocation_full.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f'图表已保存到: {os.path.abspath(output_path)}')
plt.close()

# 生成完整的数据表
with open('uav_allocation_analysis/allocation_details_full.txt', 'w', encoding='utf-8') as f:
    f.write('='*80 + '\n')
    f.write('CRITICALPATH算法 - UAV任务分配详细信息 (完整版)\n')
    f.write('='*80 + '\n\n')

    f.write(f'Makespan: {data["makespan"]:.2f}s\n')
    f.write(f'计算时间: {data["computation_time"]:.3f}s\n')
    f.write(f'约束违规: {data["violations"]}\n')
    f.write(f'总UAV数量: {len(data["uav_schedule"])}\n')
    f.write(f'总任务数量: {len(data["task_schedule"])}\n\n')

    f.write('='*80 + '\n')
    f.write('每个UAV分配的任务 (80个UAV):\n')
    f.write('='*80 + '\n')

    for uav_id, tasks in sorted(data['uav_schedule'].items(), key=lambda x: int(x[0])):
        task_list_uav = [(t['task'], t['target'], task_list[t['task']]['type'],
                        f'{t["start"]:.1f}', f'{t["end"]:.1f}') for t in tasks]
        f.write(f'\nUAV {uav_id} ({len(tasks)}个任务):\n')
        for task_id, target, task_type, start, end in task_list_uav:
            f.write(f'  Task {task_id} [{task_type}] ({target}阶段): {start}s -> {end}s\n')

    f.write('\n' + '='*80 + '\n')
    f.write('同时性约束:\n')
    f.write('='*80 + '\n')
    for i, group in enumerate(sim_groups):
        f.write(f'组 {i}: Task {group}\n')

    f.write('\n' + '='*80 + '\n')
    f.write('每个任务分配的UAV统计:\n')
    f.write('='*80 + '\n')
    for tid in task_ids:
        uavs_a = sorted(set([int(uav_id) for uav_id, tasks in data['uav_schedule'].items()
                           for t in tasks if t['task'] == tid and t['target'] == 'A']))
        uavs_b = sorted(set([int(uav_id) for uav_id, tasks in data['uav_schedule'].items()
                           for t in tasks if t['task'] == tid and t['target'] == 'B']))
        task_type = task_list[tid]['type']
        f.write(f'\nTask {tid} [{task_type}]:\n')
        f.write(f'  A阶段: {len(uavs_a)}个UAV -> {uavs_a}\n')
        f.write(f'  B阶段: {len(uavs_b)}个UAV -> {uavs_b}\n')

print(f'详细数据已保存到: {os.path.abspath("uav_allocation_analysis/allocation_details_full.txt")}')
print('\n输出路径: ' + os.path.abspath('uav_allocation_analysis'))
