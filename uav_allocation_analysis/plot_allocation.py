import os
import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
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

print('正在生成UAV任务分配分析图...')

# 创建大型综合图表
fig = plt.figure(figsize=(28, 20))
gs = GridSpec(5, 4, figure=fig, hspace=0.35, wspace=0.3)

# 颜色定义
colors = {
    'A': '#3498db',  # 蓝色
    'B': '#e74c3c'   # 红色
}

# ========== 1. UAV任务数量分布 ==========
ax1 = fig.add_subplot(gs[0, :2])
uav_task_counts = []
uav_ids = []
for uav_id, tasks in sorted(data['uav_schedule'].items(), key=lambda x: int(x[0])):
    uav_ids.append(int(uav_id))
    uav_task_counts.append(len(tasks))

bars = ax1.bar(uav_ids, uav_task_counts, color='#2ecc71', alpha=0.7, edgecolor='black')
ax1.set_xlabel('UAV ID', fontsize=11)
ax1.set_ylabel('任务数量', fontsize=11)
ax1.set_title('(1) 每个UAV分配的任务数量', fontsize=13, fontweight='bold')
ax1.grid(axis='y', alpha=0.3)
ax1.set_xlim(-1, max(uav_ids)+1)

# 添加平均值线
avg_count = np.mean(uav_task_counts)
ax1.axhline(y=avg_count, color='red', linestyle='--', linewidth=2, label=f'平均: {avg_count:.1f}')
ax1.legend()

# ========== 2. 每个任务的UAV分配数量 ==========
ax2 = fig.add_subplot(gs[0, 2:])
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
bars_a = ax2.bar(x - width/2, task_uav_counts['A'], width, label='A阶段', color=colors['A'], alpha=0.7)
bars_b = ax2.bar(x + width/2, task_uav_counts['B'], width, label='B阶段', color=colors['B'], alpha=0.7)
ax2.set_xlabel('任务ID', fontsize=11)
ax2.set_ylabel('UAV数量', fontsize=11)
ax2.set_title('(2) 每个任务分配的UAV数量', fontsize=13, fontweight='bold')
ax2.set_xticks(x)
ax2.set_xticklabels(task_ids, rotation=45, ha='right')
ax2.legend()
ax2.grid(axis='y', alpha=0.3)

# ========== 3. 同时性约束组 ==========
ax3 = fig.add_subplot(gs[1, :2])
sim_groups = env_data['sim_groups']
group_colors = ['#9b59b6', '#f39c12', '#1abc9c', '#e67e22']

for i, group in enumerate(sim_groups):
    ax3.text(0.5, 0.9 - i*0.25, f'组 {i}: Task {group}',
             transform=ax3.transAxes, fontsize=12,
             bbox=dict(boxstyle='round', facecolor=group_colors[i % len(group_colors)], alpha=0.3))

ax3.set_xlim(0, 1)
ax3.set_ylim(0, 1)
ax3.axis('off')
ax3.set_title('(3) 同时性约束 (需同时执行的任务组)', fontsize=13, fontweight='bold', pad=20)

# ========== 4. 任务执行时间统计 ==========
ax4 = fig.add_subplot(gs[1, 2:])
task_durations = []
for tid, times in data['task_schedule'].items():
    duration = times['end'] - times['start']
    task_durations.append((int(tid), duration))

task_durations.sort(key=lambda x: x[0])
task_ids_2 = [t[0] for t in task_durations]
durations = [t[1] for t in task_durations]

ax4.bar(task_ids_2, durations, color='#1abc9c', alpha=0.7, edgecolor='black')
ax4.set_xlabel('任务ID', fontsize=11)
ax4.set_ylabel('执行时间 (s)', fontsize=11)
ax4.set_title('(4) 每个任务的执行时间', fontsize=13, fontweight='bold')
ax4.grid(axis='y', alpha=0.3)
ax4.set_xticklabels(task_ids_2, rotation=45, ha='right')

# ========== 5. UAV任务甘特图 ==========
ax5 = fig.add_subplot(gs[2:4, :])
selected_uavs = sorted([int(k) for k in data['uav_schedule'].keys()])[:20]  # 前20个UAV

y_positions = []
for i, uav_id in enumerate(selected_uavs):
    uav_id_str = str(uav_id)
    tasks = data['uav_schedule'][uav_id_str]

    for task in tasks:
        tid = task['task']
        target = task['target']
        start = task['start']
        end = task['end']

        ax5.barh(i, end - start, left=start, height=0.8,
                color=colors[target], alpha=0.7, edgecolor='black', linewidth=0.5)
        ax5.text(start + (end - start)/2, i, f'{tid}-{target}',
                ha='center', va='center', fontsize=7, color='white', fontweight='bold')

    y_positions.append(i)

ax5.set_yticks(y_positions)
ax5.set_yticklabels([f'UAV {uav}' for uav in selected_uavs])
ax5.set_xlabel('时间 (s)', fontsize=12)
ax5.set_title('(5) UAV任务执行甘特图 (前20个UAV)', fontsize=13, fontweight='bold')
ax5.grid(axis='x', alpha=0.3)

# 添加图例
a_patch = mpatches.Patch(color=colors['A'], label='A阶段')
b_patch = mpatches.Patch(color=colors['B'], label='B阶段')
ax5.legend(handles=[a_patch, b_patch], loc='upper right')

# ========== 6. 统计汇总 ==========
ax6 = fig.add_subplot(gs[4, :])
ax6.axis('off')

stats_text = f"""
统计汇总信息:
  总UAV数量: {len(data['uav_schedule'])}
  总任务数量: {len(data['task_schedule'])}
  Makespan: {data['makespan']:.2f}s
  计算时间: {data['computation_time']:.3f}s
  约束违规: {data['violations']}
  平均每个UAV任务数: {np.mean(uav_task_counts):.1f}
  平均每个任务UAV数 (A阶段): {np.mean(task_uav_counts['A']):.1f}
  平均每个任务UAV数 (B阶段): {np.mean(task_uav_counts['B']):.1f}
"""

ax6.text(0.1, 0.5, stats_text, transform=ax6.transAxes, fontsize=14,
         verticalalignment='center',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# 添加总标题
fig.suptitle('CRITICALPATH算法 - UAV任务分配完整分析', fontsize=18, fontweight='bold', y=0.995)

# 保存图片
output_path = 'uav_allocation_analysis/criticalpath_allocation_analysis.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f'图表已保存到: {os.path.abspath(output_path)}')
plt.close()

# 同时生成一个详细的数据表
with open('uav_allocation_analysis/allocation_details.txt', 'w', encoding='utf-8') as f:
    f.write('='*80 + '\n')
    f.write('CRITICALPATH算法 - UAV任务分配详细信息\n')
    f.write('='*80 + '\n\n')

    f.write(f'Makespan: {data["makespan"]:.2f}s\n')
    f.write(f'计算时间: {data["computation_time"]:.3f}s\n')
    f.write(f'约束违规: {data["violations"]}\n\n')

    f.write('='*80 + '\n')
    f.write('每个UAV分配的任务:\n')
    f.write('='*80 + '\n')

    for uav_id, tasks in sorted(data['uav_schedule'].items(), key=lambda x: int(x[0])):
        task_list = [(t['task'], t['target'], f'{t["start"]:.1f}', f'{t["end"]:.1f}') for t in tasks]
        f.write(f'\nUAV {uav_id} ({len(tasks)}个任务):\n')
        for task_id, target, start, end in task_list:
            f.write(f'  Task {task_id} ({target}阶段): {start}s -> {end}s\n')

    f.write('\n' + '='*80 + '\n')
    f.write('同时性约束:\n')
    f.write('='*80 + '\n')
    for i, group in enumerate(sim_groups):
        f.write(f'组 {i}: Task {group}\n')

print(f'详细数据已保存到: {os.path.abspath("uav_allocation_analysis/allocation_details.txt")}')
print('\n输出路径: ' + os.path.abspath('uav_allocation_analysis'))
