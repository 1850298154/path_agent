import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import json
import numpy as np
from matplotlib.patches import FancyBboxPatch
import matplotlib

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

# 时间点
time_points = [0, 440, 668, 764, 830, 885]
time_labels = ['(a) t=0s', '(b) t=440s', '(c) t=668s', '(d) t=764s', '(e) t=830s', '(f) t=885s']

# 创建图表
fig = plt.figure(figsize=(20, 14))

# 创建子图布局
gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)

# 为每个时间点创建快照
for idx, (t, label) in enumerate(zip(time_points, time_labels)):
    # 确定子图位置
    if idx < 2:
        ax = fig.add_subplot(gs[0, idx*2:(idx+1)*2])
    elif idx < 4:
        ax = fig.add_subplot(gs[1, (idx-2)*2:(idx-1)*2])
    else:
        ax = fig.add_subplot(gs[2, (idx-4)*2:(idx-3)*2])

    # 收集UAV位置
    positions_at_t = []
    moving_uavs = []
    static_uavs = []

    for uav_id, uav_info in uav_data['uavs'].items():
        pos_idx = min(int(t), len(uav_info['positions'])-1)
        pos = uav_info['positions'][pos_idx]

        is_moving = False
        if pos_idx > 0:
            prev_pos = uav_info['positions'][pos_idx-1]
            is_moving = (abs(pos[0]-prev_pos[0]) > 0.01 or abs(pos[1]-prev_pos[1]) > 0.01)

        uav_status = {
            'id': int(uav_id),
            'pos': pos,
            'moving': is_moving
        }
        positions_at_t.append(uav_status)

        if is_moving:
            moving_uavs.append(uav_status)
        else:
            static_uavs.append(uav_status)

    # 收集任务信息
    active_tasks = []
    completed_tasks = []
    pending_tasks = []

    for task_id, times in schedule.items():
        if times['start'] <= t <= times['end']:
            task_pos = precomputed['task_positions'].get(str(task_id), [0, 0])
            active_tasks.append({
                'id': int(task_id),
                'pos': task_pos,
                'progress': (t - times['start']) / (times['end'] - times['start']) * 100
            })
        elif times['end'] < t:
            completed_tasks.append(int(task_id))
        else:
            pending_tasks.append(int(task_id))

    # 绘制UAV位置
    if static_uavs:
        xs = [u['pos'][0] for u in static_uavs]
        ys = [u['pos'][1] for u in static_uavs]
        ax.scatter(xs, ys, c='lightblue', s=20, alpha=0.6, label='静止UAV', marker='o')

    if moving_uavs:
        xs = [u['pos'][0] for u in moving_uavs]
        ys = [u['pos'][1] for u in moving_uavs]
        ax.scatter(xs, ys, c='blue', s=30, alpha=0.8, label='移动UAV', marker='^')

    # 绘制正在执行的任务
    if active_tasks:
        for task in active_tasks:
            task_pos = task['pos']
            ax.scatter([task_pos[0]], [task_pos[1]], c='red', s=200,
                      marker='s', edgecolors='darkred', linewidths=2, alpha=0.8)
            ax.annotate(f"T{task['id']}\n{task['progress']:.0f}%",
                       (task_pos[0], task_pos[1]),
                       textcoords="offset points",
                       xytext=(10, 10),
                       fontsize=8,
                       color='darkred',
                       weight='bold')

    # 设置标题和标签
    completed_count = len(completed_tasks)
    active_count = len(active_tasks)
    pending_count = len(pending_tasks)

    title = f"{label}\n完成:{completed_count} 执行:{active_count} 待执行:{pending_count}"
    ax.set_title(title, fontsize=12, weight='bold')

    ax.set_xlim(-5, 35)
    ax.set_ylim(-5, 30)
    ax.set_xlabel('X坐标', fontsize=10)
    ax.set_ylabel('Y坐标', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)

    # 添加进度条
    progress_bar_ax = fig.add_axes([ax.get_position().x0,
                                     ax.get_position().y0 - 0.02,
                                     ax.get_position().width,
                                     0.01])
    total_tasks = len(schedule)
    completed_ratio = completed_count / total_tasks
    active_ratio = active_count / total_tasks

    progress_bar_ax.barh(0, completed_ratio, color='green', alpha=0.7, label='已完成')
    progress_bar_ax.barh(0, active_ratio, left=completed_ratio, color='red', alpha=0.7, label='执行中')
    progress_bar_ax.set_xlim(0, 1)
    progress_bar_ax.set_ylim(0, 1)
    progress_bar_ax.axis('off')

# 添加总标题
fig.suptitle('UAV集群任务执行多时刻运行快照', fontsize=16, weight='bold', y=0.98)

# 添加图例说明
legend_elements = [
    mpatches.Patch(color='lightblue', alpha=0.6, label='静止UAV'),
    mpatches.Patch(color='blue', alpha=0.8, label='移动UAV'),
    mpatches.Patch(color='red', alpha=0.8, label='正在执行的任务'),
    mpatches.Patch(color='green', alpha=0.7, label='已完成任务'),
]
fig.legend(handles=legend_elements, loc='upper center', ncol=4,
           bbox_to_anchor=(0.5, 0.95), fontsize=10)

# 保存图表
plt.savefig('ob_2d/004/2026-03-02_06-40-04/savefig/multi_moment_snapshots.jpg',
            dpi=300, bbox_inches='tight')
plt.close()

print("可视化图表已生成: ob_2d/004/2026-03-02_06-40-04/savefig/multi_moment_snapshots.jpg")

# 生成任务甘特图
fig, ax = plt.subplots(figsize=(16, 10))

# 按开始时间排序任务
sorted_tasks = sorted(schedule.items(), key=lambda x: x[1]['start'])

# 为每个任务绘制甘特图条
for i, (task_id, times) in enumerate(sorted_tasks):
    task_id_int = int(task_id)
    start = times['start']
    duration = times['end'] - times['start']

    # 根据任务ID分配颜色
    color = plt.cm.tab20(task_id_int / len(schedule))

    ax.barh(i, duration, left=start, height=0.6,
            color=color, alpha=0.8, edgecolor='black', linewidth=0.5)

    # 添加任务ID标签
    ax.text(start + duration/2, i, f'T{task_id_int}',
            ha='center', va='center', fontsize=8, weight='bold')

    # 标记关键时间点
    for t in time_points:
        if times['start'] <= t <= times['end']:
            ax.axvline(t, color='red', linestyle='--', alpha=0.5, linewidth=1)

# 设置坐标轴
ax.set_yticks(range(len(sorted_tasks)))
ax.set_yticklabels([f'Task {i}' for i in range(len(sorted_tasks))])
ax.set_xlabel('时间 (秒)', fontsize=12)
ax.set_ylabel('任务ID', fontsize=12)
ax.set_title('任务执行时间线（甘特图）\n红色虚线标记关键时间点', fontsize=14, weight='bold')
ax.grid(True, axis='x', alpha=0.3)

# 添加时间点标注
for t in time_points:
    ax.axvline(t, color='red', linestyle='--', alpha=0.7, linewidth=2)
    ax.text(t, len(sorted_tasks), f't={t}s',
            ha='center', va='bottom', fontsize=10, color='red', weight='bold')

plt.tight_layout()
plt.savefig('ob_2d/004/2026-03-02_06-40-04/savefig/task_gantt_chart.jpg',
            dpi=300, bbox_inches='tight')
plt.close()

print("甘特图已生成: ob_2d/004/2026-03-02_06-40-04/savefig/task_gantt_chart.jpg")

# 生成统计图表
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# 1. 任务完成进度
ax1 = axes[0, 0]
completed_progress = []
active_progress = []
pending_progress = []

for t in time_points:
    completed = 0
    active = 0
    pending = 0

    for task_id, times in schedule.items():
        if times['end'] < t:
            completed += 1
        elif times['start'] <= t <= times['end']:
            active += 1
        else:
            pending += 1

    completed_progress.append(completed)
    active_progress.append(active)
    pending_progress.append(pending)

x = np.arange(len(time_points))
width = 0.25

ax1.bar(x - width, completed_progress, width, label='已完成', color='green', alpha=0.7)
ax1.bar(x, active_progress, width, label='正在执行', color='red', alpha=0.7)
ax1.bar(x + width, pending_progress, width, label='待执行', color='gray', alpha=0.7)

ax1.set_xlabel('时间点', fontsize=11)
ax1.set_ylabel('任务数量', fontsize=11)
ax1.set_title('任务状态分布', fontsize=12, weight='bold')
ax1.set_xticks(x)
ax1.set_xticklabels([f't={t}s' for t in time_points])
ax1.legend()
ax1.grid(True, alpha=0.3)

# 2. UAV移动率变化
ax2 = axes[0, 1]
moving_ratios = []

for t in time_points:
    moving_count = 0
    for uav_id, uav_info in uav_data['uavs'].items():
        pos_idx = min(int(t), len(uav_info['positions'])-1)
        if pos_idx > 0:
            prev_pos = uav_info['positions'][pos_idx-1]
            curr_pos = uav_info['positions'][pos_idx]
            if abs(curr_pos[0]-prev_pos[0]) > 0.01 or abs(curr_pos[1]-prev_pos[1]) > 0.01:
                moving_count += 1

    moving_ratios.append(moving_count / len(uav_data['uavs']) * 100)

ax2.plot(time_points, moving_ratios, 'b-o', linewidth=2, markersize=8)
ax2.fill_between(time_points, moving_ratios, alpha=0.3)
ax2.set_xlabel('时间 (秒)', fontsize=11)
ax2.set_ylabel('UAV移动率 (%)', fontsize=11)
ax2.set_title('UAV移动率变化趋势', fontsize=12, weight='bold')
ax2.grid(True, alpha=0.3)

# 3. 战术密度变化
ax3 = axes[1, 0]
densities = []

for t in time_points:
    positions = []
    for uav_id, uav_info in uav_data['uavs'].items():
        pos_idx = min(int(t), len(uav_info['positions'])-1)
        pos = uav_info['positions'][pos_idx]
        positions.append(pos)

    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]

    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    area = max(x_range * y_range, 0.01)

    density = len(positions) / area
    densities.append(density)

ax3.semilogy(time_points, densities, 'r-s', linewidth=2, markersize=8)
ax3.set_xlabel('时间 (秒)', fontsize=11)
ax3.set_ylabel('战术密度 (UAV/单位面积)', fontsize=11)
ax3.set_title('战术密度变化趋势（对数尺度）', fontsize=12, weight='bold')
ax3.grid(True, alpha=0.3)

# 4. 集群中心移动轨迹
ax4 = axes[1, 1]
centers_x = []
centers_y = []

for t in time_points:
    positions = []
    for uav_id, uav_info in uav_data['uavs'].items():
        pos_idx = min(int(t), len(uav_info['positions'])-1)
        pos = uav_info['positions'][pos_idx]
        positions.append(pos)

    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]

    centers_x.append(sum(xs) / len(xs))
    centers_y.append(sum(ys) / len(ys))

ax4.plot(centers_x, centers_y, 'g-^', linewidth=2, markersize=10)
for i, (x, y) in enumerate(zip(centers_x, centers_y)):
    ax4.annotate(f't={time_points[i]}s', (x, y),
                textcoords="offset points",
                xytext=(10, 5), fontsize=9)

ax4.set_xlabel('X坐标', fontsize=11)
ax4.set_ylabel('Y坐标', fontsize=11)
ax4.set_title('集群中心移动轨迹', fontsize=12, weight='bold')
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('ob_2d/004/2026-03-02_06-40-04/savefig/statistics_charts.jpg',
            dpi=300, bbox_inches='tight')
plt.close()

print("统计图表已生成: ob_2d/004/2026-03-02_06-40-04/savefig/statistics_charts.jpg")
