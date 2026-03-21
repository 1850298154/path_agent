#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
综合可视化 - 结合三种绘图方式的优点
1. 半径使用实际任务半径 (from uav_arrival_monitor.py)
2. 任务颜色与 criticalpath_allocation_full.png 匹配
3. 80个UAV彩虹色轨迹线 (from plot_uav_trajectory.py)
"""
import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import hsv_to_rgb
from matplotlib.patches import Patch

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


def generate_rainbow_colors(n):
    """生成彩虹渐变颜色 - 与 plot_uav_trajectory.py 一致"""
    colors = []
    for i in range(n):
        hue = i / n
        rgb = hsv_to_rgb((hue, 1.0, 0.8))
        colors.append(rgb)
    return colors


def get_task_type_color(task_type, target='A'):
    """
    获取任务颜色 - 与 criticalpath_allocation_full.py 匹配
    surveillance: 金色系 (A: #FFD700, B: #FFA500)
    attack: 红色系 (A: #FF6B6B, B: #EE5A5A)
    capture: 青色系 (A: #4ECDC4, B: #44A08D)
    """
    task_colors = {
        'surveillance': {'A': '#FFD700', 'B': '#FFA500'},
        'attack': {'A': '#FF6B6B', 'B': '#EE5A5A'},
        'capture': {'A': '#4ECDC4', 'B': '#44A08D'}
    }
    return task_colors.get(task_type, {}).get(target, '#888888')


def load_all_data():
    """加载所有必要数据"""
    # 加载UAV位置数据
    with open('uav_positions_over_time.json', 'r', encoding='utf-8') as f:
        uav_data = json.load(f)

    # 加载任务数据 (包含实际半径)
    with open('precomputed_data.json', 'r', encoding='utf-8') as f:
        task_data = json.load(f)

    # 加载调度结果 (包含target A/B阶段信息)
    with open('result_criticalpath_new.json', 'r', encoding='utf-8') as f:
        schedule_data = json.load(f)

    return uav_data, task_data, schedule_data


def plot_combined_visualization():
    """绘制综合可视化图"""
    uav_data, task_data, schedule_data = load_all_data()

    # 创建任务ID到任务信息的映射
    task_map = {task['task_id']: task for task in task_data['task_list']}

    # 生成UAV彩虹色
    n_uavs = len(uav_data['uavs'])
    uav_colors = generate_rainbow_colors(n_uavs)

    fig, ax = plt.subplots(figsize=(20, 20))

    # ========== 1. 绘制任务区域 (底层) ==========
    # 使用实际半径，颜色与 criticalpath_allocation_full.png 匹配
    for task in task_data['task_list']:
        task_id = task['task_id']
        task_type = task['type']
        task_center = task['center']
        task_radius = task['radius']  # 使用实际半径

        # 获取该任务的颜色（默认使用A阶段颜色作为基础色）
        base_color = get_task_type_color(task_type, 'A')

        # 绘制任务圆圈
        circle = patches.Circle(
            (task_center[0], task_center[1]),
            task_radius,
            facecolor=base_color,
            edgecolor='black',
            linewidth=1.5,
            alpha=0.4,
            label=f'Task {task_id}'
        )
        ax.add_patch(circle)

        # 添加任务标签
        ax.text(task_center[0], task_center[1],
                f"{task_id}", fontsize=10,
                ha='center', va='center', weight='bold',
                color='black')

    # ========== 2. 绘制UAV轨迹线 (中层) ==========
    # 80个UAV使用彩虹色，与 uav_trajectories_full.png 保持一致
    for i, (uav_id, uav_info) in enumerate(sorted(uav_data['uavs'].items(), key=lambda x: int(x[0]))):
        init_pos = uav_info['init_pos']
        positions = uav_info['positions']

        # 添加初始位置作为起点
        full_positions = [init_pos] + positions

        if len(full_positions) < 2:
            continue

        pos_array = np.array(full_positions)

        # 绘制轨迹线 (彩虹色)
        ax.plot(pos_array[:, 0], pos_array[:, 1],
                color=uav_colors[i], linewidth=0.8, alpha=0.7)

        # 绘制起点 (小圆点)
        ax.plot(full_positions[0][0], full_positions[0][1], 'o',
                color=uav_colors[i], markersize=4, alpha=0.9, zorder=3)

        # 绘制终点 (小方块)
        ax.plot(full_positions[-1][0], full_positions[-1][1], 's',
                color=uav_colors[i], markersize=5, alpha=1.0, zorder=3)

    # ========== 3. 绘制UAV到达位置标记 (上层) ==========
    for uav_id in sorted(uav_data['uavs'].keys(), key=int):
        uav_info = uav_data['uavs'][uav_id]
        uav_schedule = schedule_data['uav_schedule'][uav_id]

        # 绘制初始位置（绿色三角形）
        init_pos = uav_info['init_pos']
        ax.plot(init_pos[0], init_pos[1], '^',
                color='green', markersize=5, alpha=0.6, zorder=4)

        # 绘制每个任务的到达位置
        for schedule_item in uav_schedule:
            task_id = schedule_item['task']
            task = task_map[task_id]
            start_time = schedule_item['start']
            time_step = int(start_time)
            target = schedule_item['target']

            if time_step > 0 and time_step <= len(uav_info['positions']):
                uav_pos = uav_info['positions'][time_step - 1]
                distance = np.sqrt((uav_pos[0] - task['center'][0])**2 +
                                   (uav_pos[1] - task['center'][1])**2)

                # 到达用实心圆，未到达用空心圆
                arrived = distance <= task['radius']
                if arrived:
                    ax.plot(uav_pos[0], uav_pos[1], 'o',
                            color='blue', markersize=6,
                            alpha=0.7, zorder=5)
                else:
                    ax.plot(uav_pos[0], uav_pos[1], 'o',
                            markerfacecolor='none',
                            markeredgecolor='red', markersize=7,
                            markeredgewidth=1.5, alpha=0.8, zorder=5)

    # ========== 4. 设置图表属性 ==========
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'UAV Trajectories with Task Locations\n'
                 f'({n_uavs} UAVs with Rainbow Colors, {len(task_data["task_list"])} Tasks with Actual Radius)',
                 fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # ========== 5. 添加图例 ==========
    # 任务类型图例
    legend_elements = [
        Patch(facecolor='#FFD700', edgecolor='black', alpha=0.5, label='侦察 (surveillance)'),
        Patch(facecolor='#FF6B6B', edgecolor='black', alpha=0.5, label='攻击 (attack)'),
        Patch(facecolor='#4ECDC4', edgecolor='black', alpha=0.5, label='捕获 (capture)'),
        plt.Line2D([0], [0], marker='^', color='w', markerfacecolor='green',
                   markersize=8, label='UAV初始位置'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue',
                   markersize=8, label='已到达任务位置'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='none',
                   markeredgecolor='red', markersize=8, label='未到达'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)

    plt.tight_layout()

    # 输出到当前目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'combined_visualization.png')

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"综合可视化图已保存到: {output_path}")

    return output_path


if __name__ == '__main__':
    print("=" * 60)
    print("生成综合可视化图")
    print("=" * 60)
    print("\n特性:")
    print("  - 任务半径: 使用实际半径 (from uav_arrival_visualization)")
    print("  - 任务颜色: 与 criticalpath_allocation_full.png 匹配")
    print("  - UAV轨迹: 80个UAV彩虹色 (from uav_trajectories_full)")
    print()

    output = plot_combined_visualization()
    print(f"\n完成! 输出文件: {output}")
