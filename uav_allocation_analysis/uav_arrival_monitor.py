#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UAV到位监控 - 检查UAV是否到达任务位置（进入圆圈内）
"""
import json
import numpy as np


def load_data():
    """加载所有必要数据"""
    # 加载UAV位置数据
    with open('uav_positions_over_time.json', 'r', encoding='utf-8') as f:
        uav_data = json.load(f)

    # 加载任务数据
    with open('precomputed_data.json', 'r', encoding='utf-8') as f:
        task_data = json.load(f)

    # 加载调度结果
    with open('result_criticalpath_new.json', 'r', encoding='utf-8') as f:
        schedule_data = json.load(f)

    return uav_data, task_data, schedule_data


def create_task_map(task_list):
    """创建任务ID到任务信息的映射"""
    return {task['task_id']: task for task in task_list}


def is_within_circle(uav_pos, task_center, task_radius):
    """检查UAV是否在任务圆圈内"""
    distance = np.sqrt((uav_pos[0] - task_center[0])**2 + (uav_pos[1] - task_center[1])**2)
    return distance <= task_radius


def check_uav_arrival(uav_id, uav_positions, uav_schedule, task_map):
    """检查单个UAV是否到达任务位置"""
    arrival_info = []

    for schedule_item in uav_schedule:
        task_id = schedule_item['task']
        task = task_map[task_id]
        task_center = task['center']
        task_radius = task['radius']

        # 找到任务执行开始时间对应的时间步
        start_time = schedule_item['start']
        time_step = int(start_time)  # 时间步 = 时间（秒）

        # 检查该时间步UAV是否在任务圆圈内
        if time_step > 0 and time_step <= len(uav_positions):
            uav_pos = uav_positions[time_step - 1]
            arrived = is_within_circle(uav_pos, task_center, task_radius)
            distance = np.sqrt((uav_pos[0] - task_center[0])**2 + (uav_pos[1] - task_center[1])**2)

            arrival_info.append({
                'task_id': task_id,
                'task_type': task['type'],
                'task_center': task_center,
                'task_radius': task_radius,
                'target': schedule_item['target'],
                'start_time': start_time,
                'time_step': time_step,
                'uav_pos': uav_pos,
                'distance': distance,
                'arrived': arrived
            })

    return arrival_info


def monitor_all_uavs():
    """监控所有UAV的到位情况"""
    uav_data, task_data, schedule_data = load_data()

    print("=" * 80)
    print("UAV到位监控报告")
    print("=" * 80)

    # 创建任务映射
    task_map = create_task_map(task_data['task_list'])

    # 统计数据
    total_checks = 0
    arrived_count = 0
    not_arrived_count = 0

    # 检查每个UAV
    for uav_id in sorted(uav_data['uavs'].keys(), key=int):
        uav_positions = uav_data['uavs'][uav_id]['positions']
        uav_schedule = schedule_data['uav_schedule'][uav_id]

        arrivals = check_uav_arrival(uav_id, uav_positions, uav_schedule, task_map)

        for idx, info in enumerate(arrivals):
            status = "[OK] 到达" if info['arrived'] else "[NO] 未到达"
            total_checks += 1
            if info['arrived']:
                arrived_count += 1
            else:
                not_arrived_count += 1

            print(f"UAV {uav_id:>2} - 任务{info['task_id']:>2} ({info['task_type']:>12}) - "
                  f"{info['target']}阶段 - t={info['start_time']:7.2f}s - "
                  f"距离={info['distance']:7.2f}m - {status}")

            # 如果未到达，显示UAV实际位置和任务中心
            if not info['arrived']:
                print(f"       UAV位置: [{info['uav_pos'][0]:7.2f}, {info['uav_pos'][1]:7.2f}] "
                      f"任务中心: [{info['task_center'][0]:7.2f}, {info['task_center'][1]:7.2f}] "
                      f"任务半径: {info['task_radius']:5.2f}m")

    # 统计汇总
    print("\n" + "=" * 80)
    print("统计汇总")
    print("=" * 80)
    arrival_rate = (arrived_count / total_checks * 100) if total_checks > 0 else 0
    print(f"总检查次数: {total_checks}")
    print(f"已到位:     {arrived_count}")
    print(f"未到位:     {not_arrived_count}")
    print(f"到位率:     {arrival_rate:.2f}%")

    # 保存详细报告
    save_detailed_report(uav_data, task_data, schedule_data, task_map, arrival_rate)


def save_detailed_report(uav_data, task_data, schedule_data, task_map, arrival_rate):
    """保存详细报告到文件"""
    report = {
        'timestamp': '2026-02-27',
        'arrival_rate': round(arrival_rate, 2),
        'uav_details': {}
    }

    for uav_id in sorted(uav_data['uavs'].keys(), key=int):
        uav_positions = uav_data['uavs'][uav_id]['positions']
        uav_schedule = schedule_data['uav_schedule'][uav_id]

        report['uav_details'][uav_id] = {
            'total_tasks': len(uav_schedule),
            'arrivals': []
        }

        for schedule_item in uav_schedule:
            task_id = schedule_item['task']
            task = task_map[task_id]
            start_time = schedule_item['start']
            time_step = int(start_time)

            if time_step > 0 and time_step <= len(uav_positions):
                uav_pos = uav_positions[time_step - 1]
                distance = np.sqrt((uav_pos[0] - task['center'][0])**2 +
                               (uav_pos[1] - task['center'][1])**2)
                arrived = distance <= task['radius']

                report['uav_details'][uav_id]['arrivals'].append({
                    'task_id': task_id,
                    'task_type': task['type'],
                    'target': schedule_item['target'],
                    'start_time': round(start_time, 2),
                    'time_step': time_step,
                    'uav_position': uav_pos,
                    'task_center': task['center'],
                    'task_radius': task['radius'],
                    'distance': round(distance, 3),
                    'arrived': bool(arrived)
                })

    with open('uav_arrival_report.json', 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n详细报告已保存到: uav_arrival_report.json")


def visualize_arrival_status():
    """可视化UAV到位状态"""
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    uav_data, task_data, schedule_data = load_data()
    task_map = create_task_map(task_data['task_list'])

    fig, ax = plt.subplots(figsize=(20, 20))

    # 绘制任务区域
    task_colors = {
        'surveillance': '#FFD700',  # 金色
        'attack': '#FF6B6B',       # 红色
        'capture': '#4ECDC4'        # 青色
    }

    for task in task_data['task_list']:
        color = task_colors.get(task['type'], '#888888')
        circle = patches.Circle((task['center'][0], task['center'][1]),
                              task['radius'],
                              facecolor=color, edgecolor='black',
                              linewidth=1.0, alpha=0.3,
                              label=f'Task {task["task_id"]}')
        ax.add_patch(circle)
        # 添加任务标签
        ax.text(task['center'][0], task['center'][1],
                f"{task['task_id']}", fontsize=8,
                ha='center', va='center', weight='bold')

    # 绘制UAV初始位置和到达位置
    for uav_id in sorted(uav_data['uavs'].keys(), key=int):
        uav_info = uav_data['uavs'][uav_id]
        uav_schedule = schedule_data['uav_schedule'][uav_id]

        # 绘制初始位置（三角形）
        init_pos = uav_info['init_pos']
        ax.plot(init_pos[0], init_pos[1], '^',
                color='green', markersize=6, alpha=0.8, zorder=3)

        # 绘制每个任务的到达位置
        for schedule_item in uav_schedule:
            task_id = schedule_item['task']
            task = task_map[task_id]
            start_time = schedule_item['start']
            time_step = int(start_time)

            if time_step > 0 and time_step <= len(uav_info['positions']):
                uav_pos = uav_info['positions'][time_step - 1]
                distance = np.sqrt((uav_pos[0] - task['center'][0])**2 +
                               (uav_pos[1] - task['center'][1])**2)

                # 到达用实心圆，未到达用空心圆
                marker = 'o' if distance <= task['radius'] else 'o'
                facecolor = 'none' if distance > task['radius'] else 'blue'
                edgecolor = 'red' if distance > task['radius'] else 'blue'
                linewidth = 2 if distance > task['radius'] else 1

                ax.plot(uav_pos[0], uav_pos[1], marker,
                        color=edgecolor, markersize=8,
                        markerfacecolor=facecolor,
                        markeredgecolor=edgecolor,
                        markeredgewidth=linewidth, alpha=0.8, zorder=4)

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'UAV Arrival Status Visualization\n(Green △ = Initial, Blue ● = Arrived, Red ○ = Not Arrived)',
                  fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 添加图例
    legend_elements = [
        plt.Line2D([0], [0], marker='^', color='w', markerfacecolor='green',
                    markersize=10, label='Initial Position'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue',
                    markersize=10, label='Arrived at Task'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='none',
                    markeredgecolor='red', markersize=10, label='Not Arrived'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=12)

    plt.tight_layout()
    plt.savefig('uav_arrival_visualization.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f"可视化图已保存到: uav_arrival_visualization.png")


if __name__ == '__main__':
    # 运行监控
    monitor_all_uavs()

    # 生成可视化
    print("\n" + "=" * 80)
    print("生成到位状态可视化图...")
    print("=" * 80)
    visualize_arrival_status()

    print("\n" + "=" * 80)
    print("监控完成！")
    print("=" * 80)
