#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UAV轨迹可视化 - 900步80个UAV的完整轨迹
"""
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import hsv_to_rgb, rgb_to_hsv
from matplotlib.collections import LineCollection


def generate_rainbow_colors(n):
    """生成彩虹渐变颜色"""
    colors = []
    for i in range(n):
        hue = i / n
        rgb = hsv_to_rgb((hue, 1.0, 0.8))
        colors.append(rgb)
    return colors


def load_uav_data(filepath='uav_positions_over_time.json'):
    """加载UAV位置数据"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def extract_trajectories(data):
    """提取所有UAV的轨迹"""
    uav_trajectories = {}
    for uav_id, uav_data in data['uavs'].items():
        init_pos = uav_data['init_pos']
        positions = uav_data['positions']
        # 添加初始位置作为起点
        full_positions = [init_pos] + positions
        uav_trajectories[uav_id] = full_positions
    return uav_trajectories


def plot_uav_trajectories(uav_trajectories, save_path='uav_trajectories_full.png'):
    """绘制所有UAV的完整轨迹"""
    n_uavs = len(uav_trajectories)
    colors = generate_rainbow_colors(n_uavs)

    fig, ax = plt.subplots(figsize=(20, 20))

    # 绘制每个UAV的轨迹
    for i, (uav_id, positions) in enumerate(uav_trajectories.items()):
        if len(positions) < 2:
            continue

        pos_array = np.array(positions)
        # 绘制轨迹线
        ax.plot(pos_array[:, 0], pos_array[:, 1],
                color=colors[i], linewidth=0.5, alpha=0.6, label=f'UAV {uav_id}')

        # 绘制起点
        ax.plot(positions[0][0], positions[0][1], 'o',
                color=colors[i], markersize=3, alpha=0.8)

        # 绘制终点
        ax.plot(positions[-1][0], positions[-1][1], 's',
                color=colors[i], markersize=5, alpha=1.0)

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'UAV Trajectories (900 Time Steps, {n_uavs} UAVs)', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8, ncol=2)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"轨迹图已保存到: {save_path}")


def plot_time_step_final(data, step_idx=900, save_path='uav_positions_step900.png'):
    """绘制指定时间步的所有UAV位置"""
    if step_idx < 0 or step_idx > 900:
        step_idx = 900

    fig, ax = plt.subplots(figsize=(20, 20))

    n_uavs = len(data['uavs'])
    colors = generate_rainbow_colors(n_uavs)

    # 获取当前时间步的所有UAV位置
    positions_data = {}
    for uav_id, uav_data in data['uavs'].items():
        positions = uav_data['positions']
        if step_idx <= len(positions):
            positions_data[uav_id] = positions[step_idx - 1] if step_idx > 0 else uav_data['init_pos']
        else:
            positions_data[uav_id] = positions[-1] if positions else uav_data['init_pos']

    # 绘制每个UAV当前位置（实心圆圈）
    for i, (uav_id, pos) in enumerate(positions_data.items()):
        circle = patches.Circle((pos[0], pos[1]), 2.0,
                           facecolor=colors[i], edgecolor='black', linewidth=0.5, alpha=0.9,
                           label=f'UAV {uav_id}')
        ax.add_patch(circle)

    # 绘制之前的轨迹（淡化显示）
    for i, (uav_id, uav_data) in enumerate(data['uavs'].items()):
        positions = [uav_data['init_pos']] + uav_data['positions'][:step_idx]
        if len(positions) >= 2:
            pos_array = np.array(positions)
            ax.plot(pos_array[:, 0], pos_array[:, 1],
                    color=colors[i], linewidth=0.3, alpha=0.4)

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'UAV Positions at Time Step {step_idx} (t={step_idx * 1.0:.1f}s)',
                  fontsize=14)
    ax.set_xlim(0, 300)
    ax.set_ylim(0, 300)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=7, ncol=2)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"时间步{step_idx}位置图已保存到: {save_path}")


def plot_time_sequence_grid(data, steps=[1, 300, 600, 900],
                             save_path='uav_positions_grid.png'):
    """绘制多个时间步的网格视图"""
    n_steps = len(steps)
    # 根据步数动态调整子图布局
    if n_steps <= 4:
        fig, axes = plt.subplots(2, 2, figsize=(18, 16))
    elif n_steps <= 6:
        fig, axes = plt.subplots(2, 3, figsize=(18, 16))
    else:
        fig, axes = plt.subplots(3, 3, figsize=(18, 16))

    n_uavs = len(data['uavs'])
    colors = generate_rainbow_colors(n_uavs)

    for idx, step in enumerate(steps):
        # 根据axes布局计算正确的索引
        if n_steps <= 4:
            ax = axes[idx // 2, idx % 2]
        elif n_steps <= 6:
            ax = axes[idx // 2, idx % 2]
        else:
            row = idx // 3
            col = idx % 3
            ax = axes[row, col]
        ax.set_title(f'Time Step {step}', fontsize=11)

        positions_data = {}
        for uav_id, uav_data in data['uavs'].items():
            positions = uav_data['positions']
            if step <= len(positions):
                positions_data[uav_id] = positions[step - 1] if step > 0 else uav_data['init_pos']
            else:
                positions_data[uav_id] = positions[-1] if positions else uav_data['init_pos']

        # 绘制UAV位置
        for i, (uav_id, pos) in enumerate(positions_data.items()):
            if i < 30:  # 只显示部分UAV的label，避免拥挤
                ax.plot(pos[0], pos[1], 'o',
                        color=colors[i], markersize=2, alpha=0.8)
            else:
                ax.plot(pos[0], pos[1], 'o',
                        color=colors[i], markersize=2, alpha=0.8)

        ax.set_xlim(0, 300)
        ax.set_ylim(0, 300)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"时间序列网格图已保存到: {save_path}")


def main():
    """主函数"""
    print("=" * 60)
    print("UAV轨迹可视化工具")
    print("=" * 60)

    # 加载数据
    data = load_uav_data()
    print(f"数据来源: uav_positions_over_time.json")
    print(f"总UAV数: {len(data['uavs'])}")
    print(f"总时间步: {len(data['uavs']['0']['positions'])}")
    print(f"总时间: {data['makespan']:.1f}s")
    print()

    # 提取轨迹
    uav_trajectories = extract_trajectories(data)
    print(f"提取轨迹完成，共 {len(uav_trajectories)} 个UAV")

    # 绘制完整轨迹图
    print("\n[1/3] 绘制完整轨迹图...")
    plot_uav_trajectories(uav_trajectories, 'uav_trajectories_full.png')

    # 绘制最后时刻的位置图
    print("[2/3] 绘制第900步位置图...")
    plot_time_step_final(data, step_idx=900, save_path='uav_positions_step900.png')

    # 绘制时间序列网格
    print("[3/3] 绘制时间序列网格...")
    plot_time_sequence_grid(data, steps=[1, 100, 200, 300, 450, 600, 750, 900],
                           save_path='uav_positions_grid.png')

    print("\n" + "=" * 60)
    print("所有图像生成完成！")
    print("=" * 60)


if __name__ == '__main__':
    main()
