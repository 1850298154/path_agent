#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UAV-Task能力匹配可视化 V2
清晰展示每个UAV能支持哪些任务类型

规则：
- uavA (skills [1,2]): 可执行 attack(A/B), surveillance(A), capture(A)
- uavB (skills [1,3]): 可执行 attack(A/B), surveillance(B), capture(B)
- uavC (skills [2,3]): 可执行 surveillance(A/B), capture(A/B)

颜色 (与 criticalpath_allocation_full.png 一致):
- surveillance: 金色 (A: #FFD700, B: #FFA500)
- attack: 红色 (A: #FF6B6B, B: #EE5A5A)
- capture: 青色 (A: #4ECDC4, B: #44A08D)
"""
import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Circle, Wedge, FancyBboxPatch, Rectangle
from matplotlib.collections import PatchCollection

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


# 任务类型颜色 (与 criticalpath_allocation_full.png 完全一致)
TASK_COLORS = {
    'surveillance': {'A': '#FFD700', 'B': '#FFA500'},  # 金色系
    'attack': {'A': '#FF6B6B', 'B': '#EE5A5A'},         # 红色系
    'capture': {'A': '#4ECDC4', 'B': '#44A08D'}         # 青色系
}


def load_rule():
    """加载规则"""
    rule_path = os.path.join(os.path.dirname(__file__), 'rule.json')
    with open(rule_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def get_uav_capabilities(uav_type):
    """
    根据UAV类型返回它能执行的任务类型和阶段

    uavA (skills [1,2]): attack(A/B), surveillance(A), capture(A)
    uavB (skills [1,3]): attack(A/B), surveillance(B), capture(B)
    uavC (skills [2,3]): surveillance(A/B), capture(A/B)
    """
    if uav_type == 'uavA':
        return {
            'attack': ['A', 'B'],
            'surveillance': ['A'],
            'capture': ['A']
        }
    elif uav_type == 'uavB':
        return {
            'attack': ['A', 'B'],
            'surveillance': ['B'],
            'capture': ['B']
        }
    elif uav_type == 'uavC':
        return {
            'surveillance': ['A', 'B'],
            'capture': ['A', 'B']
        }
    return {}


def draw_uav_capability_circle(ax, x, y, uav_id, uav_type, radius=1.0):
    """
    绘制单个UAV的能力圆圈
    使用扇形分区表示可执行的任务类型
    """
    capabilities = get_uav_capabilities(uav_type)

    # 收集所有能力
    capability_sectors = []
    for task_type, phases in capabilities.items():
        if len(phases) == 2:
            # 能执行A和B阶段 - 使用主颜色
            capability_sectors.append((task_type, 'both'))
        else:
            # 只能执行一个阶段
            capability_sectors.append((task_type, phases[0]))

    # 绘制扇形
    n_sectors = len(capability_sectors)
    if n_sectors == 0:
        # 无能力 - 灰色圆
        circle = Circle((x, y), radius, facecolor='lightgray', edgecolor='black', linewidth=1)
        ax.add_patch(circle)
    elif n_sectors == 1:
        # 单一能力 - 整圆
        task_type, phase = capability_sectors[0]
        if phase == 'both':
            # 画两个半圆
            wedge1 = Wedge((x, y), radius, 0, 180, facecolor=TASK_COLORS[task_type]['A'],
                          edgecolor='black', linewidth=1)
            wedge2 = Wedge((x, y), radius, 180, 360, facecolor=TASK_COLORS[task_type]['B'],
                          edgecolor='black', linewidth=1)
            ax.add_patch(wedge1)
            ax.add_patch(wedge2)
        else:
            color = TASK_COLORS[task_type][phase]
            circle = Circle((x, y), radius, facecolor=color, edgecolor='black', linewidth=1)
            ax.add_patch(circle)
    elif n_sectors == 2:
        # 两种能力 - 半圆分割
        angle_step = 180
        for i, (task_type, phase) in enumerate(capability_sectors):
            start_angle = i * angle_step
            end_angle = (i + 1) * angle_step
            if phase == 'both':
                # 这个扇形再分成两个子扇形
                sub_angle_step = angle_step / 2
                wedge1 = Wedge((x, y), radius, start_angle, start_angle + sub_angle_step,
                              facecolor=TASK_COLORS[task_type]['A'], edgecolor='black', linewidth=0.5)
                wedge2 = Wedge((x, y), radius, start_angle + sub_angle_step, end_angle,
                              facecolor=TASK_COLORS[task_type]['B'], edgecolor='black', linewidth=0.5)
                ax.add_patch(wedge1)
                ax.add_patch(wedge2)
            else:
                color = TASK_COLORS[task_type][phase]
                wedge = Wedge((x, y), radius, start_angle, end_angle,
                             facecolor=color, edgecolor='black', linewidth=1)
                ax.add_patch(wedge)
    elif n_sectors == 3:
        # 三种能力 - 三等分
        angle_step = 120
        for i, (task_type, phase) in enumerate(capability_sectors):
            start_angle = i * angle_step
            end_angle = (i + 1) * angle_step
            if phase == 'both':
                # 再细分
                sub_angle_step = angle_step / 2
                wedge1 = Wedge((x, y), radius, start_angle, start_angle + sub_angle_step,
                              facecolor=TASK_COLORS[task_type]['A'], edgecolor='black', linewidth=0.5)
                wedge2 = Wedge((x, y), radius, start_angle + sub_angle_step, end_angle,
                              facecolor=TASK_COLORS[task_type]['B'], edgecolor='black', linewidth=0.5)
                ax.add_patch(wedge1)
                ax.add_patch(wedge2)
            else:
                color = TASK_COLORS[task_type][phase]
                wedge = Wedge((x, y), radius, start_angle, end_angle,
                             facecolor=color, edgecolor='black', linewidth=0.5)
                ax.add_patch(wedge)

    # 中心白色圆 + UAV ID
    center = Circle((x, y), radius * 0.35, facecolor='white', edgecolor='black', linewidth=0.5)
    ax.add_patch(center)
    ax.text(x, y, str(uav_id), ha='center', va='center', fontsize=7, fontweight='bold')


def plot_uav_capability_grid(output_path):
    """
    绘制UAV能力网格图
    每个UAV用一个圆表示，圆内用任务颜色填充
    """
    rule = load_rule()
    uav_skills = rule['技能与价值配置']['UAV能力技能']

    fig, ax = plt.subplots(figsize=(24, 20))

    # 布局：10列×8行
    n_cols = 10
    n_rows = 8
    spacing = 2.8

    # 按UAV类型分组绘制
    uav_types = ['uavA', 'uavB', 'uavC']

    for uav_id in range(80):
        # 确定UAV类型
        if uav_id < 10 or (40 <= uav_id < 50):
            uav_type = 'uavA'
        elif uav_id < 20 or (50 <= uav_id < 60):
            uav_type = 'uavB'
        else:
            uav_type = 'uavC'

        row = uav_id // n_cols
        col = uav_id % n_cols

        x = col * spacing + 1.4
        y = (n_rows - 1 - row) * spacing + 1.4

        draw_uav_capability_circle(ax, x, y, uav_id, uav_type, radius=1.2)

    # 添加UAV类型分隔线
    for uav_id in [10, 20, 40, 50, 60]:
        row = uav_id // n_cols
        y_line = (n_rows - 1 - row + 0.5) * spacing + 1.4
        if uav_id == 20 or uav_id == 60:
            # uavA/uavB 与 uavC 之间的分隔
            ax.axhline(y=y_line, color='red', linewidth=2, linestyle='--', alpha=0.7)
        else:
            ax.axhline(y=y_line, color='gray', linewidth=1, linestyle=':', alpha=0.5)

    # 添加类型标签
    label_x = n_cols * spacing + 2
    type_ranges = [
        ('uavA (skills 1,2)', 0, 10),
        ('uavA (skills 1,2)', 40, 50),
        ('uavB (skills 1,3)', 10, 20),
        ('uavB (skills 1,3)', 50, 60),
        ('uavC (skills 2,3)', 20, 40),
        ('uavC (skills 2,3)', 60, 80),
    ]

    # 设置图表属性
    ax.set_xlim(-0.5, n_cols * spacing + 1)
    ax.set_ylim(-1, n_rows * spacing + 1)
    ax.set_aspect('equal')
    ax.axis('off')

    # 标题
    ax.set_title('UAV Task Capability Map\n'
                 'Each circle shows which task types the UAV can execute\n'
                 'Colors match criticalpath_allocation_full.png',
                 fontsize=14, fontweight='bold', pad=20)

    # 图例
    legend_elements = [
        mpatches.Patch(facecolor=TASK_COLORS['surveillance']['A'], edgecolor='black',
                      label='Surveillance A (skill 2)'),
        mpatches.Patch(facecolor=TASK_COLORS['surveillance']['B'], edgecolor='black',
                      label='Surveillance B (skill 3)'),
        mpatches.Patch(facecolor=TASK_COLORS['attack']['A'], edgecolor='black',
                      label='Attack A (skill 1)'),
        mpatches.Patch(facecolor=TASK_COLORS['attack']['B'], edgecolor='black',
                      label='Attack B (skill 1)'),
        mpatches.Patch(facecolor=TASK_COLORS['capture']['A'], edgecolor='black',
                      label='Capture A (skill 2)'),
        mpatches.Patch(facecolor=TASK_COLORS['capture']['B'], edgecolor='black',
                      label='Capture B (skill 3)'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9, ncol=2)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_uav_capability_summary(output_path):
    """
    绘制简洁的UAV能力汇总图
    按UAV类型分组，清晰展示每种UAV能执行的任务
    """
    fig, axes = plt.subplots(1, 3, figsize=(18, 8))

    uav_configs = [
        ('uavA', [1, 2], {'attack': ['A', 'B'], 'surveillance': ['A'], 'capture': ['A']}),
        ('uavB', [1, 3], {'attack': ['A', 'B'], 'surveillance': ['B'], 'capture': ['B']}),
        ('uavC', [2, 3], {'surveillance': ['A', 'B'], 'capture': ['A', 'B']}),
    ]

    for ax, (uav_type, skills, capabilities) in zip(axes, uav_configs):
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_aspect('equal')
        ax.axis('off')

        # 标题
        ax.set_title(f'{uav_type}\nSkills: {skills}', fontsize=16, fontweight='bold', pad=10)

        # 绘制任务类型方块
        y_pos = 7
        for task_type in ['surveillance', 'attack', 'capture']:
            if task_type in capabilities:
                phases = capabilities[task_type]

                # 绘制任务标签
                ax.text(1, y_pos, f'{task_type}:', fontsize=12, ha='left', va='center')

                # 绘制阶段方块
                x_pos = 4
                for phase in phases:
                    color = TASK_COLORS[task_type][phase]
                    rect = Rectangle((x_pos, y_pos - 0.4), 1.5, 0.8,
                                     facecolor=color, edgecolor='black', linewidth=1)
                    ax.add_patch(rect)
                    ax.text(x_pos + 0.75, y_pos, phase, ha='center', va='center',
                           fontsize=10, fontweight='bold')
                    x_pos += 2

                # 打勾标记
                ax.text(9, y_pos, '✓', fontsize=14, ha='center', va='center', color='green')
            else:
                # 不能执行
                ax.text(1, y_pos, f'{task_type}:', fontsize=12, ha='left', va='center', color='gray')
                ax.text(4, y_pos, '—', fontsize=14, ha='center', va='center', color='gray')
                ax.text(9, y_pos, '✗', fontsize=14, ha='center', va='center', color='red')

            y_pos -= 1.8

        # 添加说明
        ax.text(5, 1, f'UAV IDs: see below', fontsize=10, ha='center', va='center', color='gray')

    plt.suptitle('UAV Task Capability Summary\nColors match criticalpath_allocation_full.png',
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_uav_capability_matrix(output_path):
    """
    绘制UAV-Task能力矩阵
    行=UAV, 列=Task, 单元格显示能否执行
    """
    # 加载任务数据
    data_path = os.path.join(os.path.dirname(__file__), '..', 'data', '01_original_data', 'precomputed_data.json')
    with open(data_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    uav_list = data['uav_list']
    task_list = data['task_list']

    fig, ax = plt.subplots(figsize=(24, 20))

    # 创建能力矩阵
    # 值: 0=不能, 1=A阶段, 2=B阶段, 3=A+B阶段
    matrix = np.zeros((80, 30, 3))  # 3通道: surveillance, attack, capture

    for i, uav in enumerate(uav_list):
        capabilities = get_uav_capabilities(uav['type'])

        for j, task in enumerate(task_list):
            task_type = task['type']

            if task_type in capabilities:
                phases = capabilities[task_type]
                if 'A' in phases and 'B' in phases:
                    matrix[i, j, ['surveillance', 'attack', 'capture'].index(task_type)] = 3
                elif 'A' in phases:
                    matrix[i, j, ['surveillance', 'attack', 'capture'].index(task_type)] = 1
                elif 'B' in phases:
                    matrix[i, j, ['surveillance', 'attack', 'capture'].index(task_type)] = 2

    # 绘制矩阵
    cell_width = 0.8
    cell_height = 0.3

    for i in range(80):
        for j in range(30):
            task = task_list[j]
            task_type = task['type']
            type_idx = ['surveillance', 'attack', 'capture'].index(task_type)

            val = matrix[i, j, type_idx]

            if val == 0:
                color = '#f0f0f0'
            elif val == 1:
                color = TASK_COLORS[task_type]['A']
            elif val == 2:
                color = TASK_COLORS[task_type]['B']
            else:  # val == 3
                # 画两半
                rect1 = Rectangle((j * cell_width, i * cell_height),
                                  cell_width / 2, cell_height,
                                  facecolor=TASK_COLORS[task_type]['A'],
                                  edgecolor='gray', linewidth=0.3)
                rect2 = Rectangle((j * cell_width + cell_width / 2, i * cell_height),
                                  cell_width / 2, cell_height,
                                  facecolor=TASK_COLORS[task_type]['B'],
                                  edgecolor='gray', linewidth=0.3)
                ax.add_patch(rect1)
                ax.add_patch(rect2)
                continue

            rect = Rectangle((j * cell_width, i * cell_height),
                            cell_width, cell_height,
                            facecolor=color, edgecolor='gray', linewidth=0.3)
            ax.add_patch(rect)

    # 设置坐标轴
    ax.set_xlim(0, 30 * cell_width)
    ax.set_ylim(0, 80 * cell_height)

    # 添加任务类型标签
    for j, task in enumerate(task_list):
        task_type = task['type']
        color = TASK_COLORS[task_type]['A']
        ax.text(j * cell_width + cell_width/2, 80 * cell_height + 0.5,
               f"{task['task_id']}\n{task_type[:3]}", ha='center', va='bottom',
               fontsize=6, rotation=0, color=color)

    # 添加UAV标签（每10个显示一次）
    for i in range(0, 80, 10):
        ax.text(-0.5, i * cell_height + cell_height * 5, f'UAV {i}-{i+9}',
               ha='right', va='center', fontsize=8)

    ax.axis('off')
    ax.set_title('UAV-Task Capability Matrix\n'
                 'Colors: Gold=Surveillance, Red=Attack, Cyan=Capture\n'
                 'Left half=A phase, Right half=B phase',
                 fontsize=12, fontweight='bold', pad=10)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def main():
    print("=" * 60)
    print("UAV Task Capability Visualization V2")
    print("=" * 60)

    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("\n[1/3] Generating capability grid...")
    plot_uav_capability_grid(os.path.join(script_dir, 'uav_capability_grid.png'))

    print("\n[2/3] Generating capability summary...")
    plot_uav_capability_summary(os.path.join(script_dir, 'uav_capability_summary.png'))

    print("\n[3/3] Generating capability matrix...")
    plot_uav_capability_matrix(os.path.join(script_dir, 'uav_capability_matrix.png'))

    print("\n" + "=" * 60)
    print("Done!")
    print("=" * 60)


if __name__ == '__main__':
    main()
