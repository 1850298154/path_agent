#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UAV-Task能力匹配可视化
展示80个UAV的技能与30个任务的匹配关系

设计思路：
- 每个UAV用同心双环表示两个技能
- 每个技能环用该技能可执行任务类型的颜色填充
- 颜色与 criticalpath_allocation_full.png 匹配
"""
import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Wedge, Circle, Arc
from matplotlib.collections import PatchCollection
import matplotlib.colors as mcolors

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


def load_data():
    """加载数据"""
    data_path = os.path.join(os.path.dirname(__file__), '..', 'data', '01_original_data', 'precomputed_data.json')
    with open(data_path, 'r', encoding='utf-8') as f:
        return json.load(f)


# 任务类型颜色 (与 criticalpath_allocation_full.png 一致)
TASK_COLORS = {
    'surveillance': {'A': '#FFD700', 'B': '#FFA500'},  # 金色系
    'attack': {'A': '#FF6B6B', 'B': '#EE5A5A'},         # 红色系
    'capture': {'A': '#4ECDC4', 'B': '#44A08D'}         # 青色系
}

# 技能对应的任务类型映射
# skill 1: 可以执行 attack 任务 (A/B阶段都需要skill 1)
# skill 2: 可以执行 surveillance/capture 任务的 A阶段
# skill 3: 可以执行 surveillance/capture 任务的 B阶段
SKILL_TASK_MAPPING = {
    1: {'task_type': 'attack', 'phase': 'both'},      # skill 1 用于 attack A/B
    2: {'task_type': 'surveillance', 'phase': 'A'},   # skill 2 用于 surveillance/capture A
    3: {'task_type': 'capture', 'phase': 'B'}         # skill 3 用于 surveillance/capture B
}

# 技能颜色 (根据可执行的任务类型)
SKILL_COLORS = {
    1: '#FF6B6B',  # 红色 - attack
    2: '#FFD700',  # 金色 - surveillance/capture A阶段
    3: '#4ECDC4'   # 青色 - surveillance/capture B阶段
}


def get_skill_color(skill):
    """获取技能对应的颜色"""
    return SKILL_COLORS.get(skill, '#888888')


def plot_uav_capability_circles(data, output_path):
    """
    绘制UAV能力圆环图
    每个UAV用一个圆表示，圆内有两个同心环代表两个技能
    """
    uav_list = data['uav_list']

    fig, axes = plt.subplots(2, 1, figsize=(28, 24))

    # ===== 上半部分: UAV技能圆环展示 =====
    ax1 = axes[0]

    # 计算布局：10列8行
    n_cols = 10
    n_rows = 8
    circle_spacing = 2.8

    for i, uav in enumerate(uav_list):
        row = i // n_cols
        col = i % n_cols

        x = col * circle_spacing + 1.4
        y = (n_rows - 1 - row) * circle_spacing + 1.4

        skills = sorted(uav['skills'])

        # 绘制外环 (第一个技能，半径1.0-1.3)
        skill1 = skills[0]
        wedge1 = Wedge((x, y), 1.3, 0, 180, width=0.3,
                       facecolor=get_skill_color(skill1), edgecolor='black', linewidth=1)
        ax1.add_patch(wedge1)

        # 绘制内环 (第二个技能，半径0.6-0.9)
        skill2 = skills[1]
        wedge2 = Wedge((x, y), 0.9, 180, 360, width=0.3,
                       facecolor=get_skill_color(skill2), edgecolor='black', linewidth=1)
        ax1.add_patch(wedge2)

        # 中心圆
        center = Circle((x, y), 0.5, facecolor='white', edgecolor='black', linewidth=1)
        ax1.add_patch(center)

        # UAV ID
        ax1.text(x, y, str(uav['uav_id']), ha='center', va='center', fontsize=9, fontweight='bold')

        # UAV类型标签
        ax1.text(x, y - 1.6, uav['type'], ha='center', va='top', fontsize=8, color='gray')

    ax1.set_xlim(-0.5, n_cols * circle_spacing)
    ax1.set_ylim(-1, n_rows * circle_spacing + 0.5)
    ax1.set_aspect('equal')
    ax1.set_title('(1) UAV Skill Capability Map - 80 UAVs with Dual Skills\n'
                  'Upper Ring = Primary Skill, Lower Ring = Secondary Skill',
                  fontsize=14, fontweight='bold', pad=20)
    ax1.axis('off')

    # 添加图例
    legend_elements = [
        mpatches.Patch(facecolor=SKILL_COLORS[1], edgecolor='black', label='Skill 1 (Attack)'),
        mpatches.Patch(facecolor=SKILL_COLORS[2], edgecolor='black', label='Skill 2 (Surveillance/Capture A)'),
        mpatches.Patch(facecolor=SKILL_COLORS[3], edgecolor='black', label='Skill 3 (Surveillance/Capture B)'),
    ]
    ax1.legend(handles=legend_elements, loc='upper right', fontsize=10)

    # ===== 下半部分: UAV-Task匹配矩阵 =====
    ax2 = axes[1]

    task_list = data['task_list']
    n_uavs = len(uav_list)
    n_tasks = len(task_list)

    # 创建匹配矩阵
    # 0: 不能执行
    # 1: 只能执行A阶段
    # 2: 只能执行B阶段
    # 3: 可执行A和B阶段
    match_matrix = np.zeros((n_uavs, n_tasks))

    for i, uav in enumerate(uav_list):
        uav_skills = set(uav['skills'])
        for j, task in enumerate(task_list):
            task_type = task['type']
            skills_a = set(task['skills_A'])
            skills_b = set(task['skills_B'])

            can_do_a = bool(uav_skills & skills_a)
            can_do_b = bool(uav_skills & skills_b)

            if can_do_a and can_do_b:
                match_matrix[i, j] = 3
            elif can_do_a:
                match_matrix[i, j] = 1
            elif can_do_b:
                match_matrix[i, j] = 2

    # 绘制热力图
    cmap = mcolors.ListedColormap(['#f0f0f0', TASK_COLORS['surveillance']['A'],
                                    TASK_COLORS['capture']['B'], '#2ecc71'])
    im = ax2.imshow(match_matrix, cmap=cmap, aspect='auto', vmin=0, vmax=3)

    # 设置坐标轴
    ax2.set_xlabel('Task ID', fontsize=12)
    ax2.set_ylabel('UAV ID', fontsize=12)
    ax2.set_title('(2) UAV-Task Capability Matrix\n'
                  'Green = Can do A+B phases, Gold = Only A phase, Cyan = Only B phase',
                  fontsize=14, fontweight='bold', pad=20)

    # 添加任务类型颜色条
    for j, task in enumerate(task_list):
        task_type = task['type']
        color = TASK_COLORS[task_type]['A']
        ax2.axvline(x=j-0.5, color=color, linewidth=2, alpha=0.3)

    # 添加颜色条
    cbar = plt.colorbar(im, ax=ax2, ticks=[0, 1, 2, 3], shrink=0.6)
    cbar.ax.set_yticklabels(['No Match', 'A Phase Only', 'B Phase Only', 'A+B Phases'])

    # 添加UAV类型分隔线
    uav_type_changes = []
    prev_type = None
    for i, uav in enumerate(uav_list):
        if uav['type'] != prev_type:
            uav_type_changes.append(i)
            prev_type = uav['type']

    for idx in uav_type_changes[1:]:  # 跳过第一个
        ax2.axhline(y=idx-0.5, color='red', linewidth=2, linestyle='--')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"UAV-Task capability map saved to: {output_path}")


def plot_detailed_uav_task_matching(data, output_path):
    """
    绘制详细的UAV-Task匹配图
    左侧展示UAV技能组合，右侧展示任务需求，中间用连线表示匹配关系
    """
    uav_list = data['uav_list']
    task_list = data['task_list']

    fig, ax = plt.subplots(figsize=(36, 24))

    # ===== 左侧: UAV技能圆环 =====
    uav_x_start = 2
    uav_y_start = 80
    uav_spacing = 1.0

    uav_positions = {}

    for i, uav in enumerate(uav_list):
        y = uav_y_start - i * uav_spacing
        x = uav_x_start
        uav_positions[uav['uav_id']] = (x, y)

        skills = sorted(uav['skills'])

        # 绘制双环 (上半圆=skill1, 下半圆=skill2)
        # 外环
        wedge1 = Wedge((x, y), 0.4, 0, 180, width=0.15,
                       facecolor=get_skill_color(skills[0]), edgecolor='black', linewidth=0.5)
        ax.add_patch(wedge1)

        # 内环
        wedge2 = Wedge((x, y), 0.25, 180, 360, width=0.15,
                       facecolor=get_skill_color(skills[1]), edgecolor='black', linewidth=0.5)
        ax.add_patch(wedge2)

        # UAV ID标签
        ax.text(x - 0.8, y, f"{uav['uav_id']}", ha='right', va='center', fontsize=7)

    # ===== 右侧: 任务圆圈 =====
    task_x_start = 32
    task_y_start = 75
    task_spacing = 2.5

    task_positions = {}

    for j, task in enumerate(task_list):
        y = task_y_start - (j % 15) * task_spacing
        x = task_x_start + (j // 15) * 4
        task_positions[task['task_id']] = (x, y)

        task_type = task['type']

        # 任务圆圈 (使用A阶段颜色)
        circle = Circle((x, y), 1.0,
                        facecolor=TASK_COLORS[task_type]['A'],
                        edgecolor='black', linewidth=1, alpha=0.6)
        ax.add_patch(circle)

        # 任务ID和类型
        ax.text(x, y, f"{task['task_id']}\n{task_type[:3]}", ha='center', va='center', fontsize=7, fontweight='bold')

        # 显示技能需求
        skills_a = task['skills_A']
        skills_b = task['skills_B']
        ax.text(x, y - 1.3, f"A:{skills_a} B:{skills_b}", ha='center', va='top', fontsize=6, color='gray')

    # ===== 中间: 匹配连线 =====
    # 为了避免过于拥挤，只绘制部分代表性连线
    # 选择每个任务的前几个匹配UAV

    for task in task_list:
        task_id = task['task_id']
        task_type = task['type']
        skills_a = set(task['skills_A'])
        skills_b = set(task['skills_B'])

        tx, ty = task_positions[task_id]

        # 找到匹配的UAV
        matched_uavs = []
        for uav in uav_list:
            uav_skills = set(uav['skills'])
            can_do_a = bool(uav_skills & skills_a)
            can_do_b = bool(uav_skills & skills_b)

            if can_do_a or can_do_b:
                matched_uavs.append((uav['uav_id'], can_do_a, can_do_b))

        # 只绘制前5个匹配的UAV连线（避免过于密集）
        for uav_id, can_do_a, can_do_b in matched_uavs[:5]:
            ux, uy = uav_positions[uav_id]

            # 根据匹配类型选择颜色
            if can_do_a and can_do_b:
                line_color = '#2ecc71'  # 绿色 - 完全匹配
                alpha = 0.4
            elif can_do_a:
                line_color = TASK_COLORS[task_type]['A']  # A阶段颜色
                alpha = 0.3
            else:
                line_color = TASK_COLORS[task_type]['B']  # B阶段颜色
                alpha = 0.3

            ax.plot([ux + 0.5, tx - 1.0], [uy, ty], color=line_color, alpha=alpha, linewidth=0.5)

    # 设置图表属性
    ax.set_xlim(-1, 40)
    ax.set_ylim(-5, 85)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('UAV-Task Capability Matching Diagram\n'
                 'Left: 80 UAVs with Dual Skills (Upper=Primary, Lower=Secondary)\n'
                 'Right: 30 Tasks with Skill Requirements\n'
                 'Lines show matching relationships (top 5 UAVs per task)',
                 fontsize=16, fontweight='bold', pad=20)

    # 添加图例
    legend_elements = [
        mpatches.Patch(facecolor=SKILL_COLORS[1], edgecolor='black', label='Skill 1 (Attack)'),
        mpatches.Patch(facecolor=SKILL_COLORS[2], edgecolor='black', label='Skill 2 (Surveillance/Capture A)'),
        mpatches.Patch(facecolor=SKILL_COLORS[3], edgecolor='black', label='Skill 3 (Surveillance/Capture B)'),
        mpatches.Patch(facecolor=TASK_COLORS['surveillance']['A'], edgecolor='black', alpha=0.6, label='Surveillance Task'),
        mpatches.Patch(facecolor=TASK_COLORS['attack']['A'], edgecolor='black', alpha=0.6, label='Attack Task'),
        mpatches.Patch(facecolor=TASK_COLORS['capture']['A'], edgecolor='black', alpha=0.6, label='Capture Task'),
    ]
    ax.legend(handles=legend_elements, loc='lower right', fontsize=10, ncol=2)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Detailed matching diagram saved to: {output_path}")


def plot_skill_distribution(data, output_path):
    """
    绘制UAV技能分布与任务需求匹配图
    使用堆叠环形图展示
    """
    uav_list = data['uav_list']
    task_list = data['task_list']

    fig, axes = plt.subplots(1, 3, figsize=(24, 8))

    # ===== 子图1: UAV类型分布 =====
    ax1 = axes[0]

    uav_types = {}
    for uav in uav_list:
        uav_type = uav['type']
        if uav_type not in uav_types:
            uav_types[uav_type] = []
        uav_types[uav_type].append(uav['uav_id'])

    type_colors_map = {'uavA': '#FF6B6B', 'uavB': '#4ECDC4', 'uavC': '#FFD700'}
    labels = list(uav_types.keys())
    sizes = [len(v) for v in uav_types.values()]
    colors = [type_colors_map[t] for t in labels]

    wedges, texts, autotexts = ax1.pie(sizes, labels=labels, colors=colors, autopct='%1.0f%%',
                                        startangle=90, textprops={'fontsize': 12})
    ax1.set_title('(1) UAV Type Distribution\n(80 UAVs total)', fontsize=14, fontweight='bold')

    # 添加技能标注
    skill_labels = ['uavA: skills [1,2]', 'uavB: skills [1,3]', 'uavC: skills [2,3]']
    ax1.legend(wedges, skill_labels, loc='lower center', fontsize=9)

    # ===== 子图2: 任务类型分布 =====
    ax2 = axes[1]

    task_types = {}
    for task in task_list:
        task_type = task['type']
        if task_type not in task_types:
            task_types[task_type] = 0
        task_types[task_type] += 1

    task_type_colors = {
        'surveillance': TASK_COLORS['surveillance']['A'],
        'attack': TASK_COLORS['attack']['A'],
        'capture': TASK_COLORS['capture']['A']
    }

    labels = list(task_types.keys())
    sizes = list(task_types.values())
    colors = [task_type_colors[t] for t in labels]

    wedges, texts, autotexts = ax2.pie(sizes, labels=labels, colors=colors, autopct='%1.0f%%',
                                        startangle=90, textprops={'fontsize': 12})
    ax2.set_title('(2) Task Type Distribution\n(30 Tasks total)', fontsize=14, fontweight='bold')

    # ===== 子图3: 技能供需对比 =====
    ax3 = axes[2]

    # 统计技能供给
    skill_supply = {1: 0, 2: 0, 3: 0}
    for uav in uav_list:
        for skill in uav['skills']:
            skill_supply[skill] += 1

    # 统计技能需求
    skill_demand = {1: 0, 2: 0, 3: 0}
    for task in task_list:
        for skill in task['skills_A']:
            skill_demand[skill] += task['targetA_num']
        for skill in task['skills_B']:
            skill_demand[skill] += task['targetB_num']

    x = np.arange(3)
    width = 0.35

    supply_vals = [skill_supply[1], skill_supply[2], skill_supply[3]]
    demand_vals = [skill_demand[1], skill_demand[2], skill_demand[3]]

    bars1 = ax3.bar(x - width/2, supply_vals, width, label='Supply (UAVs)', color='#3498db', alpha=0.7)
    bars2 = ax3.bar(x + width/2, demand_vals, width, label='Demand (Task slots)', color='#e74c3c', alpha=0.7)

    ax3.set_xlabel('Skill ID', fontsize=12)
    ax3.set_ylabel('Count', fontsize=12)
    ax3.set_title('(3) Skill Supply vs Demand', fontsize=14, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(['Skill 1\n(Attack)', 'Skill 2\n(Surveillance/Capture A)', 'Skill 3\n(Surveillance/Capture B)'])
    ax3.legend()
    ax3.grid(axis='y', alpha=0.3)

    # 添加数值标签
    for bar, val in zip(bars1, supply_vals):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height(), str(val),
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    for bar, val in zip(bars2, demand_vals):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height(), str(val),
                ha='center', va='bottom', fontsize=10, fontweight='bold')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Skill distribution chart saved to: {output_path}")


def main():
    print("=" * 60)
    print("UAV-Task Capability Visualization")
    print("=" * 60)

    # 加载数据
    data = load_data()
    print(f"Loaded {len(data['uav_list'])} UAVs and {len(data['task_list'])} tasks")

    # 输出目录
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 生成三张图
    print("\n[1/3] Generating UAV capability circles...")
    plot_uav_capability_circles(data, os.path.join(script_dir, 'uav_capability_circles.png'))

    print("\n[2/3] Generating detailed matching diagram...")
    plot_detailed_uav_task_matching(data, os.path.join(script_dir, 'uav_task_matching_detail.png'))

    print("\n[3/3] Generating skill distribution chart...")
    plot_skill_distribution(data, os.path.join(script_dir, 'skill_distribution.png'))

    print("\n" + "=" * 60)
    print("All visualizations completed!")
    print("=" * 60)


if __name__ == '__main__':
    main()
