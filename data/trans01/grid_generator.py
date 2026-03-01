"""
网格位置生成器 - 用于生成不相撞的 agent 起始位置排列

功能：
1. 根据给定的中心点和数量，生成网格状排列的位置
2. 计算最小正方形边长
3. 可视化展示生成结果
"""

import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import json


def generate_grid_positions(center, num_agents, radius=1.0):
    """
    生成网格状不相撞的 agent 起始位置

    从左上角到右上角，一行一行往下填充

    Args:
        center: 中心点坐标 [cx, cy]
        num_agents: agent 数量
        radius: agent 半径（默认 1.0）

    Returns:
        list: 起始位置列表 [[x1, y1], [x2, y2], ...]

    计算：
        - 直径 = 2 * radius
        - 网格间距 = 直径
        - 网格尺寸 k = ceil(√num_agents)
        - 正方形边长 = (k - 1) * 网格间距
    """
    diameter = 2 * radius  # 中心间距
    k = math.ceil(math.sqrt(num_agents))  # 网格行/列数

    # 正方形边长（中心点之间的跨度）
    square_span = (k - 1) * diameter

    # 起始偏移（左上角）
    start_x = center[0] - square_span / 2
    start_y = center[1] + square_span / 2  # y 向上为正

    positions = []
    for i in range(num_agents):
        row = i // k
        col = i % k
        # 从左上角到右上角，一行一行往下
        x = start_x + col * diameter
        y = start_y - row * diameter  # y 向下
        positions.append([x, y])

    return positions


def plot_grid_positions(positions_list, centers=None, radius=1.0, title="网格位置分布", save_path=None):
    """
    可视化网格位置

    Args:
        positions_list: 位置列表的列表，每个元素是一组位置
                     例如: [[pos1_1, pos1_2, ...], [pos2_1, pos2_2, ...]]
        centers: 各组中心点列表，用于标记（可选）
        radius: agent 半径
        title: 图表标题
        save_path: 图片保存路径（可选）
    """
    fig, ax = plt.subplots(figsize=(12, 10))

    # 颜色列表
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'cyan', 'magenta']

    for group_idx, positions in enumerate(positions_list):
        color = colors[group_idx % len(colors)]

        # 绘制 agent 圆圈
        for pos in positions:
            circle = patches.Circle(pos, radius, facecolor=color, edgecolor='black', alpha=0.7)
            ax.add_patch(circle)

            # 标注位置索引
            ax.text(pos[0], pos[1], f'{pos[0]:.1f},{pos[1]:.1f}',
                   ha='center', va='center', fontsize=6, color='white')

        # 绘制边框矩形
        if positions:
            x_coords = [pos[0] for pos in positions]
            y_coords = [pos[1] for pos in positions]
            min_x, max_x = min(x_coords), max(x_coords)
            min_y, max_y = min(y_coords), max(y_coords)
            width = max_x - min_x + 2 * radius
            height = max_y - min_y + 2 * radius

            rect = patches.Rectangle((min_x - radius, min_y - radius), width, height,
                                 facecolor='none', edgecolor=color, linestyle='--', linewidth=2)
            ax.add_patch(rect)

            # 标注组信息
            group_center = [(min_x + max_x) / 2, (min_y + max_y) / 2]
            ax.text(group_center[0], group_center[1] + radius * 2,
                   f'Group {group_idx}: {len(positions)} agents',
                   ha='center', va='bottom', fontsize=10, color=color, fontweight='bold')

    # 绘制中心点标记
    if centers:
        for i, center in enumerate(centers):
            ax.plot(center[0], center[1], 'r*', markersize=20, label=f'Center {i}')

    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"图片已保存: {save_path}")
        plt.close()  # 关闭图形，不显示
    else:
        plt.show()


def calculate_grid_info(num_agents, radius=1.0):
    """
    计算网格排列的详细信息

    Args:
        num_agents: agent 数量
        radius: agent 半径

    Returns:
        dict: 包含网格信息的字典
    """
    diameter = 2 * radius
    k = math.ceil(math.sqrt(num_agents))
    square_span = (k - 1) * diameter
    grid_capacity = k * k

    return {
        'num_agents': num_agents,
        'radius': radius,
        'diameter': diameter,
        'grid_size': k,  # k x k
        'grid_capacity': grid_capacity,  # 总容量
        'square_span': square_span,  # 正方形边长（跨度）
        'unused_slots': grid_capacity - num_agents  # 未使用的格子数
    }


def save_positions_to_json(positions_list, output_path, description=""):
    """
    将位置数据保存为 JSON 文件

    Args:
        positions_list: 位置列表的列表
        output_path: 输出文件路径
        description: 描述信息
    """
    data = {
        'description': description,
        'groups': []
    }

    for i, positions in enumerate(positions_list):
        group_data = {
            'group_id': i,
            'num_agents': len(positions),
            'positions': positions
        }
        data['groups'].append(group_data)

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"位置数据已保存: {output_path}")


# ==================== 测试代码 ====================

if __name__ == '__main__':
    print("=" * 60)
    print("网格位置生成器测试")
    print("=" * 60)

    # 测试参数
    radius = 1.0
    num_agents_per_group = 40
    num_groups = 2
    total_agents = num_agents_per_group * num_groups

    print(f"\n参数设置:")
    print(f"  - agent 半径: {radius}")
    print(f"  - agent 直径: {2 * radius}")
    print(f"  - 每组 agent 数量: {num_agents_per_group}")
    print(f"  - 组数: {num_groups}")
    print(f"  - 总 agent 数量: {total_agents}")

    # 计算并显示网格信息
    print(f"\n" + "=" * 60)
    print("网格信息计算")
    print("=" * 60)
    info = calculate_grid_info(num_agents_per_group, radius)
    for key, value in info.items():
        print(f"  {key}: {value}")

    # 定义两组中心点
    center1 = [10, 15]
    center2 = [30, 15]

    print(f"\n" + "=" * 60)
    print("生成网格位置")
    print("=" * 60)

    # 生成第一组位置
    print(f"\n生成第 1 组位置，中心点: {center1}")
    positions1 = generate_grid_positions(center1, num_agents_per_group, radius)
    print(f"  生成了 {len(positions1)} 个位置")
    print(f"  范围: X[{min(p[0] for p in positions1):.2f}, {max(p[0] for p in positions1):.2f}], "
          f"Y[{min(p[1] for p in positions1):.2f}, {max(p[1] for p in positions1):.2f}]")

    # 生成第二组位置
    print(f"\n生成第 2 组位置，中心点: {center2}")
    positions2 = generate_grid_positions(center2, num_agents_per_group, radius)
    print(f"  生成了 {len(positions2)} 个位置")
    print(f"  范围: X[{min(p[0] for p in positions2):.2f}, {max(p[0] for p in positions2):.2f}], "
          f"Y[{min(p[1] for p in positions2):.2f}, {max(p[1] for p in positions2):.2f}]")

    all_positions = [positions1, positions2]

    # 保存到 JSON
    json_output = "data/trans/grid_positions.json"
    save_positions_to_json(
        all_positions,
        json_output,
        description=f"{total_agents} agents, 分成{num_groups}组，每组{num_agents_per_group}个"
    )

    # 绘制可视化
    print(f"\n" + "=" * 60)
    print("可视化")
    print("=" * 60)
    plot_grid_positions(
        all_positions,
        centers=[center1, center2],
        radius=radius,
        title=f"{total_agents} agents 网格分布（{num_groups} 组，每组 {num_agents_per_group}）",
        save_path="data/trans/grid_positions.png"
    )

    print("\n测试完成！")
