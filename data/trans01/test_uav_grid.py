"""
从 UAV 原始数据读取初始点，生成网格分布并可视化
"""

import json
import sys
import os

# 添加父目录到路径以便导入 grid_generator
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from grid_generator import generate_grid_positions, plot_grid_positions, save_positions_to_json


def load_uav_init_positions(json_path):
    """
    从 UAV 数据文件中读取初始点并统计

    Returns:
        centers: 初始点列表
        counts: 每个初始点对应的 UAV 数量
        uav_ids_by_center: 每个初始点对应的 UAV ID 列表
    """
    with open(json_path, 'r') as f:
        data = json.load(f)

    uavs = data['uavs']

    # 统计初始点
    init_dict = {}
    for uav_id, uav_data in uavs.items():
        init_pos = tuple(uav_data['init_pos'])
        if init_pos not in init_dict:
            init_dict[init_pos] = []
        init_dict[init_pos].append(int(uav_id))

    centers = []
    counts = []
    uav_ids_by_center = []

    for i, (pos, ids) in enumerate(init_dict.items()):
        centers.append(list(pos))
        counts.append(len(ids))
        uav_ids_by_center.append(sorted(ids))
        print(f'初始点 {i+1}: {pos} - {len(ids)} 个 UAV (IDs: {sorted(ids)[:5]}...)')

    print()
    return centers, counts, uav_ids_by_center


def main():
    print("=" * 70)
    print("UAV 网格分布生成器")
    print("=" * 70)
    print()

    # 原始数据路径
    json_path = '../original_data/uav_positions_over_time.json'

    # 加载并统计初始点
    centers, counts, uav_ids_by_center = load_uav_init_positions(json_path)

    # agent 半径（缩小一半）
    radius = 0.5

    # 生成网格位置
    print("=" * 70)
    print("生成网格位置")
    print("=" * 70)
    print()

    positions_list = []
    all_uav_ids = []

    for i, (center, count, uav_ids) in enumerate(zip(centers, counts, uav_ids_by_center)):
        print(f'生成第 {i+1} 组:')
        print(f'  原始中心点: {center}')
        print(f'  UAV 数量: {count}')

        positions = generate_grid_positions(center, count, radius)

        print(f'  生成位置数: {len(positions)}')
        print(f'  X 范围: [{min(p[0] for p in positions):.2f}, {max(p[0] for p in positions):.2f}]')
        print(f'  Y 范围: [{min(p[1] for p in positions):.2f}, {max(p[1] for p in positions):.2f}]')
        print()

        positions_list.append(positions)
        all_uav_ids.extend(uav_ids)

    # 保存结果
    output_json = 'uav_grid_positions.json'
    description = f"从 UAV 数据生成，共 {len(all_uav_ids)} 个 UAV，{len(centers)} 个初始点"

    save_positions_to_json(
        positions_list,
        output_json,
        description=description
    )

    # 可视化
    print("=" * 70)
    print("可视化")
    print("=" * 70)
    print()

    title = f"UAV 网格分布 ({len(all_uav_ids)} UAV, {len(centers)} 组)"
    save_img = 'uav_grid_positions.png'

    plot_grid_positions(
        positions_list,
        centers=centers,
        radius=radius,
        title=title,
        save_path=save_img
    )

    print()
    print("=" * 70)
    print(f"完成！")
    print(f"  JSON: {output_json}")
    print(f"  图片: {save_img}")
    print("=" * 70)


if __name__ == '__main__':
    main()
