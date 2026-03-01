"""
UAV 初始位置转换脚本

功能：
1. 从 01_original_data 读取 uav_positions_over_time.json
2. 提取所有 UAV 的 init_pos
3. 统计初始点分布
4. 使用网格生成器生成新的起始位置
5. 输出到 02_processed_data
"""

import json
import os
import sys

# 添加父目录到路径以导入 grid_generator
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from grid_generator import generate_grid_positions, calculate_grid_info


def load_uav_data(input_path):
    """加载 UAV 位置数据"""
    with open(input_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def extract_init_positions(uav_data):
    """
    提取所有 UAV 的初始位置并统计

    Returns:
        init_dict: {init_pos_tuple: [uav_id, uav_id, ...]}
        init_list: [(uav_id, init_pos), ...]  # 按 ID 排序
    """
    uavs = uav_data['uavs']
    init_dict = {}
    init_list = []

    for uav_id_str, uav_data in uavs.items():
        uav_id = int(uav_id_str)
        init_pos = tuple(uav_data['init_pos'])

        if init_pos not in init_dict:
            init_dict[init_pos] = []
        init_dict[init_pos].append(uav_id)
        init_list.append((uav_id, init_pos))

    # 按 UAV ID 排序
    init_list.sort(key=lambda x: x[0])

    return init_dict, init_list


def generate_new_positions(init_dict, radius=0.5):
    """
    根据初始点分布生成新的网格位置

    Args:
        init_dict: {init_pos: [uav_ids]}
        radius: agent 半径

    Returns:
        new_positions_by_uav: {uav_id: new_pos}
        groups_info: 各组生成的信息
    """
    new_positions_by_uav = {}
    groups_info = []

    for group_idx, (init_pos, uav_ids) in enumerate(init_dict.items()):
        num_uavs = len(uav_ids)

        # 计算网格信息
        info = calculate_grid_info(num_uavs, radius)

        # 生成网格位置
        grid_positions = generate_grid_positions(list(init_pos), num_uavs, radius)

        # 为每个 UAV 分配新位置
        for uav_id, new_pos in zip(uav_ids, grid_positions):
            new_positions_by_uav[uav_id] = list(new_pos)

        # 记录组信息
        group_info = {
            'group_id': group_idx,
            'original_center': list(init_pos),
            'num_uavs': num_uavs,
            'grid_info': info,
            'uav_ids': sorted(uav_ids),
            'positions_range': {
                'x_min': min(p[0] for p in grid_positions),
                'x_max': max(p[0] for p in grid_positions),
                'y_min': min(p[1] for p in grid_positions),
                'y_max': max(p[1] for p in grid_positions)
            }
        }
        groups_info.append(group_info)

        print(f'\n组 {group_idx + 1}:')
        print(f'  原始中心: {init_pos}')
        print(f'  UAV 数量: {num_uavs}')
        print(f'  UAV IDs: {sorted(uav_ids)}')
        print(f'  网格: {info["grid_size"]}x{info["grid_size"]}')
        print(f'  边长: {info["square_span"]:.2f}')
        print(f'  范围: X[{group_info["positions_range"]["x_min"]:.2f}, {group_info["positions_range"]["x_max"]:.2f}], '
              f'Y[{group_info["positions_range"]["y_min"]:.2f}, {group_info["positions_range"]["y_max"]:.2f}]')

    return new_positions_by_uav, groups_info


def save_processed_data(uav_data, new_positions_by_uav, groups_info, output_path):
    """
    保存处理后的 UAV 数据

    保持原有结构，只替换 init_pos
    """
    processed_data = {
        'time_step': uav_data['time_step'],
        'makespan': uav_data['makespan'],
        'uavs': {}
    }

    # 替换 init_pos，保持 positions 不变
    for uav_id_str, uav_data in uav_data['uavs'].items():
        uav_id = int(uav_id_str)
        new_init_pos = new_positions_by_uav.get(uav_id, uav_data['init_pos'])

        processed_data['uavs'][uav_id_str] = {
            'init_pos': new_init_pos,
            'positions': uav_data['positions']
        }

    # 添加转换信息
    processed_data['_transform_info'] = {
        'description': 'UAV 初始位置从单点转换为网格分布',
        'radius': 0.5,
        'groups': groups_info
    }

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(processed_data, f, indent=2, ensure_ascii=False)

    print(f'\n处理后的数据已保存: {output_path}')


def main():
    print("=" * 70)
    print("UAV 初始位置转换工具")
    print("=" * 70)

    # 输入输出路径
    input_path = '../01_original_data/uav_positions_over_time.json'
    output_path = '../02_processed_data/uav_positions_transformed.json'

    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # 加载数据
    print(f'\n加载数据: {input_path}')
    uav_data = load_uav_data(input_path)
    print(f"总 UAV 数: {len(uav_data['uavs'])}")
    print(f"时间步长: {uav_data['time_step']}")
    print(f"总时间: {uav_data['makespan']}")

    # 提取初始位置
    print('\n' + '=' * 70)
    print("提取初始位置")
    print('=' * 70)
    init_dict, init_list = extract_init_positions(uav_data)

    print(f'\n不同的初始点数: {len(init_dict)}')
    for i, (init_pos, uav_ids) in enumerate(init_dict.items()):
        print(f'  初始点 {i+1}: {init_pos} - {len(uav_ids)} 个 UAV')

    # 生成新位置
    print('\n' + '=' * 70)
    print("生成网格位置")
    print('=' * 70)
    radius = 0.5
    new_positions_by_uav, groups_info = generate_new_positions(init_dict, radius)

    # 保存处理后的数据
    print('\n' + '=' * 70)
    print("保存处理后的数据")
    print('=' * 70)
    save_processed_data(uav_data, new_positions_by_uav, groups_info, output_path)

    print('\n' + '=' * 70)
    print("完成！")
    print('=' * 70)
    print(f'\n输入: {input_path}')
    print(f'输出: {output_path}')


if __name__ == '__main__':
    main()
