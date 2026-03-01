"""
CCPP 简单使用示例
演示如何使用 CCPP 规划器生成覆盖路径
"""
import os
import sys

import numpy as np

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ccpp import CCPPPlanner, PolygonRegion, UAVAssignment, LoadBalancer


def example_1_basic_usage():
    """示例 1: 基本使用"""
    print("=" * 60)
    print("示例 1: 基本使用")
    print("=" * 60)

    # 创建规划器
    planner = CCPPPlanner()

    # 定义多边形顶点 (矩形)
    polygon = [[0, 0], [100, 0], [100, 100], [0, 100]]

    # 定义无人机位置
    uav_positions = [[0, 0], [100, 100]]

    # 定义扫描范围（半个范围，实际扫描宽度 = 2 * scout_range）
    scout_range = 10

    # 执行规划
    assignments = planner.plan(polygon, uav_positions, scout_range)

    # 输出结果
    print(f"多边形顶点数: {len(polygon)}")
    print(f"无人机数量: {len(uav_positions)}")
    print(f"扫描范围: {scout_range}")
    print(f"生成的路径点总数: {sum(len(a) for a in assignments)}")

    for i, path in enumerate(assignments):
        print(f"\nUAV {i + 1} 路径 ({len(path)} 个点):")
        for j, point in enumerate(path):
            print(f"  点 {j:2d}: ({point[0]:7.2f}, {point[1]:7.2f})")

    # 可视化
    output_path = os.path.join(
        os.path.dirname(__file__), "test_outputs", "example_1.png"
    )
    planner.visualize(save_path=output_path)
    print(f"\n可视化结果已保存到: {output_path}")
    print()


def example_2_complex_polygon():
    """示例 2: 复杂多边形"""
    print("=" * 60)
    print("示例 2: 复杂多边形")
    print("=" * 60)

    planner = CCPPPlanner()

    # 定义复杂多边形 (六边形)
    polygon = [
        [0, 50],
        [50, 100],
        [100, 50],
        [100, -50],
        [50, -100],
        [0, -50]
    ]

    # 定义无人机位置
    uav_positions = [
        [0, 0],
        [50, 50],
        [100, 0]
    ]

    # 执行规划
    scout_range = 15
    assignments = planner.plan(polygon, uav_positions, scout_range)

    # 输出结果
    print(f"多边形顶点数: {len(polygon)}")
    print(f"无人机数量: {len(uav_positions)}")
    print(f"扫描范围: {scout_range}")

    for i, path in enumerate(assignments):
        print(f"\nUAV {i + 1}: {len(path)} 个路径点")

    # 可视化
    output_path = os.path.join(
        os.path.dirname(__file__), "test_outputs", "example_2.png"
    )
    planner.visualize(save_path=output_path)
    print(f"\n可视化结果已保存到: {output_path}")
    print()


def example_3_single_uav():
    """示例 3: 单架无人机"""
    print("=" * 60)
    print("示例 3: 单架无人机")
    print("=" * 60)

    planner = CCPPPlanner()

    # 定义三角形
    polygon = [[0, 0], [100, 0], [50, 86.6]]

    # 单架无人机
    uav_positions = [[50, 30]]

    # 执行规划
    scout_range = 8
    assignments = planner.plan(polygon, uav_positions, scout_range)

    # 输出结果
    print(f"多边形顶点数: {len(polygon)}")
    print(f"无人机数量: {len(uav_positions)}")
    print(f"扫描范围: {scout_range}")
    print(f"路径点数: {len(assignments[0]) if assignments else 0}")

    # 可视化
    output_path = os.path.join(
        os.path.dirname(__file__), "test_outputs", "example_3.png"
    )
    planner.visualize(save_path=output_path)
    print(f"\n可视化结果已保存到: {output_path}")
    print()


def example_4_path_generation_only():
    """示例 4: 仅使用路径生成"""
    print("=" * 60)
    print("示例 4: 仅使用路径生成")
    print("=" * 60)

    # 创建多边形区域
    polygon = [[0, 0], [100, 0], [100, 100], [0, 100]]
    region = PolygonRegion(polygon, scout_range=10, uav_num=2)

    # 设置起点
    region.update_start_point([0, 0])

    # 生成路径
    region.initilize_boundpoint_list_edge()

    # 获取路径点
    path = region.get_scan_path()

    print(f"多边形顶点数: {len(polygon)}")
    print(f"生成的路径点数: {len(path)}")
    print("\n路径点:")
    for i, point in enumerate(path):
        print(f"  点 {i:2d}: ({point[0]:7.2f}, {point[1]:7.2f})")
    print()


def example_5_assignment_only():
    """示例 5: 仅使用无人机分配"""
    print("=" * 60)
    print("示例 5: 仅使用无人机分配")
    print("=" * 60)

    # 定义边界点（已生成的扫描路径，需要转换为 numpy 数组）
    bound_points = np.array([
        [0, 10], [100, 10],
        [100, 30], [0, 30],
        [0, 50], [100, 50],
        [100, 70], [0, 70]
    ])

    # 定义无人机位置
    uav_positions = [[0, 0], [100, 100]]

    # 执行分配
    assignments = UAVAssignment.assign(uav_positions, bound_points)

    print(f"边界点总数: {len(bound_points)}")
    print(f"无人机数量: {len(uav_positions)}")

    for i, path in enumerate(assignments):
        print(f"\nUAV {i + 1} 分配的路径点数: {len(path)}")
        print(f"  起始点: ({path[0][0]:7.2f}, {path[0][1]:7.2f})")
        print(f"  终止点: ({path[-1][0]:7.2f}, {path[-1][1]:7.2f})")
    print()


def example_6_load_balancer():
    """示例 6: 负载均衡算法"""
    print("=" * 60)
    print("示例 6: 负载均衡算法")
    print("=" * 60)

    # 分配无人机到多个区域
    n_drone = 10
    m_region = 5
    region_sizes = [20, 15, 30, 10, 25]

    assignments = LoadBalancer.assign_drones(n_drone, m_region, region_sizes)

    print(f"总无人机数: {n_drone}")
    print(f"区域数量: {m_region}")
    print(f"区域大小: {region_sizes}")
    print(f"\n分配结果:")
    total = 0
    for i, count in enumerate(assignments):
        total += count
        percentage = count / n_drone * 100
        size_percentage = region_sizes[i] / sum(region_sizes) * 100
        print(f"  区域 {i + 1}: {count:2d} 架无人机 "
              f"(占比 {percentage:5.1f}%, 区域大小占比 {size_percentage:5.1f}%)")
    print(f"  总计: {total} 架无人机")
    print()


def main():
    """运行所有示例"""
    print("\n" + "=" * 60)
    print("CCPP 平面覆盖扫描路径规划 - 使用示例")
    print("=" * 60 + "\n")

    # 运行示例
    example_1_basic_usage()
    example_2_complex_polygon()
    example_3_single_uav()
    example_4_path_generation_only()
    example_5_assignment_only()
    example_6_load_balancer()

    print("=" * 60)
    print("所有示例运行完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
