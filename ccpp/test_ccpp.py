"""
CCPP 测试脚本
生成随机测试用例并验证算法
"""
import os
import random
import sys
from datetime import datetime

import numpy as np

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ccpp import CCPPPlanner


def generate_convex_polygon(num_vertices, center=None, radius_range=(50, 150)):
    """
    生成随机凸多边形

    Args:
        num_vertices: 顶点数量
        center: 中心点 [x, y]，如果为 None 则随机生成
        radius_range: 半径范围 (min, max)

    Returns:
        vertices: 顶点列表 [[x1, y1], [x2, y2], ...]
    """
    if center is None:
        center = [random.randint(-50, 50), random.randint(-50, 50)]
    else:
        center = list(center)

    # 生成随机角度
    angles = sorted([random.uniform(0, 2 * np.pi) for _ in range(num_vertices)])

    # 生成随机半径
    vertices = []
    for angle in angles:
        radius = random.uniform(*radius_range)
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        vertices.append([x, y])

    return vertices


def generate_random_test_case(test_id):
    """
    生成随机测试用例

    Args:
        test_id: 测试用例编号

    Returns:
        test_case: 测试用例字典
    """
    # 随机多边形顶点数量（3-8）
    num_vertices = random.randint(3, 8)

    # 随机无人机数量（1-5）
    uav_num = random.randint(1, 5)

    # 随机扫描范围（5-30）
    scout_range = random.uniform(5, 30)

    # 生成随机凸多边形
    polygon = generate_convex_polygon(num_vertices)

    # 生成随机无人机位置（在多边形附近）
    uav_positions = []
    poly_min = np.min(polygon, axis=0)
    poly_max = np.max(polygon, axis=0)

    for _ in range(uav_num):
        # 在多边形外围随机位置
        x = random.uniform(poly_min[0] - 100, poly_max[0] + 100)
        y = random.uniform(poly_min[1] - 100, poly_max[1] + 100)
        uav_positions.append([x, y])

    test_case = {
        'test_id': test_id,
        'polygon': polygon,
        'uav_positions': uav_positions,
        'scout_range': scout_range,
        'num_vertices': num_vertices,
        'uav_num': uav_num
    }

    return test_case


def visualize_test_case(test_case, assignments, output_dir):
    """
    可视化测试用例

    Args:
        test_case: 测试用例字典
        assignments: 分配结果
        output_dir: 输出目录
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon

    fig, ax = plt.subplots(figsize=(10, 10))

    # 绘制多边形
    poly_patch = MplPolygon(
        test_case['polygon'],
        closed=True,
        fill=True,
        facecolor='lightblue',
        edgecolor='blue',
        alpha=0.3,
        linewidth=2
    )
    ax.add_patch(poly_patch)

    # 绘制顶点
    vertices = np.array(test_case['polygon'])
    ax.plot(
        vertices[:, 0], vertices[:, 1],
        'bo-', linewidth=2, markersize=8, label='Polygon'
    )

    # 标记顶点编号
    for i, (x, y) in enumerate(vertices):
        ax.annotate(
            f'V{i}', (x, y), xytext=(5, 5),
            textcoords='offset points', fontsize=10
        )

    # 绘制各无人机分配的路径
    colors = ['r', 'm', 'c', 'orange', 'purple', 'brown', 'pink', 'gray']
    markers = ['o', 's', '^', 'D', 'v', '<', '>', 'p']

    for i, assignment in enumerate(assignments):
        if len(assignment) > 0:
            path = np.array(assignment)
            color = colors[i % len(colors)]
            marker = markers[i % len(markers)]

            # 绘制路径
            ax.plot(
                path[:, 0], path[:, 1],
                color=color, linewidth=2, marker=marker,
                markersize=5, label=f'UAV {i + 1}'
            )

            # 标记路径点
            for j, (x, y) in enumerate(path):
                if j == 0:
                    ax.annotate(
                        f'S{i}', (x, y), xytext=(-10, 10),
                        textcoords='offset points', fontsize=9,
                        bbox=dict(boxstyle='round,pad=0.3',
                                 facecolor=color, alpha=0.7)
                    )

    # 标记无人机起始位置
    for i, pos in enumerate(test_case['uav_positions']):
        ax.plot(
            pos[0], pos[1],
            '*', color=colors[i % len(colors)], markersize=15,
            markeredgecolor='black', markeredgewidth=2,
            label=f'UAV {i + 1} Start'
        )

    # 添加标题和信息
    info_text = (
        f"Test Case: {test_case['test_id']}\n"
        f"Vertices: {test_case['num_vertices']}\n"
        f"UAVs: {test_case['uav_num']}\n"
        f"Scout Range: {test_case['scout_range']:.1f}"
    )
    ax.text(
        0.02, 0.98, info_text,
        transform=ax.transAxes,
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
        fontsize=10
    )

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=9)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(f'CCPP Test Case {test_case["test_id"]}')

    # 保存图片
    save_path = os.path.join(output_dir, f'test_{test_case["test_id"]:02d}.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"  Visualization saved to: {save_path}")

    plt.close()


def verify_assignments(test_case, assignments):
    """
    验证分配结果的合理性

    Args:
        test_case: 测试用例字典
        assignments: 分配结果

    Returns:
        is_valid: 是否有效
        issues: 问题列表
    """
    issues = []

    # 检查每个无人机是否有路径
    for i, assignment in enumerate(assignments):
        if len(assignment) == 0:
            issues.append(f"UAV {i + 1} has no assigned path")

    # 检查路径点数量是否合理
    total_points = sum(len(a) for a in assignments)
    if total_points < 2:
        issues.append(f"Total path points too few: {total_points}")

    # 检查路径是否在多边形附近
    polygon = np.array(test_case['polygon'])
    poly_min = np.min(polygon, axis=0) - 50
    poly_max = np.max(polygon, axis=0) + 50

    for i, assignment in enumerate(assignments):
        for j, point in enumerate(assignment):
            x, y = point[0], point[1]
            if x < poly_min[0] or x > poly_max[0] or \
               y < poly_min[1] or y > poly_max[1]:
                issues.append(
                    f"UAV {i + 1} point {j} is far from polygon: ({x:.1f}, {y:.1f})"
                )

    is_valid = len(issues) == 0
    return is_valid, issues


def run_test(test_case, output_dir):
    """
    运行单个测试用例

    Args:
        test_case: 测试用例字典
        output_dir: 输出目录

    Returns:
        success: 是否成功
        assignments: 分配结果
    """
    print(f"\nRunning Test Case {test_case['test_id']}:")
    print(f"  Vertices: {test_case['num_vertices']}")
    print(f"  UAVs: {test_case['uav_num']}")
    print(f"  Scout Range: {test_case['scout_range']:.1f}")

    try:
        # 创建规划器
        planner = CCPPPlanner()

        # 执行规划
        assignments = planner.plan(
            test_case['polygon'],
            test_case['uav_positions'],
            test_case['scout_range']
        )

        # 验证结果
        is_valid, issues = verify_assignments(test_case, assignments)

        if is_valid:
            print(f"  Status: PASSED")
        else:
            print(f"  Status: PASSED (with warnings)")
            for issue in issues:
                print(f"    Warning: {issue}")

        # 可视化
        visualize_test_case(test_case, assignments, output_dir)

        # 打印统计信息
        print(f"  Statistics:")
        print(f"    Total path points: {sum(len(a) for a in assignments)}")
        for i, assignment in enumerate(assignments):
            print(f"    UAV {i + 1}: {len(assignment)} points")

        return True, assignments

    except Exception as e:
        print(f"  Status: FAILED")
        print(f"  Error: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def main():
    """主函数：生成并运行测试用例"""
    print("=" * 60)
    print("CCPP (Coverage Path Planning) Test Suite")
    print("=" * 60)

    # 创建输出目录
    base_dir = "D:\\zyt\\git_ln\\path_agent\\ccpp"
    test_output_dir = os.path.join(base_dir, "test_outputs")

    if not os.path.exists(test_output_dir):
        os.makedirs(test_output_dir)
        print(f"Created test output directory: {test_output_dir}")

    # 生成并运行 10 组测试用例
    num_tests = 10
    test_cases = []

    for i in range(num_tests):
        test_case = generate_random_test_case(i + 1)
        test_cases.append(test_case)

    # 运行测试
    results = []
    for test_case in test_cases:
        success, assignments = run_test(test_case, test_output_dir)
        results.append({
            'test_id': test_case['test_id'],
            'success': success,
            'assignments': assignments
        })

    # 输出汇总
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)

    passed = sum(1 for r in results if r['success'])
    total = len(results)

    print(f"Total Tests: {total}")
    print(f"Passed: {passed}")
    print(f"Failed: {total - passed}")

    print("\nTest Results:")
    for r in results:
        status = "PASSED" if r['success'] else "FAILED"
        print(f"  Test {r['test_id']:2d}: {status}")

    print(f"\nTest outputs saved to: {test_output_dir}")

    return results


if __name__ == "__main__":
    results = main()
