"""
CCPP (Coverage Path Planning) 主求解器
整合路径点生成和无人机分配算法
"""
import numpy as np

from .polygon_region import PolygonRegion
from .uav_assignment import UAVAssignment


class CCPPPlanner:
    """
    CCPP 平面覆盖扫描路径规划器

    功能:
    1. 给定多边形区域和扫描范围，生成蛇形走位路径
    2. 根据无人机位置，将路径合理分配给各无人机

    示例:
        planner = CCPPPlanner()

        # 设置参数
        polygon = [[0, 0], [100, 0], [100, 100], [0, 100]]
        uav_positions = [[0, 0], [100, 100]]
        scout_range = 10

        # 生成路径并分配
        assignments = planner.plan(polygon, uav_positions, scout_range)

        # assignments[0] 是第一架无人机的路径点
        # assignments[1] 是第二架无人机的路径点
    """

    def __init__(self):
        self.polygon_region = None
        self.assignments = None
        self.full_path = None

    def plan(self, polygon_vertices, uav_positions, scout_range):
        """
        执行完整的 CCPP 规划

        Args:
            polygon_vertices: 多边形顶点列表，格式为 [[x1, y1], [x2, y2], ...]
            uav_positions: 无人机位置列表，格式为 [[x1, y1], [x2, y2], ...]
            scout_range: 半个扫描范围（实际扫描宽度 = 2 * scout_range）

        Returns:
            assignments: 分配结果列表，每个元素是一个路径点列表
        """
        # 1. 创建多边形区域并生成扫描路径
        uav_num = len(uav_positions)
        self.polygon_region = PolygonRegion(
            polygon_vertices, scout_range, uav_num
        )

        # 2. 选择一个无人机起点作为入口点（这里选择第一个无人机的位置）
        self.polygon_region.update_start_point(uav_positions[0])

        # 3. 生成蛇形走位边界点
        self.polygon_region.initilize_boundpoint_list_edge()
        self.full_path = self.polygon_region.get_scan_path()

        # 4. 分配路径给各无人机
        self.assignments = UAVAssignment.assign(uav_positions, self.full_path)

        return self.assignments

    def get_full_path(self):
        """获取完整的扫描路径"""
        if self.full_path is None:
            return []
        return self.full_path

    def get_assignments(self):
        """获取无人机分配结果"""
        if self.assignments is None:
            return []
        return self.assignments

    def visualize(self, save_path=None):
        """
        可视化规划结果

        Args:
            save_path: 保存路径，如果为 None 则不保存
        """
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as MplPolygon

        fig, ax = plt.subplots(figsize=(10, 10))

        # 绘制多边形
        if self.polygon_region and self.polygon_region.vertices:
            poly_patch = MplPolygon(
                self.polygon_region.vertices,
                closed=True,
                fill=True,
                facecolor='lightblue',
                edgecolor='blue',
                alpha=0.3,
                linewidth=2
            )
            ax.add_patch(poly_patch)

            # 绘制顶点
            vertices = np.array(self.polygon_region.vertices)
            ax.plot(
                vertices[:, 0], vertices[:, 1],
                'bo-', linewidth=2, markersize=8, label='Polygon'
            )

        # 绘制完整路径
        if self.full_path:
            path = np.array(self.full_path)
            ax.plot(
                path[:, 0], path[:, 1],
                'g--', linewidth=1, alpha=0.5, label='Full Path'
            )

        # 绘制各无人机分配的路径
        colors = ['r', 'm', 'c', 'orange', 'purple']
        if self.assignments:
            for i, assignment in enumerate(self.assignments):
                if len(assignment) > 0:
                    path = np.array(assignment)
                    color = colors[i % len(colors)]
                    ax.plot(
                        path[:, 0], path[:, 1],
                        color=color, linewidth=2, marker='o',
                        markersize=4, label=f'UAV {i + 1}'
                    )

        # 标记起点
        if self.polygon_region and self.polygon_region.start_point is not None:
            ax.plot(
                self.polygon_region.start_point[0],
                self.polygon_region.start_point[1],
                'y*', markersize=15, label='Start Point'
            )

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('CCPP Coverage Path Planning')

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Visualization saved to: {save_path}")
        else:
            plt.show()

        plt.close()
