"""
多边形区域类，用于生成蛇形走位边界点
"""
import numpy as np
from math import floor

from .line_segment import LineSeg


class PolygonRegion:
    """
    多边形区域类，用于生成蛇形走位边界点

    算法流程:
    1. 找到离起点最近的边作为入口边
    2. 选择相邻边中截距跨度最小的边作为平行边
    3. 按扫描宽度生成平行线段
    4. 计算所有平行线与多边形的交点
    5. 调整点的顺序形成蛇形走位路径
    """

    def __init__(self, vertices, scout_range, uav_num):
        """
        初始化多边形区域

        Args:
            vertices: 多边形顶点列表，格式为 [[x1, y1], [x2, y2], ...]
            scout_range: 半个扫描范围（实际扫描范围 = 2 * scout_range）
            uav_num: 无人机数量
        """
        self.vertices = vertices
        self.scout_range = scout_range
        self.uav_num = uav_num
        self.start_point = None
        self.bound_points = []
        self.line_pts = []

    def update_start_point(self, start_point):
        """更新起点坐标"""
        self.start_point = np.array(start_point)

    def calc_nearest_seg_idx(self):
        """
        计算距离起点最近的线段下标

        Returns:
            nearest_seg_idx: 最近线段的下标
        """
        points = self.vertices
        linesegs = [
            LineSeg(points[i], points[i + 1])
            if i + 1 < len(points)
            else LineSeg(points[i], points[0])
            for i in range(len(points))
        ]

        # 找到最近的线段
        dist_to_start_point = [
            lineseg.distance_to_point(self.start_point) for lineseg in linesegs
        ]

        # 当最小值不止一个时，需要进一步判断选择哪个线段
        min_d = np.min(dist_to_start_point)
        min_indexes = np.argwhere(dist_to_start_point == min_d).flatten()

        if min_indexes.size == 1:
            nearest_seg_idx = min_indexes[0]
        else:
            # 当最小值不止一个时，需要进一步判断
            # 此时为点 p 到相邻的两条线段的最小距离为公共顶点
            alpha_list = [
                linesegs[idx].calc_alpha(self.start_point)
                for idx in min_indexes
            ]
            a_list = []
            for alpha in alpha_list:
                if alpha >= 1:
                    a_list.append(alpha - 1)
                elif alpha <= 0:
                    a_list.append(-alpha)
                else:
                    a_list.append(alpha)
            nearest_seg_idx = min_indexes[np.argmin(a_list)]

        return nearest_seg_idx

    def initilize_boundpoint_list_edge(self):
        """
        选择入口边作为最近的边，返回蛇形走位的边界点

        算法:
        1. 找到离起点最近的边作为入口边
        2. 选择相邻边中截距跨度最小的边作为平行边
        3. 按扫描宽度生成平行线段
        4. 计算所有平行线与多边形的交点
        5. 调整点的顺序形成蛇形走位路径
        """
        # 扫描宽度 = 2 * scout_range
        D = self.scout_range * 2
        points = self.vertices
        linesegs = [
            LineSeg(points[i], points[i + 1])
            if i + 1 < len(points)
            else LineSeg(points[i], points[0])
            for i in range(len(points))
        ]

        # 选择最近的边
        nearest_seg_idx = self.calc_nearest_seg_idx()

        # 选择相邻边
        idx = [(nearest_seg_idx - 1) % len(linesegs),
                (nearest_seg_idx + 1) % len(linesegs)]

        # 选择相邻边中截距跨度最小的边作为平行边
        intercept_ranges0 = [
            lineseg.intercept_range(linesegs[idx[0]].m) for lineseg in linesegs
        ]
        intercept_ranges1 = [
            lineseg.intercept_range(linesegs[idx[1]].m) for lineseg in linesegs
        ]

        intercept_ranges = [
            np.max(intercept_ranges0) - np.min(intercept_ranges0),
            np.max(intercept_ranges1) - np.min(intercept_ranges1)
        ]
        selected_seg = linesegs[idx[np.argmin(intercept_ranges)]]
        m = selected_seg.m  # 选择平行边的斜率

        # 计算所有边的截距范围
        intercept_ranges = [lineseg.intercept_range(m) for lineseg in linesegs]
        max_intercept = np.max(intercept_ranges)
        min_intercept = np.min(intercept_ranges)

        # 计算平行线的间隔
        if m is None:
            spacing = D
        else:
            spacing = D / np.cos(np.arctan(m))

        # 生成平行线的截距
        intercepts = np.arange(min_intercept + spacing, max_intercept, spacing)

        # 计算每条平行线与多边形的交点
        line_pts = [
            [
                lineseg.intersect_w_line(m, intercept)
                for lineseg in linesegs
                if lineseg.intersect_w_line(m, intercept)[0] is not None
            ]
            for intercept in intercepts
        ]

        # 如果截距刚好在顶点处，会被加入两次，需要剔除重复的
        refined_pts = []
        for pts in line_pts:
            if len(pts) == 2:
                refined_pts.append(pts)
            else:
                line_p = []
                for idx_p, p in enumerate(pts):
                    if idx_p == 0:
                        line_p.append(p)
                    elif p != line_p[-1]:
                        line_p.append(p)
                if len(line_p) == 2:
                    refined_pts.append(line_p)
                # 如果线段长度为0，跳过

        # 构造蛇形走位边界点
        bound_points = []

        for idx, pts in enumerate(refined_pts):
            if idx % 2 == 0:
                bound_points.append(np.array(pts[0]))
                bound_points.append(np.array(pts[1]))
            else:
                bound_points.append(np.array(pts[1]))
                bound_points.append(np.array(pts[0]))

        self.line_pts = line_pts
        self.bound_points = bound_points
        self.adjust_points_order()

    def adjust_points_order(self):
        """
        调整边界点的顺序，确保从最近的入口点开始

        蛇形走位模式:
        _______
        |1  2  5  6|
        |           |
        |0  3  4  7|
        ￣￣￣￣￣￣
        [0, 1, 2, 3, 4, 5, 6, 7]
        """
        if len(self.bound_points) < 4:
            print(f'Warning: len(bound_points) = {len(self.bound_points)}')
            return

        # 获取四个关键点
        key_points = [
            self.bound_points[0],
            self.bound_points[1],
            self.bound_points[-1],
            self.bound_points[-2]
        ]
        pos = self.start_point

        # 计算四个关键点到起点的距离
        dis_list = [np.linalg.norm(p - pos) for p in key_points]
        min_idx = np.argmin(np.array(dis_list))

        if min_idx == 0:
            # 已经是正确的顺序
            return
        elif min_idx == 1:
            # 对称翻转，因为是从第二个开始
            flip = True
            reverse = False
        elif min_idx == 2:
            # 从蛇尾巴开始
            flip = False
            reverse = True
        elif min_idx == 3:
            flip = True
            reverse = True

        # 对称反转
        if flip:
            bound_points = []
            for idx in range(len(self.bound_points)):
                if idx % 2 == 0:
                    bound_points.append(self.bound_points[idx + 1])
                    bound_points.append(self.bound_points[idx])
            self.bound_points = bound_points

        # 蛇尾开始逆序
        if reverse:
            self.bound_points = self.bound_points[::-1]

    def get_scan_path(self):
        """
        获取完整的扫描路径点

        Returns:
            path_points: 扫描路径点列表
        """
        return self.bound_points
