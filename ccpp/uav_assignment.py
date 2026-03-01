"""
无人机分配算法
"""
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


class UAVAssignment:
    """
    无人机分配算法，将扫描路径合理分配给多架无人机

    算法流程:
    1. 将扫描路径按任务段划分
    2. 计算每段的工作量（距离）
    3. 平均分配任务段给各无人机
    4. 使用匈牙利算法优化无人机-任务分配
    """

    @staticmethod
    def divide_path_into_segments(bound_points):
        """
        将边界点划分为任务段

        蛇形走位模式:
        _______
        |1  2  5  6|
        | n  _       |
        |0  3  4  7|
        ￣￣￣￣￣￣

        分段规则:
        - 每段包含3个直线段 (n-n-1-n-2)
        - 或1个直线段 (_)

        Args:
            bound_points: 边界点列表

        Returns:
            edge_pts: 分段起点的索引列表
        """
        # 分成多个任务段，每次的起点
        edge_pts = [0]
        i = 0

        while i < len(bound_points):
            i += 3
            if i < len(bound_points):
                edge_pts.append(i)
            i += 1
            if i < len(bound_points):
                edge_pts.append(i)

        # 确保包含最后一个点
        if edge_pts[-1] + 1 != len(bound_points):
            edge_pts.append(len(bound_points) - 1)

        return edge_pts

    @staticmethod
    def calculate_workload(bound_points, edge_pts):
        """
        计算每段的工作量（距离）

        Args:
            bound_points: 边界点列表
            edge_pts: 分段起点的索引列表

        Returns:
            workload: 每段的工作量列表
            workload_cumsum: 累积工作量列表
        """
        workload = [0]

        for i in range(len(edge_pts) - 1):
            s_idx, e_idx = edge_pts[i], edge_pts[i + 1]
            d = 0
            for j in range(s_idx, e_idx):
                d += np.linalg.norm(bound_points[j + 1] - bound_points[j])
            workload.append(d)

        workload_cumsum = np.cumsum(workload)

        return workload, workload_cumsum

    @staticmethod
    def assign_segments_to_uavs(uav_num, edge_pts):
        """
        将任务段分配给各无人机（平均分配）

        Args:
            uav_num: 无人机数量
            edge_pts: 分段起点的索引列表

        Returns:
            split_idx: 分割点索引列表
        """
        # uav_num <= pts_num
        basic_seg_n = (len(edge_pts) - 1) // uav_num
        reminder = (len(edge_pts) - 1) % uav_num

        # 初始化分割点选择
        split_idx = [0]

        for i in range(uav_num):
            if i < reminder:
                # 前 reminder 个无人机多分配一个余数
                split_idx.append(split_idx[-1] + basic_seg_n + 1)
            else:
                split_idx.append(split_idx[-1] + basic_seg_n)

        return split_idx

    @staticmethod
    def calculate_segment_distances(workload_cumsum, split_idx):
        """
        计算每个分割段的距离

        Args:
            workload_cumsum: 累积工作量列表
            split_idx: 分割点索引列表

        Returns:
            split_dis: 每个分割段的距离列表
        """
        split_dis = []

        for i in range(len(split_idx) - 1):
            split_dis.append(
                workload_cumsum[split_idx[i + 1]] - workload_cumsum[split_idx[i]]
            )

        return split_dis

    @staticmethod
    def optimize_assignment(uav_positions, edge_pts, split_idx, split_dis,
                         bound_points):
        """
        使用匈牙利算法优化无人机-任务分配

        Args:
            uav_positions: 无人机位置列表 [(x1, y1), (x2, y2), ...]
            edge_pts: 分段起点的索引列表
            split_idx: 分割点索引列表
            split_dis: 每个分割段的距离列表
            bound_points: 边界点列表

        Returns:
            task_idx: 每个无人机分配的任务段索引
        """
        uav_num = len(uav_positions)

        # 构建代价矩阵
        # cost[i, j] = 无人机 i 到任务段 j 的起点的距离 + 任务段 j 的距离
        cost = np.zeros((uav_num, uav_num))

        for i, pos in enumerate(uav_positions):
            for j in range(uav_num):
                total_dis = np.linalg.norm(
                    np.array(pos) - bound_points[edge_pts[split_idx[j]]]
                ) + split_dis[j]
                cost[i, j] = total_dis

        # 使用匈牙利算法进行分配
        _, task_idx = linear_sum_assignment(cost)

        return task_idx

    @staticmethod
    def assign(uav_positions, bound_points, uav_num=None):
        """
        完整的无人机分配流程

        Args:
            uav_positions: 无人机位置列表 [(x1, y1), (x2, y2), ...]
            bound_points: 边界点列表
            uav_num: 无人机数量（如果不指定则使用 len(uav_positions)）

        Returns:
            assignments: 分配结果列表，每个元素是一个列表，表示分配给该无人机的路径点
        """
        if uav_num is None:
            uav_num = len(uav_positions)

        if uav_num == 0:
            return []

        # 1. 将路径划分为任务段
        edge_pts = UAVAssignment.divide_path_into_segments(bound_points)
        pts_num = len(edge_pts)

        # 如果无人机数量大于任务段数，简单平均分配点
        if uav_num > pts_num:
            # 简单分配：每个无人机分配一些连续的点
            assignments = []
            points_per_uav = len(bound_points) // uav_num
            remainder = len(bound_points) % uav_num

            start = 0
            for i in range(uav_num):
                num_points = points_per_uav + (1 if i < remainder else 0)
                assignments.append(bound_points[start:start + num_points])
                start += num_points

            return assignments

        # 2. 计算每段的工作量
        workload, workload_cumsum = UAVAssignment.calculate_workload(
            bound_points, edge_pts
        )

        # 3. 将任务段分配给各无人机
        split_idx = UAVAssignment.assign_segments_to_uavs(uav_num, edge_pts)

        # 4. 计算每个分割段的距离
        split_dis = UAVAssignment.calculate_segment_distances(
            workload_cumsum, split_idx
        )

        # 5. 使用匈牙利算法优化分配
        task_idx = UAVAssignment.optimize_assignment(
            uav_positions, edge_pts, split_idx, split_dis, bound_points
        )

        # 6. 构建分配结果
        assignments = []
        for i, j in enumerate(task_idx):
            s_idx = edge_pts[split_idx[j]]
            e_idx = edge_pts[split_idx[j + 1]] if j + 1 < len(
                split_idx
            ) else len(bound_points)
            assignments.append(bound_points[s_idx:e_idx + 1])

        return assignments


class LoadBalancer:
    """负载均衡器，基于区域面积分配无人机"""

    @staticmethod
    def assign_drones(n_drone, m_region, region_sizes):
        """
        将 n 架无人机分配到 m 个区域，根据区域大小分配

        Args:
            n_drone: 无人机数量
            m_region: 区域数量
            region_sizes: 各区域的大小列表

        Returns:
            drones_assigned: 每个区域分配的无人机数量列表

        Raises:
            ValueError: 输入参数无效
        """
        if all(x <= 0 for x in region_sizes):
            raise ValueError("Invalid input: region_sizes all <= 0")
        if n_drone <= 0 or m_region <= 0 or n_drone < m_region:
            raise ValueError(
                "Invalid input: n must be greater than m and both must be positive integers."
            )

        # 初始化分配列表
        drones_assigned = [0] * m_region

        # 计算总大小
        total_size = sum(region_sizes)

        # 计算每个区域的百分比
        percentage_of_area = [
            region_sizes[i] / total_size for i in range(m_region)
        ]

        # 计算每个区域应该分配的无人机数量
        for i in range(m_region):
            drones_assigned[i] = int(
                round(n_drone * percentage_of_area[i])
            )

        # 调整分配，确保恰好分配 n 架无人机
        while sum(drones_assigned) != n_drone:
            if sum(drones_assigned) < n_drone:
                # 给分配数量最多的区域增加一架无人机
                max_index = max(
                    [i for i in range(m_region) if drones_assigned[i] > 0],
                    key=lambda x: drones_assigned[x]
                )
                drones_assigned[max_index] += 1
            elif sum(drones_assigned) > n_drone:
                # 从分配数量最少且有无人机的区域减少一架无人机
                min_index = min(
                    [i for i in range(m_region) if drones_assigned[i] > 0],
                    key=lambda x: drones_assigned[x]
                )
                drones_assigned[min_index] -= 1

        return drones_assigned
