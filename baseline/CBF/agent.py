# CBF基线算法 - 智能体类
import numpy as np
import time

class CBFAgent:
    """基于CBF启发的简化智能体类"""

    def __init__(self, index, start_pos, target_pos, config):
        self.index = index
        self.config = config

        # 位置和速度
        self.p = np.array(start_pos, dtype=float)  # 当前位置
        self.v = np.zeros(2, dtype=float)           # 当前速度
        self.target = np.array(target_pos, dtype=float)

        # 初始位置（记录用）
        self.start_pos = np.array(start_pos, dtype=float)

        # 参数
        self.vmax = config.vmax
        self.umax = config.umax
        self.h = config.h
        self.radius = config.radius
        self.physical_radius = config.physical_radius
        self.r_min = 2 * self.radius  # 直径

        # 状态标记
        self.reached_target = False   # 是否到达目标
        self.collision = False        # 是否发生碰撞（内部）
        self.ex_collision = False     # 是否发生外部碰撞
        self.damaged = False          # 是否损毁

        # 轨迹记录
        self.trajectory = [self.p.copy()]
        self.pre_traj_list = []       # 用于可视化

        # 规划时间记录
        self.plan_time_list = []

        # CBF相关参数
        self.risk_threshold = 2.0     # 风险阈值
        self.safety_margin = 1.5      # 安全边际系数

    def compute_cbf_risk(self, other_agents, obstacles):
        """
        计算CBF启发的风险度量
        简化版本：基于距离的风险评估
        """
        risk = 0.0

        # 与其他智能体的碰撞风险
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            dist = np.linalg.norm(self.p - other.p)
            if dist < self.r_min * self.safety_margin:
                # 距离越近风险越大
                risk += max(0, (self.r_min * self.safety_margin - dist) / self.r_min)

        # 与障碍物的碰撞风险
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]
            # 障碍物中心（ob_x, ob_y是左下角坐标）
            ob_center = np.array([ob_x + ob_size/2, ob_y + ob_size/2])
            half_size = ob_size / 2 + self.radius  # 膨胀后的半边长

            # 简化：用圆近似障碍物
            dist_to_center = np.linalg.norm(self.p - ob_center)
            effective_radius = half_size * 1.414  # 正方形对角线一半

            if dist_to_center < effective_radius:
                risk += (effective_radius - dist_to_center) / self.radius

        return risk

    def compute_velocity_command(self, other_agents, obstacles):
        """
        计算速度命令 - 简化的CBF避障策略
        """
        start_time = time.time()

        # 目标方向
        to_target = self.target - self.p
        dist_to_target = np.linalg.norm(to_target)

        # 检查是否到达目标
        if dist_to_target < self.radius:
            self.reached_target = True
            self.v = np.zeros(2)
            self.plan_time_list.append(time.time() - start_time)
            return np.zeros(2)

        # 理想速度方向（指向目标）
        desired_direction = to_target / dist_to_target

        # 计算避障调整
        avoidance_velocity = np.zeros(2)

        # 与其他智能体的避障
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            diff = self.p - other.p
            dist = np.linalg.norm(diff)

            if dist < self.r_min * 2 and dist > 0.01:
                # 计算排斥速度
                repulsion_strength = (self.r_min * 2 - dist) / (self.r_min * 2)
                avoidance_velocity += repulsion_strength * (diff / dist)

        # 与障碍物的避障
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]
            # 障碍物中心（ob_x, ob_y是左下角坐标）
            ob_center = np.array([ob_x + ob_size/2, ob_y + ob_size/2])

            # 膨胀后的障碍物边界
            inflated_half_size = ob_size / 2 + self.radius * 2

            diff = self.p - ob_center
            dist = np.linalg.norm(diff)

            # 计算到矩形的最短距离（简化用圆）
            effective_radius = inflated_half_size * 0.7

            if dist < effective_radius and dist > 0.01:
                repulsion_strength = (effective_radius - dist) / effective_radius
                avoidance_velocity += repulsion_strength * (diff / dist) * 2

        # 组合速度命令
        # 风险越大，避障权重越高
        risk = self.compute_cbf_risk(other_agents, obstacles)
        avoidance_weight = min(1.0, risk * 0.5)

        velocity_cmd = (1 - avoidance_weight) * desired_direction + avoidance_weight * avoidance_velocity

        # 归一化并限制速度
        if np.linalg.norm(velocity_cmd) > 0.01:
            velocity_cmd = velocity_cmd / np.linalg.norm(velocity_cmd) * self.vmax

        # 规划时间
        self.plan_time_list.append(time.time() - start_time)

        return velocity_cmd

    def update(self, velocity_cmd):
        """更新位置和速度"""
        if self.damaged or self.reached_target:
            self.trajectory.append(self.p.copy())
            self.pre_traj_list.append(np.array([self.p, self.p]))
            return

        # 限制速度
        speed = np.linalg.norm(velocity_cmd)
        if speed > self.vmax:
            velocity_cmd = velocity_cmd / speed * self.vmax

        # 更新速度
        self.v = velocity_cmd

        # 更新位置
        self.p = self.p + self.v * self.h

        # 边界检查
        self.p[0] = np.clip(self.p[0], 0, self.config.map_xlim)
        self.p[1] = np.clip(self.p[1], 0, self.config.map_ylim)

        # 记录轨迹
        self.trajectory.append(self.p.copy())
        self.pre_traj_list.append(np.array([self.p.copy(), self.p.copy() + self.v * self.h]))

    def check_internal_collision(self, other):
        """检查与另一个智能体的内部碰撞"""
        if self.damaged or other.damaged:
            return False
        dist = np.linalg.norm(self.p - other.p)
        return dist < self.r_min

    def check_external_collision(self, obstacles):
        """
        检查与障碍物的外部碰撞
        障碍物格式: (x, y, size) - 左下角坐标 + 边长（正方形）
        膨胀方式: 四边向外扩展 radius
        """
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]

            # 障碍物膨胀后的边界（左下角向外扩展radius）
            left = ob_x - self.radius
            right = ob_x + ob_size + self.radius
            bottom = ob_y - self.radius
            top = ob_y + ob_size + self.radius

            # 检查点是否在膨胀后的正方形内
            if left <= self.p[0] <= right and bottom <= self.p[1] <= top:
                return True
        return False

    def mark_damaged(self):
        """标记为损毁"""
        self.damaged = True
        self.v = np.zeros(2)

    def get_distance_to_target(self):
        """获取到目标的距离"""
        return np.linalg.norm(self.p - self.target)
