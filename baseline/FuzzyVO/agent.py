# FuzzyVO基线算法 - 智能体类
import numpy as np
import time

class FuzzyVOAgent:
    """基于Fuzzy Rules + Velocity Obstacles的简化智能体类"""

    def __init__(self, index, start_pos, target_pos, config):
        self.index = index
        self.config = config

        # 位置和速度
        self.p = np.array(start_pos, dtype=float)
        self.v = np.zeros(2, dtype=float)
        self.target = np.array(target_pos, dtype=float)
        self.start_pos = np.array(start_pos, dtype=float)

        # 参数
        self.vmax = config.vmax
        self.umax = config.umax
        self.h = config.h
        self.radius = config.radius
        self.physical_radius = config.physical_radius
        self.r_min = 2 * self.radius

        # 状态标记
        self.reached_target = False
        self.collision = False
        self.ex_collision = False
        self.damaged = False

        # 轨迹记录
        self.trajectory = [self.p.copy()]
        self.pre_traj_list = []
        self.plan_time_list = []

        # FuzzyVO参数
        self.lookahead_dist = 5.0  # 前瞻距离

    def compute_velocity_obstacle(self, other_agents, obstacles):
        """
        计算速度障碍物(VO)
        返回会导致碰撞的速度区域
        """
        vo_velocities = []

        # 与其他智能体的VO
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            rel_pos = other.p - self.p
            dist = np.linalg.norm(rel_pos)

            if dist < self.r_min * 3 and dist > 0.01:
                # VO: 相对速度指向对方的区域
                rel_vel = other.v - self.v
                # 简化：将对方看作圆形障碍物
                vo_radius = self.r_min

                # 计算VO圆锥的顶点方向
                if dist > 0.01:
                    direction = rel_pos / dist
                    # 计算会导致碰撞的速度范围
                    max_avoid_speed = dist / self.h - abs(np.linalg.norm(rel_vel))
                    vo_velocities.append((direction, vo_radius, max_avoid_speed))

        # 与障碍物的VO
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]
            # 障碍物中心（ob_x, ob_y是左下角坐标）
            ob_center = np.array([ob_x + ob_size/2, ob_y + ob_size/2])

            rel_pos = ob_center - self.p
            dist = np.linalg.norm(rel_pos)

            # 膨胀后的障碍物半径（正方形对角线一半）
            inflated_radius = ob_size * 0.707 + self.radius  # 0.707 = sqrt(2)/2

            if dist < inflated_radius * 2 and dist > 0.01:
                direction = rel_pos / dist
                vo_velocities.append((direction, inflated_radius, dist))

        return vo_velocities

    def fuzzy_avoidance(self, desired_velocity, vo_velocities):
        """
        使用模糊规则调整速度以避免碰撞
        简化的模糊规则实现
        """
        velocity = desired_velocity.copy()
        speed = np.linalg.norm(velocity)

        # 模糊规则参数
        danger_threshold = self.r_min * 2
        caution_threshold = self.r_min * 4

        for direction, obstacle_radius, distance in vo_velocities:
            # 计算危险程度 (0-1)
            if distance < danger_threshold:
                danger = 1.0
            elif distance < caution_threshold:
                danger = (caution_threshold - distance) / (caution_threshold - danger_threshold)
            else:
                danger = 0.0

            if danger > 0.1:
                # 计算避障速度
                avoid_dir = -direction  # 远离障碍物
                avoid_speed = speed * danger * 1.5

                # 简单模糊规则：危险越大，避障越强
                velocity += avoid_dir * avoid_speed

        return velocity

    def compute_velocity_command(self, other_agents, obstacles):
        """
        计算速度命令 - FuzzyVO方法
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

        # 理想速度（指向目标）
        desired_velocity = to_target / dist_to_target * self.vmax

        # 计算速度障碍物
        vo_velocities = self.compute_velocity_obstacle(other_agents, obstacles)

        # 使用模糊规则避障
        velocity = self.fuzzy_avoidance(desired_velocity, vo_velocities)

        # 限制速度
        speed = np.linalg.norm(velocity)
        if speed > self.vmax:
            velocity = velocity / speed * self.vmax

        # 记录规划时间
        self.plan_time_list.append(time.time() - start_time)

        return velocity

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
        """检查内部碰撞"""
        if self.damaged or other.damaged:
            return False
        dist = np.linalg.norm(self.p - other.p)
        return dist < self.r_min

    def check_external_collision(self, obstacles):
        """
        检查与障碍物的外部碰撞
        障碍物格式: (x, y, size)，其中(x,y)是左下角坐标
        膨胀后的障碍物边界：左下角向外扩展radius
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
