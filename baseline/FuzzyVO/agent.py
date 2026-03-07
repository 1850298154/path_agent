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
        self.lookahead_dist = self.radius * 8  # 前瞻距离

    def _distance_to_obstacle(self, point, obstacle):
        """计算点到矩形障碍物的最短距离"""
        x, y, size = obstacle[0], obstacle[1], obstacle[2]
        left, right = x, x + size
        bottom, top = y, y + size

        dx = max(left - point[0], 0, point[0] - right)
        dy = max(bottom - point[1], 0, point[1] - top)
        return np.sqrt(dx**2 + dy**2)

    def _point_to_obstacle_direction(self, point, obstacle):
        """计算从点到障碍物最近点的方向"""
        x, y, size = obstacle[0], obstacle[1], obstacle[2]
        left, right = x, x + size
        bottom, top = y, y + size

        # 找到矩形上最近的点
        closest_x = np.clip(point[0], left, right)
        closest_y = np.clip(point[1], bottom, top)

        direction = np.array([closest_x - point[0], closest_y - point[1]])
        dist = np.linalg.norm(direction)
        if dist > 0.01:
            return direction / dist
        else:
            return np.array([1.0, 0.0])

    def compute_velocity_obstacle(self, other_agents, obstacles):
        """
        计算速度障碍物(VO) - 改进版
        返回避障信息列表
        """
        vo_list = []

        # 与其他智能体的VO
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            rel_pos = other.p - self.p
            dist = np.linalg.norm(rel_pos)

            if dist < self.lookahead_dist and dist > 0.01:
                direction = rel_pos / dist
                vo_list.append({
                    'direction': direction,
                    'distance': dist,
                    'radius': self.r_min,
                    'type': 'agent'
                })

        # 与障碍物的VO
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]

            dist_to_ob = self._distance_to_obstacle(self.p, ob)
            direction_to_ob = self._point_to_obstacle_direction(self.p, ob)

            # 膨胀半径 = 原始障碍物对角线一半 + 安全半径
            inflated_radius = ob_size * 0.707 + self.radius * 2

            if dist_to_ob < inflated_radius + self.lookahead_dist:
                vo_list.append({
                    'direction': direction_to_ob,
                    'distance': dist_to_ob,
                    'radius': inflated_radius,
                    'type': 'obstacle'
                })

        return vo_list

    def fuzzy_avoidance(self, desired_velocity, vo_list):
        """
        使用模糊规则调整速度以避免碰撞 - 改进版
        """
        velocity = desired_velocity.copy()
        speed = np.linalg.norm(velocity)

        if speed < 0.01:
            speed = self.vmax
            velocity = np.array([self.vmax, 0])

        # 模糊规则参数 - 增加检测距离
        danger_dist = self.radius * 5
        caution_dist = self.radius * 10
        safe_dist = self.radius * 15

        for vo in vo_list:
            dist = vo['distance']
            direction = vo['direction']
            vo_type = vo['type']

            # 计算危险程度 (0-1)
            if dist < danger_dist:
                danger = 1.0
            elif dist < caution_dist:
                danger = (caution_dist - dist) / (caution_dist - danger_dist)
            elif dist < safe_dist:
                danger = (safe_dist - dist) / (safe_dist - caution_dist) * 0.5
            else:
                danger = 0.0

            if danger > 0.05:
                # 避障方向：远离 + 绕行
                away_dir = -direction
                perpendicular = np.array([-direction[1], direction[0]])

                to_target = self.target - self.p
                to_target_norm = to_target / (np.linalg.norm(to_target) + 0.01)

                if np.dot(perpendicular, to_target_norm) > 0:
                    bypass_dir = perpendicular
                else:
                    bypass_dir = -perpendicular

                # 障碍物避障更强
                avoid_speed = speed * danger * (3.0 if vo_type == 'obstacle' else 1.5)
                avoid_velocity = away_dir * avoid_speed * 0.5 + bypass_dir * avoid_speed * 0.5

                velocity += avoid_velocity

        return velocity

    def compute_velocity_command(self, other_agents, obstacles):
        """
        计算速度命令 - 改进的FuzzyVO方法
        """
        start_time = time.time()

        to_target = self.target - self.p
        dist_to_target = np.linalg.norm(to_target)

        if dist_to_target < self.radius:
            self.reached_target = True
            self.v = np.zeros(2)
            self.plan_time_list.append(time.time() - start_time)
            return np.zeros(2)

        desired_velocity = to_target / dist_to_target * self.vmax
        vo_list = self.compute_velocity_obstacle(other_agents, obstacles)
        velocity = self.fuzzy_avoidance(desired_velocity, vo_list)

        speed = np.linalg.norm(velocity)
        if speed > self.vmax:
            velocity = velocity / speed * self.vmax
        elif speed < 0.01:
            velocity = desired_velocity

        self.plan_time_list.append(time.time() - start_time)
        return velocity

    def update(self, velocity_cmd):
        """更新位置和速度"""
        if self.damaged or self.reached_target:
            self.trajectory.append(self.p.copy())
            self.pre_traj_list.append(np.array([self.p, self.p]))
            return

        speed = np.linalg.norm(velocity_cmd)
        if speed > self.vmax:
            velocity_cmd = velocity_cmd / speed * self.vmax

        self.v = velocity_cmd
        self.p = self.p + self.v * self.h

        self.p[0] = np.clip(self.p[0], 0, self.config.map_xlim)
        self.p[1] = np.clip(self.p[1], 0, self.config.map_ylim)

        self.trajectory.append(self.p.copy())
        self.pre_traj_list.append(np.array([self.p.copy(), self.p.copy() + self.v * self.h]))

    def check_internal_collision(self, other):
        """检查内部碰撞"""
        if self.damaged or other.damaged:
            return False
        dist = np.linalg.norm(self.p - other.p)
        return dist < self.r_min

    def check_external_collision(self, obstacles):
        """检查与障碍物的外部碰撞"""
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]
            left = ob_x - self.radius
            right = ob_x + ob_size + self.radius
            bottom = ob_y - self.radius
            top = ob_y + ob_size + self.radius

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
