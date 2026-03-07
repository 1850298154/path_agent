# CBF基线算法 - 智能体类
import numpy as np
import time

class CBFAgent:
    """基于CBF启发的简化智能体类"""

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

        # CBF参数 - 参考FuzzyVO成功的参数
        self.lookahead_dist = self.radius * 15  # 前瞻距离
        self.safety_distance = self.radius * 10

    def _distance_to_obstacle(self, point, obstacle):
        """计算点到膨胀后障碍物的最短距离"""
        x, y, size = obstacle[0], obstacle[1], obstacle[2]
        # 膨胀后的边界
        left = x - self.radius
        right = x + size + self.radius
        bottom = y - self.radius
        top = y + size + self.radius

        dx = max(left - point[0], 0, point[0] - right)
        dy = max(bottom - point[1], 0, point[1] - top)
        return np.sqrt(dx**2 + dy**2)

    def _point_to_obstacle_direction(self, point, obstacle):
        """计算从点到障碍物最近点的方向"""
        x, y, size = obstacle[0], obstacle[1], obstacle[2]
        left, right = x, x + size
        bottom, top = y, y + size

        closest_x = np.clip(point[0], left, right)
        closest_y = np.clip(point[1], bottom, top)

        direction = np.array([closest_x - point[0], closest_y - point[1]])
        dist = np.linalg.norm(direction)
        if dist > 0.01:
            return direction / dist
        else:
            return np.array([1.0, 0.0])

    def compute_cbf_risk(self, other_agents, obstacles):
        """计算CBF风险度量"""
        risk = 0.0

        # 智能体风险
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue
            dist = np.linalg.norm(self.p - other.p)
            if dist < self.safety_distance:
                risk += (self.safety_distance - dist) / self.safety_distance

        # 障碍物风险
        for ob in obstacles:
            dist_to_ob = self._distance_to_obstacle(self.p, ob)
            if dist_to_ob < self.safety_distance:
                risk += (self.safety_distance - dist_to_ob) / self.safety_distance

        return risk

    def compute_velocity_command(self, other_agents, obstacles):
        """计算速度命令 - 参考FuzzyVO的避障逻辑"""
        start_time = time.time()

        # 目标方向
        to_target = self.target - self.p
        dist_to_target = np.linalg.norm(to_target)

        if dist_to_target < self.radius:
            self.reached_target = True
            self.v = np.zeros(2)
            self.plan_time_list.append(time.time() - start_time)
            return np.zeros(2)

        desired_velocity = to_target / dist_to_target * self.vmax

        # 初始化避障速度
        avoidance_velocity = np.zeros(2)

        # ===== 障碍物避障（与FuzzyVO相同的参数）=====
        danger_dist = self.radius * 5
        caution_dist = self.radius * 10
        safe_dist = self.radius * 15

        for ob in obstacles:
            dist_to_ob = self._distance_to_obstacle(self.p, ob)
            direction_to_ob = self._point_to_obstacle_direction(self.p, ob)

            if dist_to_ob < safe_dist:
                # 危险程度
                if dist_to_ob < danger_dist:
                    danger = 1.0
                elif dist_to_ob < caution_dist:
                    danger = (caution_dist - dist_to_ob) / (caution_dist - danger_dist)
                else:
                    danger = (safe_dist - dist_to_ob) / (safe_dist - caution_dist) * 0.5

                if danger > 0.05:
                    # 远离方向
                    away_dir = -direction_to_ob
                    # 绕行方向
                    perpendicular = np.array([-direction_to_ob[1], direction_to_ob[0]])

                    to_target_norm = to_target / dist_to_target
                    if np.dot(perpendicular, to_target_norm) > 0:
                        bypass_dir = perpendicular
                    else:
                        bypass_dir = -perpendicular

                    # 避障强度（障碍物更强）
                    avoid_speed = self.vmax * danger * 3.0
                    avoid_velocity = away_dir * avoid_speed * 0.5 + bypass_dir * avoid_speed * 0.5

                    avoidance_velocity += avoid_velocity

        # ===== 智能体避障 =====
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            diff = self.p - other.p
            dist = np.linalg.norm(diff)

            if dist < self.lookahead_dist and dist > 0.01:
                direction = diff / dist

                # 危险程度
                if dist < self.r_min * 2:
                    danger = 1.0
                elif dist < self.r_min * 4:
                    danger = (self.r_min * 4 - dist) / (self.r_min * 2)
                elif dist < self.lookahead_dist:
                    danger = (self.lookahead_dist - dist) / self.lookahead_dist * 0.3
                else:
                    danger = 0.0

                if danger > 0.05:
                    avoid_speed = self.vmax * danger * 1.5
                    avoidance_velocity += direction * avoid_speed

        # ===== 组合速度 =====
        risk = self.compute_cbf_risk(other_agents, obstacles)

        if risk > 0.05 and np.linalg.norm(avoidance_velocity) > 0.01:
            avoidance_weight = min(0.85, risk * 0.7)
            velocity_cmd = desired_velocity * (1 - avoidance_weight) + avoidance_velocity * avoidance_weight
        else:
            velocity_cmd = desired_velocity

        # 限制速度
        speed = np.linalg.norm(velocity_cmd)
        if speed > self.vmax:
            velocity_cmd = velocity_cmd / speed * self.vmax

        self.plan_time_list.append(time.time() - start_time)
        return velocity_cmd

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
        """检查外部碰撞"""
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
