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
        self.safety_distance = self.radius * 4  # 安全距离，提前避障

    def compute_cbf_risk(self, other_agents, obstacles):
        """
        计算CBF启发的风险度量
        """
        risk = 0.0

        # 与其他智能体的碰撞风险
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            dist = np.linalg.norm(self.p - other.p)
            if dist < self.safety_distance:
                risk += max(0, (self.safety_distance - dist) / self.safety_distance)

        # 与障碍物的碰撞风险
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]
            # 计算到障碍物的最短距离
            dist_to_ob = self._distance_to_obstacle(self.p, ob)
            if dist_to_ob < self.safety_distance:
                risk += max(0, (self.safety_distance - dist_to_ob) / self.safety_distance)

        return risk

    def _distance_to_obstacle(self, point, obstacle):
        """计算点到矩形障碍物的最短距离"""
        x, y, size = obstacle[0], obstacle[1], obstacle[2]
        # 矩形边界
        left, right = x, x + size
        bottom, top = y, y + size

        # 点到矩形的最短距离
        dx = max(left - point[0], 0, point[0] - right)
        dy = max(bottom - point[1], 0, point[1] - top)
        return np.sqrt(dx**2 + dy**2)

    def _get_obstacle_avoidance_direction(self, obstacle):
        """获取绕过障碍物的最佳方向"""
        x, y, size = obstacle[0], obstacle[1], obstacle[2]
        ob_center = np.array([x + size/2, y + size/2])

        # 从障碍物中心到智能体的方向
        to_agent = self.p - ob_center
        dist = np.linalg.norm(to_agent)

        if dist < 0.01:
            # 在障碍物中心，随机选一个方向
            return np.array([1.0, 0.0])

        # 计算绕行方向（垂直于到障碍物的方向）
        perpendicular = np.array([-to_agent[1], to_agent[0]]) / dist

        # 选择更靠近目标的方向
        to_target = self.target - self.p
        to_target_norm = to_target / (np.linalg.norm(to_target) + 0.01)

        if np.dot(perpendicular, to_target_norm) > 0:
            return perpendicular
        else:
            return -perpendicular

    def compute_velocity_command(self, other_agents, obstacles):
        """
        计算速度命令 - 改进的CBF避障策略
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
        desired_velocity = desired_direction * self.vmax

        # 初始化避障速度
        avoidance_velocity = np.zeros(2)

        # ===== 障碍物避障（优先级最高） =====
        for ob in obstacles:
            ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]
            dist_to_ob = self._distance_to_obstacle(self.p, ob)

            # 膨胀后的安全距离 - 增大检测范围
            safe_dist = self.radius * 3 + ob_size * 0.5

            if dist_to_ob < safe_dist * 5:  # 更远距离检测
                # 绕行方向
                avoid_dir = self._get_obstacle_avoidance_direction(ob)

                # 避障强度：距离越近越强
                if dist_to_ob < safe_dist:
                    strength = 3.0  # 非常近，强力避障
                elif dist_to_ob < safe_dist * 2:
                    strength = 2.0
                elif dist_to_ob < safe_dist * 3:
                    strength = 1.0
                else:
                    strength = (safe_dist * 5 - dist_to_ob) / safe_dist

                avoidance_velocity += avoid_dir * strength * self.vmax

                # 同时添加排斥力（远离障碍物）
                ob_center = np.array([ob_x + ob_size/2, ob_y + ob_size/2])
                to_agent = self.p - ob_center
                dist_center = np.linalg.norm(to_agent)
                if dist_center > 0.01:
                    repulsion = to_agent / dist_center * strength * 1.0
                    avoidance_velocity += repulsion

        # ===== 智能体避障 =====
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            diff = self.p - other.p
            dist = np.linalg.norm(diff)

            if dist < self.safety_distance * 2 and dist > 0.01:
                # 排斥方向
                repulsion_dir = diff / dist

                # 避障强度
                if dist < self.r_min:
                    strength = 2.0
                elif dist < self.r_min * 2:
                    strength = (self.r_min * 2 - dist) / self.r_min
                else:
                    strength = (self.safety_distance * 2 - dist) / self.safety_distance

                avoidance_velocity += repulsion_dir * strength * self.vmax

        # ===== 组合速度 =====
        risk = self.compute_cbf_risk(other_agents, obstacles)

        if risk > 0.1:
            # 有风险时，混合目标速度和避障速度
            avoidance_weight = min(0.8, risk * 0.6)  # 最大0.8，保证还有目标方向
            velocity_cmd = (1 - avoidance_weight) * desired_velocity + avoidance_weight * avoidance_velocity
        else:
            velocity_cmd = desired_velocity

        # 限制速度
        speed = np.linalg.norm(velocity_cmd)
        if speed > self.vmax:
            velocity_cmd = velocity_cmd / speed * self.vmax

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
