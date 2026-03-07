# CBF基线算法 - 智能体类（完整论文实现）
import numpy as np
import time

class CBFAgent:
    """基于CBF论文的完整实现，包含加速度约束和死锁避免

    论文: Adaptive Deadlock Avoidance for Decentralized Multi-agent Systems
          via CBF-inspired Risk Measurement
    """

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
        self.umax = config.umax  # 最大加速度
        self.h = config.h
        self.radius = config.radius
        self.physical_radius = config.physical_radius
        self.r_min = 2 * self.radius

        # 状态标记
        self.reached_target = False
        self.collision = False
        self.ex_collision = False
        self.damaged = False
        self.in_deadlock = False
        self.deadlock_counter = 0

        # 轨迹记录
        self.trajectory = [self.p.copy()]
        self.pre_traj_list = []
        self.plan_time_list = []

        # CBF参数
        self.lookahead_dist = self.radius * 15
        self.safety_distance = self.radius * 10

        # 死锁指示函数参数 (Eq. 10)
        self.zeta_t = 5.0
        self.zeta_c = 0.5

        # CBF增益
        self.alpha = 1.0
        self.gamma = 1.0

        # 旋转角度和方向（用于死锁解决）
        self.rotation_angle = 0.0
        self.rotation_direction = 1

        # 速度历史
        self.velocity_history = []
        self.position_history = [self.p.copy()]

    def _distance_to_obstacle(self, point, obstacle):
        """计算点到膨胀后障碍物的最短距离"""
        x, y, size = obstacle[0], obstacle[1], obstacle[2]
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

    def compute_cbf_h(self, other_agent):
        """CBF安全函数 h_ij(x) = ||x_i - x_j||^2 - (r_i + r_j)^2"""
        diff = self.p - other_agent.p
        dist_sq = np.dot(diff, diff)
        r_sum_sq = (self.radius + other_agent.radius) ** 2
        return dist_sq - r_sum_sq

    def compute_cbf_h_dot(self, other_agent):
        """CBF安全函数导数"""
        diff = self.p - other_agent.p
        rel_vel = self.v - other_agent.v
        return 2 * np.dot(diff, rel_vel)

    def compute_cbf_risk(self, other_agents, obstacles):
        """计算CBF风险度量 R_i"""
        risk = 0.0
        count = 0

        # 智能体风险
        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            dist = np.linalg.norm(self.p - other.p)
            if dist < self.safety_distance:
                h = self.compute_cbf_h(other)
                h_dot = self.compute_cbf_h_dot(other)
                risk += max(0, (-h_dot - self.alpha * h))
                count += 1

        # 障碍物风险
        for ob in obstacles:
            dist_to_ob = self._distance_to_obstacle(self.p, ob)
            if dist_to_ob < self.safety_distance:
                # 简化：将障碍物风险转换为距离风险
                risk += (self.safety_distance - dist_to_ob) / self.safety_distance
                count += 1

        if count > 0:
            risk = risk / count

        phi = 0.1
        return max(0.0, risk + phi)

    def compute_deadlock_indicator(self, risk):
        """死锁指示函数 ζ_i(R_i) = 1 / (1 + e^{-t(R_i - c)})"""
        zeta = 1.0 / (1.0 + np.exp(-self.zeta_t * (risk - self.zeta_c)))
        return zeta

    def detect_deadlock(self):
        """检测死锁"""
        if len(self.velocity_history) >= 15:
            recent_velocities = self.velocity_history[-15:]
            avg_speed = np.mean([np.linalg.norm(v) for v in recent_velocities])

            # 计算到目标距离变化
            dist_to_target = np.linalg.norm(self.p - self.target)
            initial_dist = np.linalg.norm(self.start_pos - self.target)

            # 如果速度很低且距离目标还很远
            if avg_speed < 0.2 * self.vmax and dist_to_target > 0.3 * initial_dist:
                self.deadlock_counter += 1
                if self.deadlock_counter > 10:
                    return True
            else:
                self.deadlock_counter = max(0, self.deadlock_counter - 1)
        return False

    def compute_rotation_matrix(self, angle):
        """2D旋转矩阵"""
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        return np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    def compute_velocity_command(self, other_agents, obstacles):
        """计算速度命令"""
        start_time = time.time()

        # 更新历史
        self.velocity_history.append(self.v.copy())
        self.position_history.append(self.p.copy())
        if len(self.velocity_history) > 30:
            self.velocity_history.pop(0)
            self.position_history.pop(0)

        # 目标方向
        to_target = self.target - self.p
        dist_to_target = np.linalg.norm(to_target)

        if dist_to_target < self.radius:
            self.reached_target = True
            self.v = np.zeros(2)
            self.plan_time_list.append(time.time() - start_time)
            return np.zeros(2)

        desired_velocity = to_target / dist_to_target * self.vmax

        # 计算风险和死锁指示器
        risk = self.compute_cbf_risk(other_agents, obstacles)
        zeta = self.compute_deadlock_indicator(risk)

        # 检测死锁
        if self.detect_deadlock():
            self.in_deadlock = True
            self.rotation_direction = np.random.choice([-1, 1])
        else:
            self.in_deadlock = False

        # ===== 核心控制策略 =====
        # 首先计算基础避障速度
        avoidance_velocity = np.zeros(2)

        # 障碍物避障（关键！考虑加速度约束，需要更早开始避障）
        # 由于加速度限制，需要更大的安全距离
        safe_margin = self.umax * self.h * 2  # 考虑加速度约束的安全余量
        danger_dist = self.radius * 5 + safe_margin
        caution_dist = self.radius * 10 + safe_margin
        safe_dist = self.radius * 15 + safe_margin

        for ob in obstacles:
            dist_to_ob = self._distance_to_obstacle(self.p, ob)
            direction_to_ob = self._point_to_obstacle_direction(self.p, ob)

            # 计算当前速度是否会导致碰撞
            # 如果正在向障碍物移动，需要更早避障
            approaching = np.dot(self.v, direction_to_ob) > 0 if np.linalg.norm(self.v) > 0.1 else False

            if dist_to_ob < self.radius * 2:
                # 非常危险 - 紧急避障（最高优先级）
                danger = 1.0
                # 完全远离障碍物
                away_dir = -direction_to_ob
                avoidance_velocity = away_dir * self.vmax * 2.0
                break  # 紧急避障时跳过其他处理
            elif dist_to_ob < safe_dist:
                if approaching:
                    # 正在接近，提高危险级别
                    danger_mult = 1.5
                else:
                    danger_mult = 1.0

                if dist_to_ob < danger_dist:
                    danger = 1.0 * danger_mult
                elif dist_to_ob < caution_dist:
                    danger = (caution_dist - dist_to_ob) / (caution_dist - danger_dist) * danger_mult
                else:
                    danger = (safe_dist - dist_to_ob) / (safe_dist - caution_dist) * 0.5 * danger_mult

                if danger > 0.01:
                    away_dir = -direction_to_ob
                    perpendicular = np.array([-direction_to_ob[1], direction_to_ob[0]])

                    to_target_norm = to_target / dist_to_target
                    if np.dot(perpendicular, to_target_norm) > 0:
                        bypass_dir = perpendicular
                    else:
                        bypass_dir = -perpendicular

                    avoid_speed = self.vmax * danger * 3.0
                    avoid_velocity = away_dir * avoid_speed * 0.6 + bypass_dir * avoid_speed * 0.4
                    avoidance_velocity += avoid_velocity

        # 智能体避障（考虑加速度约束，需要更大的安全距离）
        agent_safe_margin = self.umax * self.h * 2
        agent_lookahead = self.lookahead_dist + agent_safe_margin
        agent_danger_dist = self.r_min * 2 + agent_safe_margin
        agent_caution_dist = self.r_min * 4 + agent_safe_margin

        for other in other_agents:
            if other.index == self.index or other.damaged:
                continue

            diff = self.p - other.p
            dist = np.linalg.norm(diff)

            if dist < agent_lookahead and dist > 0.01:
                direction = diff / dist

                # 计算相对速度（用于预测碰撞）
                rel_vel = self.v - other.v
                approaching = np.dot(rel_vel, -diff/dist) > 0

                if dist < self.r_min * 1.5:
                    # 非常危险 - 紧急避障
                    danger = 1.0
                    avoidance_velocity += direction * self.vmax * 2.0
                elif dist < agent_lookahead:
                    if approaching:
                        danger_mult = 1.5  # 正在接近，提高警觉
                    else:
                        danger_mult = 1.0

                    if dist < agent_danger_dist:
                        danger = 1.0 * danger_mult
                    elif dist < agent_caution_dist:
                        danger = (agent_caution_dist - dist) / (agent_caution_dist - agent_danger_dist) * danger_mult
                    else:
                        danger = (agent_lookahead - dist) / agent_lookahead * 0.3 * danger_mult

                    if danger > 0.05:
                        avoid_speed = self.vmax * danger * 1.5
                        # 添加垂直方向绕行
                        perpendicular = np.array([-direction[1], direction[0]])
                        # 选择更好的绕行方向
                        if np.dot(perpendicular, to_target/dist_to_target) > 0:
                            bypass = perpendicular
                        else:
                            bypass = -perpendicular
                        avoidance_velocity += direction * avoid_speed * 0.7 + bypass * avoid_speed * 0.3

        # ===== 组合速度 =====
        if np.linalg.norm(avoidance_velocity) > 0.01:
            avoidance_weight = min(0.85, max(0.3, risk * 0.6))
            velocity_cmd = desired_velocity * (1 - avoidance_weight) + avoidance_velocity * avoidance_weight
        else:
            velocity_cmd = desired_velocity

        # ===== 死锁解决策略 =====
        if zeta > 0.6 or self.in_deadlock:
            # 更新旋转角度
            self.rotation_angle += self.rotation_direction * 0.05

            # 找到主要障碍方向
            main_obstacle_dir = np.zeros(2)
            min_dist = float('inf')

            for other in other_agents:
                if other.index == self.index or other.damaged:
                    continue
                dist = np.linalg.norm(self.p - other.p)
                if dist < min_dist:
                    min_dist = dist
                    main_obstacle_dir = (self.p - other.p) / (dist + 0.01)

            # 绕行方向
            perpendicular = np.array([-main_obstacle_dir[1], main_obstacle_dir[0]])
            if self.rotation_direction < 0:
                perpendicular = -perpendicular

            # 添加绕行分量
            bypass_weight = min(0.4, zeta * 0.5)
            velocity_cmd = velocity_cmd * (1 - bypass_weight) + perpendicular * self.vmax * bypass_weight

        # ===== 应用加速度约束 =====
        velocity_cmd = self._apply_acceleration_constraint(velocity_cmd)

        # 限制最大速度
        speed = np.linalg.norm(velocity_cmd)
        if speed > self.vmax:
            velocity_cmd = velocity_cmd / speed * self.vmax

        self.plan_time_list.append(time.time() - start_time)
        return velocity_cmd

    def _apply_acceleration_constraint(self, velocity_cmd):
        """应用加速度约束

        论文要求：||u|| ≤ u_max
        离散时间：||v_new - v_old|| / h ≤ u_max
        """
        dv = velocity_cmd - self.v
        dv_norm = np.linalg.norm(dv)
        max_dv = self.umax * self.h  # 时间步内最大速度变化

        if dv_norm > max_dv and dv_norm > 0.01:
            # 限制加速度
            dv = dv / dv_norm * max_dv
            velocity_cmd = self.v + dv

        return velocity_cmd

    def update(self, velocity_cmd):
        """更新位置和速度"""
        if self.damaged or self.reached_target:
            self.trajectory.append(self.p.copy())
            self.pre_traj_list.append(np.array([self.p, self.p]))
            return

        # 应用加速度约束
        velocity_cmd = self._apply_acceleration_constraint(velocity_cmd)

        # 限制最大速度
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
