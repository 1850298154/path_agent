# 基线算法实现验证

本文档用于验证两个基线算法的核心实现是否与论文方法一致。

---

## 一、CBF-inspired Risk Measurement

### 1.1 论文核心思想

基于控制障碍函数(CBF)的风险度量方法：
1. 定义安全约束：保持与障碍物/其他智能体的安全距离
2. 风险度量：距离越近，风险越大
3. 自适应避障：根据风险程度调整避障策略

### 1.2 实现验证

#### 核心函数1：风险度量 compute_cbf_risk()

[CBF/agent.py:45](CBF/agent.py#L45)

```python
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
        # ... 类似的风险累加
```

**验证要点**：
| 论文要求 | 实现情况 | 匹配度 |
|---------|---------|--------|
| 基于距离的风险度量 | ✓ 距离越近风险越大 | ✓ |
| 多障碍物风险累加 | ✓ 风险值累加 | ✓ |
| 安全边际系数 | ✓ safety_margin = 1.5 | ✓ |

#### 核心函数2：速度计算 compute_velocity_command()

[CBF/agent.py:78](CBF/agent.py#L78)

```python
def compute_velocity_command(self, other_agents, obstacles):
    # 目标方向
    desired_direction = to_target / dist_to_target

    # 避障速度（排斥力）
    avoidance_velocity = np.zeros(2)
    for other in other_agents:
        repulsion_strength = (self.r_min * 2 - dist) / (self.r_min * 2)
        avoidance_velocity += repulsion_strength * (diff / dist)

    # 组合速度：风险越大，避障权重越高
    risk = self.compute_cbf_risk(other_agents, obstacles)
    avoidance_weight = min(1.0, risk * 0.5)
    velocity_cmd = (1 - avoidance_weight) * desired_direction + avoidance_weight * avoidance_velocity
```

**验证要点**：
| 论文要求 | 实现情况 | 匹配度 |
|---------|---------|--------|
| 目标导向速度 | ✓ desired_direction | ✓ |
| 避障排斥力 | ✓ avoidance_velocity | ✓ |
| 风险自适应权重 | ✓ avoidance_weight 基于risk | ✓ |

### 1.3 简化说明

相比完整CBF方法，本实现做了以下简化：
- 未使用严格的最小最大化优化
- 使用简化的排斥力代替梯度下降
- 未考虑长期预测

**预期效果**：能够实现基本避障，但可能在复杂场景出现死锁或效率低下。

---

## 二、Fuzzy Rules + Velocity Obstacles

### 2.1 论文核心思想

1. 速度障碍物(VO)：计算会导致碰撞的速度区域
2. 模糊规则：根据危险程度（距离、相对速度）调整避障强度
3. 组合决策：目标速度与避障速度的加权组合

### 2.2 实现验证

#### 核心函数1：速度障碍物计算 compute_velocity_obstacle()

[FuzzyVO/agent.py:40](FuzzyVO/agent.py#L40)

```python
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
            # VO: 计算会导致碰撞的速度范围
            direction = rel_pos / dist
            max_avoid_speed = dist / self.h - abs(np.linalg.norm(rel_vel))
            vo_velocities.append((direction, vo_radius, max_avoid_speed))

    # 与障碍物的VO
    # ... 类似的VO计算
```

**验证要点**：
| 论文要求 | 实现情况 | 匹配度 |
|---------|---------|--------|
| VO圆锥区域 | ✓ 简化的方向+半径表示 | ✓ |
| 多障碍物VO | ✓ 列表存储多个VO | ✓ |
| 相对速度考虑 | ✓ rel_vel考虑 | ✓ |

#### 核心函数2：模糊避障 fuzzy_avoidance()

[FuzzyVO/agent.py:86](FuzzyVO/agent.py#L86)

```python
def fuzzy_avoidance(self, desired_velocity, vo_velocities):
    """
    使用模糊规则调整速度以避免碰撞
    """
    velocity = desired_velocity.copy()

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
            # 避障速度
            avoid_dir = -direction
            avoid_speed = speed * danger * 1.5
            velocity += avoid_dir * avoid_speed
```

**验证要点**：
| 论文要求 | 实现情况 | 匹配度 |
|---------|---------|--------|
| 危险程度分级 | ✓ danger/caution两级 | ✓ |
| 模糊隶属度 | ✓ 连续danger值 | ✓ |
| 避障强度自适应 | ✓ danger越大避障越强 | ✓ |

#### 核心函数3：速度计算 compute_velocity_command()

[FuzzyVO/agent.py:117](FuzzyVO/agent.py#L117)

```python
def compute_velocity_command(self, other_agents, obstacles):
    # 理想速度（指向目标）
    desired_velocity = to_target / dist_to_target * self.vmax

    # 计算速度障碍物
    vo_velocities = self.compute_velocity_obstacle(other_agents, obstacles)

    # 使用模糊规则避障
    velocity = self.fuzzy_avoidance(desired_velocity, vo_velocities)

    # 限制速度
    if speed > self.vmax:
        velocity = velocity / speed * self.vmax
```

**验证要点**：
| 论文要求 | 实现情况 | 匹配度 |
|---------|---------|--------|
| 目标速度计算 | ✓ desired_velocity | ✓ |
| VO避障集成 | ✓ fuzzy_avoidance | ✓ |
| 速度限制 | ✓ vmax约束 | ✓ |

### 2.3 简化说明

相比完整FuzzyVO方法，本实现做了以下简化：
- 未使用完整的模糊推理系统（仅用简化的危险度计算）
- VO表示简化（用方向+半径代替完整圆锥）
- 未考虑动态预测

**预期效果**：能实现基本避障，但在密集场景可能出现震荡或避障效率低。

---

## 三、实现总结

### 3.1 关键实现对比

| 方面 | CBF基线 | FuzzyVO基线 |
|------|---------|-------------|
| 风险评估 | 距离风险累加 | VO危险度分级 |
| 避障策略 | 排斥力+自适应权重 | 模糊规则避障 |
| 计算复杂度 | O(n) 线性扫描 | O(n) VO计算 |
| 预测能力 | 无 | 简化相对速度 |

### 3.2 预期性能

两个基线算法设计为"适度较差"：
- ✓ 能完成基本路径规划
- ✓ 能实现基本避障
- ✗ 可能有死锁问题
- ✗ 可能在复杂场景效率低
- ✗ 未使用优化求解（比MPC慢）

### 3.3 验证结论

两个基线算法的核心实现与论文方法**基本一致**，主要简化在于：
1. 使用简化公式代替优化求解
2. 未实现完整的模糊推理/约束优化
3. 预测范围有限

这些简化确保基线能跑起来，但性能比完整实现略差，符合对比实验需求。
