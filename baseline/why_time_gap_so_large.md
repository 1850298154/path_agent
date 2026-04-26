# 为什么四个算法的时间差距这么大？

## 实测数据

| 算法 | 单步时间(ms) | 成功率 | 碰撞率 |
|------|-------------|--------|--------|
| 本文方法 | 23.06 | 100% | 0% |
| RL-NET | 3.52 | — | — |
| CBF | 0.63 | 57% | 34% |
| FuzzyVO | 1.43 | 73% | 18% |

数据来源：[本文方法统计](baseline/OURS/2023-10-31_20-42-54/agent100/a_statistics.json)、[CBF统计](baseline/CBF/output/2026-03-07_23-37-57/a_statistics.json)、[FuzzyVO统计](baseline/FuzzyVO/output/2026-03-07_22-03-11/a_statistics.json)

---

## 核心原因：一个在做"迭代优化"，另外两个只做"算术运算"

| 算法 | 每步核心计算 | 是否调用优化求解器 |
|------|-------------|------------------|
| 本文方法 | 构建SOCP → 内点法迭代求解 → 死锁检测+破解 → Bug全局路径 | **是，cvxopt.coneqp** |
| RL-NET | 神经网络一次前向推理（矩阵乘法） | 否 |
| CBF | 计算风险值 + 向量加减 + clip | **否，纯代数运算** |
| FuzzyVO | 计算VO列表 + if-then模糊规则 + 向量叠加 | **否，纯代数运算** |

---

## 四个算法每步具体在算什么？

### 本文方法（23.06ms）— 每步解一个凸优化问题

计算流程见 [](ob_2d/run.py:219) `run_one_agent()` 和 [](ob_2d/run.py:130) `run_one_step()`：

1. **数据共享** — 收集所有智能体的预测轨迹、优先级、距离信息
2. **邻居避障约束** [](ob_2d/inter_avoid.py:79) `Get_inter_cons()` — 对m个邻居、K个预测步，构造MBVC半平面约束矩阵A、B，终端步使用带对数障碍的 [](ob_2d/inter_avoid.py:238) `MBVC_WB()`
3. **障碍物走廊约束** [](ob_2d/obstacle_corridor.py:15) `Get_ob_cons()` — 对最近1-2个障碍物，**调用SVM求解**（[](ob_2d/geometry.py:484) `SVM()` 内部调用 `cvxopt.qp`），求分离超平面，这本身就是一次QP求解
4. **SOCP求解** [](ob_2d/run.py:758) — 组装P、q、G、h、A、b矩阵，调用 `cvxopt.solvers.coneqp`，**内点法需要迭代10-50次，每次迭代都要解一个线性方程组**
5. **死锁检测+破解** — 优先级调整、eta更新
6. **Bug全局路径规划** — 更新牵引点

计时点在 [](ob_2d/run.py:354) `agent.plan_time_list.append(total_seconds_float)`

**关键：第4步的SOCP求解是耗时主体。** 内点法不是一步到位的公式，而是一个迭代过程。决策变量规模 O(K×D)，约束规模 O(m×K + ob_count)，每次迭代的线性方程组求解复杂度 O(n²)~O(n³)。

### CBF（0.63ms）— 纯向量运算，无求解器

计算流程见 [](baseline/CBF/agent.py:165) `compute_velocity_command()`：

```python
# 1. 算目标方向（一次向量减法+归一化）
desired_velocity = to_target / dist_to_target * self.vmax

# 2. 算风险值（几个dot product + max运算）
h = dist_sq - r_sum_sq           # 一个减法
h_dot = 2 * dot(diff, rel_vel)   # 一个点积
risk += max(0, -h_dot - alpha*h) # 一个max

# 3. 算避障速度（向量加法 + if-else）
avoidance_velocity += direction * avoid_speed * 0.7 + bypass * avoid_speed * 0.3

# 4. 加速度约束clip（一次norm + 缩放）
dv = dv / dv_norm * max_dv
```

**没有任何优化求解器调用。** 全是 O(1) 的简单算术：距离计算 → if-else判断危险级别 → 向量叠加 → clip。

计时点在 [](baseline/CBF/agent.py:342) `self.plan_time_list.append(time.time() - start_time)`

### FuzzyVO（1.43ms）— 比CBF更简单的if-then规则

计算流程见 [](baseline/FuzzyVO/agent.py:163) `compute_velocity_command()`：

```python
# 1. 算目标方向
desired_velocity = to_target / dist_to_target * self.vmax

# 2. 对每个VO计算危险级别（if-else分段函数）
if dist < danger_dist: danger = 1.0
elif dist < caution_dist: danger = (caution_dist - dist) / (caution_dist - danger_dist)

# 3. 向量叠加
velocity += away_dir * avoid_speed * 0.5 + bypass_dir * avoid_speed * 0.5

# 4. clip
velocity = velocity / speed * self.vmax
```

**同样没有任何优化求解器。** 甚至连CBF的风险函数都没有，就是纯粹的if-then规则。

VO计算见 [](baseline/FuzzyVO/agent.py:67) `compute_velocity_obstacle()`，模糊规则见 [](baseline/FuzzyVO/agent.py:111) `fuzzy_avoidance()`

计时点在 [](baseline/FuzzyVO/agent.py:188) `self.plan_time_list.append(time.time() - start_time)`

### RL-NET（3.52ms）— 神经网络前向推理

本项目中**没有RL-NET的实现**，3.52ms这个数据大概率来自论文原文。神经网络前向推理本质就是矩阵乘法（权重×输入）+ 激活函数，比优化求解快，比纯规则慢，3.52ms是合理的。

---

## 智能体/障碍物增多，时间增长规律是什么？

### 本文方法：非线性增长

| 增长因素 | 影响方式 | 代码中的上限 |
|----------|---------|-------------|
| 邻居数m | 约束矩阵行数×m，SOCP求解时间**非线性增长** | [](ob_2d/inter_avoid.py:125) 限制为 `sorted_obstacles[:min(max(i,2),zr.m)]` |
| 障碍物数 | SVM调用次数×障碍物数 | [](ob_2d/obstacle_corridor.py:54) 限制为 `sorted_obstacles[:min(max(i,1),2)]`，最多2个 |
| 预测步数K | 决策变量 O(K×D)，约束 O(K)，求解复杂度 O(n²)~O(n³) | 配置参数 |
| 智能体总数N | 系统总时间 O(N) × 单智能体时间 | 可并行，见 [](ob_2d/run.py:130) `run_one_step()` 支持 norm/thread/process |

代码中**硬性限制了邻居数m和障碍物数为2个**，所以单智能体的计算量不会无限增长。但即便如此，SOCP内点法的迭代特性决定了它远比代数运算慢。

### CBF/FuzzyVO：线性增长，系数极小

| 增长因素 | 影响方式 | 每个额外邻居的开销 |
|----------|---------|------------------|
| 邻居数N | 多遍历一个邻居，就多算一次dot product | O(1) 简单算术，微秒级 |
| 障碍物数 | 多遍历一个障碍物，就多算一次距离 | O(1) 简单算术，微秒级 |

CBF/FuzzyVO的计算量确实 O(N) 增长，但每次迭代只做向量加减乘除。10个智能体变20个，总时间从0.5ms变成1ms，几乎感觉不到。

---

## 有没有多项式增长？

**有，所有算法都有，但增长的阶数和系数天差地别：**

| 算法 | 复杂度 | 说明 |
|------|--------|------|
| 本文方法 | O(n²)~O(n³)，n=决策变量+约束总数 | 内点法迭代，每轮解线性方程组 |
| CBF | O(N)，但系数极小 | 每个邻居一次dot product |
| FuzzyVO | O(N)，但系数极小 | 每个邻居一次if-else |
| RL-NET | O(1)（推理阶段） | 固定网络结构，输入维度固定 |

---

## 一句话总结

时间差距大不是bug，是算法本质决定的：**本文方法每步要解一个凸优化问题（迭代式求解器），CBF/FuzzyVO每步只是算几个向量和if-else规则。** 这就像"解方程组"和"算加减法"的差距——复杂度不在一个量级。本文方法多花的这20ms换来的是100%成功率和0碰撞率，CBF成功率57%、FuzzyVO成功率73%。
