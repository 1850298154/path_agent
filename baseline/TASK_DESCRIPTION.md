# 基线算法实现任务描述

## 一、任务概述

实现两个多智能体路径规划基线算法，用于与用户的MPC改良版本进行对比实验。**关键要求：基线算法性能应当"适度较差"——能够跑起来，但可以有死锁问题、计算时间慢等问题，以衬托用户算法的优越性。设置step=863步骤（用我的算法的step）最为最大限制步骤，从0开始~863结束，到达不了终点就记录下来。**

---

## 二、实验环境

### 2.1 场景配置文件
- **场景描述**: `D:\zyt\git_ln\path_agent\ob_2d\004\2023-10-31_20-42-54\description.json`
- **参数配置**: `D:\zyt\git_ln\path_agent\ob_2d\004\2023-10-31_20-42-54\parameters.yaml`

### 2.2 场景参数
| 参数 | 值 | 说明 |
|------|-----|------|
| `agent.Num` | 100 | 智能体数量 |
| `agent.Umax` | 40.0 | 最大加速度 |
| `agent.Vmax` | 3.0 | 最大速度 |
| `agent.physical_radius` | 0.25 | 物理半径 |
| `agent.radius` | 1.0 | 安全半径（膨胀后） |
| `map.set_xlim` | 300 | 地图X范围 |
| `map.set_ylim` | 300 | 地图Y范围 |
| `ob.num` | 9 | 障碍物数量 |
| `ob.lower_limit_Square_side_length` | 40.82 | 障碍物边长 |

另外每一步规划h=0.2，最长移动也就是 0.2*3=0.6

### 2.3 参考实验结果（用户MPC改良版本）
位置: `D:\zyt\git_ln\path_agent\ob_2d\004\2023-10-31_20-42-54\agent100\a_statistics.json`

| 指标 | 值 |
|------|-----|
| `success_rate` | 1.0 (100%) |
| `collision_rate` | 0.0 (0%) |
| `average_planning_time` | 0.02306 秒 |
| `ex_collision_rate` | 0.0 (0%) |

成功率是  (到达终点数量) / 100，碰撞率是 (发生碰撞数量) / 100，平均规划时间是所有智能体每一步规划时间的平均值，外部碰撞率是 (与障碍物发生碰撞数量) / 100。

外部碰撞就是智能体的半径与障碍物有交集，或者说是把障碍物按质能体的半径来膨胀，膨胀完之后质能体的点落到了那个膨胀后的障碍物。
内部碰撞是指两个半径之和，也就是说是一个直径，然后两点之间的距离小于这个直径就是内部碰撞。
如果内部直接发生了碰撞，就直接让它停下来，不要动了，就当是损毁了。
执行时间是指规划时间，指的是我们求解的时间。比如说，每一步有很多步，这一步求解花时间多少，然后一百个智能体的时间是多少求出来。因为有一百个智能体，他们不是分布式的，所以还是要把这一百个智能体的步骤运动方法都求出来之后，把结果整体作为一个时间。

---

## 三、用户代码结构参考

### 3.1 核心文件
```
ob_2d/
├── test.py          # 主程序入口
├── run.py           # 单步运行逻辑、MPC求解
├── uav.py           # 智能体类定义
├── SET.py           # 参数初始化
├── zyaml.py         # 参数读取
├── zrand.py         # 随机场景生成
├── zstatistics.py   # 统计指标计算
├── bug.py           # 路径规划（A*变种）
├── inter_avoid.py   # 智能体间避障约束
├── obstacle_corridor.py  # 障碍物走廊约束
├── geometry.py      # 几何计算
└── plot.py          # 可视化
```

### 3.2 关键接口

#### 智能体类 (uav.py)
```python
class uav2D:
    def __init__(self, index, ini_x, target, type, ini_K=11):
        self.index = index       # 智能体编号
        self.p = ini_x.copy()    # 当前位置
        self.v = np.zeros(2)     # 当前速度
        self.target = target     # 目标位置
        self.K = ini_K           # 预测时域长度
        self.Vmax = 3.0          # 最大速度
        self.Umax = 40.0         # 最大加速度
        self.r_min = 2*radius    # 最小安全距离（直径）
        self.pre_traj_list = []  # 轨迹历史
        self.plan_time_list = [] # 规划时间记录
        self.deadlock = False    # 死锁标记
```

#### 统计指标 (zstatistics.py)
```python
# 需要输出的指标:
- success_rate      # 成功率
- collision_rate    # 碰撞率
- average_planning_time  # 平均规划时间
- ex_collision_rate # 外部碰撞率（与障碍物）
```

#### 输出格式
```json
// a_statistics.json
{
    "success_rate": 1.0,
    "collision_rate": 0.0,
    "average_planning_time": 0.02306,
    "ex_collision_rate": 0.0
}
```

---

## 四、基线算法一：CBF-inspired Risk Measurement

### 4.1 论文信息
```bibtex
@misc{zhang2025adaptivedeadlockavoidancedecentralized,
      title={Adaptive Deadlock Avoidance for Decentralized Multi-agent Systems via CBF-inspired Risk Measurement},
      author={Yanze Zhang and Yiwei Lyu and Siwon Jo and Yupeng Yang and Wenhao Luo},
      year={2025},
      eprint={2503.09621},
      archivePrefix={arXiv},
      primaryClass={eess.SY},
      url={https://arxiv.org/abs/2503.09621},
}
```

### 4.2 实现路径
```
D:\zyt\git_ln\path_agent\baseline\CBF\
```

### 4.3 算法核心思想
该论文提出一种基于CBF（Control Barrier Function）启发的风险度量方法，用于分散式多智能体系统的自适应死锁避免：

1. **风险度量**: 基于CBF设计风险度量函数，评估智能体之间的碰撞风险
2. **自适应避障**: 根据风险程度动态调整避障策略
3. **死锁检测与解决**: 检测潜在死锁情况并采取相应措施

### 4.4 简化实现要点（性能"适度较差"）
- **可简化部分**:
  - 使用简化的风险度量函数
  - 死锁检测可以不那么精确
  - 避障策略可以相对保守（导致路径更长、时间更慢）
  - 不需要完美的自适应机制

- **可能导致"性能差"的设计**:
  - 较大的安全距离参数
  - 保守的速度限制
  - 简单的贪心避障策略
  - 遇到冲突时的简单等待机制

---

## 五、基线算法二：Fuzzy Rules + Velocity Obstacles

### 5.1 论文信息
```bibtex
@article{E2023Cooperative,
  title={Cooperative collision avoidance in multirobot systems using fuzzy rules and velocity obstacles},
  author={Tang, Wenbing and Zhou, Yuan, Zhang, Tianwei and Liu, Yang, Liu, Jing, Ding, Zuohua},
  journal={Robotica},
  volume={41},
  number={2},
  year={2023},
}
```

### 5.2 实现路径
```
D:\zyt\git_ln\path_agent\baseline\FuzzyVO\
```

### 5.3 算法核心思想
该论文结合模糊规则和速度障碍物(Velocity Obstacles, VO)实现多机器人协同避障：

1. **速度障碍物(VO)**: 计算与障碍物和其他智能体碰撞的速度区域
2. **模糊规则**: 使用模糊逻辑选择最优速度方向
3. **协同避障**: 多智能体协同调整速度避免碰撞

### 5.4 简化实现要点（性能"适度较差"）
- **可简化部分**:
  - 简化的模糊规则（较少的模糊集）
  - 基础的VO计算（不考虑动态预测）
  - 简单的速度选择策略

- **可能导致"性能差"的设计**:
  - 模糊规则设计不那么优化
  - VO计算不考虑长时间预测
  - 遇到复杂情况时的简单策略
  - 较低的优先级判断精度

---

## 六、代码实现规范

### 6.1 统一接口要求

每个基线算法目录应包含：

```
baseline/
├── CBF/
│   ├── __init__.py
│   ├── test.py              # 主程序入口（与ob_2d/test.py兼容）
│   ├── agent.py             # 智能体类
│   ├── planner.py           # 路径规划器
│   ├── config.py            # 配置参数
│   └── utils.py             # 工具函数
│
└── FuzzyVO/
    ├── __init__.py
    ├── test.py
    ├── agent.py
    ├── planner.py
    ├── config.py
    └── utils.py
```

### 6.2 输入输出规范

#### 输入
- 从 `description.json` 读取:
  - `agent_start_list`: 100个智能体起点坐标
  - `agent_end_list`: 100个智能体终点坐标
  - `obstacle_list`: 9个障碍物位置和大小

#### 输出
- 在同目录下创建 `agent100/` 文件夹
- 输出 `a_statistics.json` 包含四个指标
- 可选：输出轨迹可视化图

### 6.3 兼容性要求
- 能够直接读取实验配置文件
- 输出格式与用户算法一致，便于对比
- 运行命令格式：`python test.py <config_path> [not_show]`

---

## 七、预期对比结果

### 7.1 目标性能指标

| 指标 | MPC改良版（用户） | CBF基线 | FuzzyVO基线 |
|------|-------------------|---------|-------------|
| success_rate | 1.0 | 0.7-0.9 | 0.6-0.85 |
| collision_rate | 0.0 | 0.05-0.15 | 0.1-0.2 |
| average_planning_time | 0.023 | 0.05-0.15 | 0.03-0.1 |
| ex_collision_rate | 0.0 | 0.02-0.1 | 0.05-0.15 |

### 7.2 预期优势点
用户MPC改良版相比基线算法应体现：
1. 更高的成功率
2. 更低的碰撞率
3. 更快的规划时间
4. 更平滑的轨迹

---

## 八、注意事项

### 8.1 算法实现原则
1. **必须能跑起来**: 算法要有基本的正确性，不能直接崩溃
2. **性能适度较差**: 成功率可以低一些，时间可以慢一些
3. **轨迹问题可接受**: 允许出现次优路径、轻微碰撞等问题
4. **真实基线感**: 算法要有学术依据，不能太假

### 8.2 代码质量
1. 代码结构清晰，便于理解
2. 关键步骤有注释说明
3. 与原论文方法有对应关系
4. 参数可调，便于实验

### 8.3 测试验证
1. 先在小规模（10个智能体）测试
2. 确认能正常完成运行
3. 再扩展到100个智能体
4. 记录运行时间和结果

---

## 九、时间安排

| 阶段 | 任务 | 预估时间 |
|------|------|----------|
| 第一阶段 | CBF基线算法实现 | - |
| 第二阶段 | FuzzyVO基线算法实现 | - |
| 第三阶段 | 统一测试与对比分析 | - |

---

## 十、参考链接

- CBF论文: https://arxiv.org/abs/2503.09621
- FuzzyVO论文: https://xueshu.baidu.com/usercenter/paper/show?paperid=156t00p0ne1v0e80en5s02f0gr315192
- 用户代码: `D:\zyt\git_ln\path_agent\ob_2d\`
