# 基线算法实现任务描述

## 一、任务概述

实现两个多智能体路径规划基线算法，用于与用户的MPC改良版本进行对比实验。

**关键要求**：基线算法性能应当"适度较差"——能够跑起来，但可以有死锁问题、计算时间慢等问题，以衬托用户算法的优越性。

**步数限制**：最大 `step=863`（从0开始到863结束，共864步），到达不了终点就记录为失败。

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

**时间步长**: `h = 0.2`，每步最长移动距离 = `h × Vmax = 0.2 × 3.0 = 0.6`

### 2.3 参考实验结果（用户MPC改良版本）
位置: `D:\zyt\git_ln\path_agent\ob_2d\004\2023-10-31_20-42-54\agent100\a_statistics.json`

| 指标 | 值 |
|------|-----|
| `success_rate` | 1.0 (100%) |
| `collision_rate` | 0.0 (0%) |
| `average_planning_time` | 0.02306 秒 |
| `ex_collision_rate` | 0.0 (0%) |

---

## 三、指标定义（重要）

### 3.1 成功率 (success_rate)
```
success_rate = (到达终点数量) / 100
```
**到达终点判定条件**: 智能体当前位置与目标点的距离 `< radius`（即 `< 1.0`）

### 3.2 碰撞率 (collision_rate) - 内部碰撞
```
collision_rate = (发生内部碰撞数量) / 100
```
**内部碰撞定义**: 两个智能体中心距离 `< 2 × radius`（即 `< 2.0`，直径）

**碰撞处理**: 发生内部碰撞后，该智能体**停止不动**，视为损毁

### 3.3 外部碰撞率 (ex_collision_rate)
```
ex_collision_rate = (与障碍物发生碰撞数量) / 100
```
**外部碰撞定义**: 障碍物按智能体半径膨胀后，智能体中心点落在膨胀区域内

**碰撞处理**: 发生外部碰撞后，该智能体**停止不动**，视为损毁

### 3.4 平均规划时间 (average_planning_time)
```
average_planning_time = 所有智能体每一步规划时间的总和 / (智能体数量 × 步数)
```
**规划时间定义**:
- 指求解/计算的时间，不是仿真时间
- 每一步需要把100个智能体都算完后，作为一个整体时间
- 由于不是分布式计算，需要等所有智能体都完成规划后统计

---

## 四、输出要求

### 4.1 输出文件结构
```
baseline/
├── CBF/
│   └── output/
│       └── <timestamp>/          # 时间戳目录
│           ├── a_statistics.json # 统计指标
│           ├── savefig/          # 轨迹图片
│           │   ├── episode-0.jpg
│           │   ├── episode-1.jpg
│           │   └── ...
│           └── a_video.avi       # 轨迹视频
│
└── FuzzyVO/
    └── output/
        └── <timestamp>/
            ├── a_statistics.json
            ├── savefig/
            └── a_video.avi
```

### 4.2 统计指标输出格式
```json
// a_statistics.json
{
    "success_rate": 0.85,
    "collision_rate": 0.05,
    "average_planning_time": 0.05,
    "ex_collision_rate": 0.02
}
```

### 4.3 可视化输出
- **轨迹图片**: 每一步生成一张图片，显示所有智能体位置和轨迹
- **轨迹视频**: 将所有图片合成为视频（可复用 `ob_2d/jpg2mp4.py`）

---

## 五、用户代码结构参考

### 5.1 核心文件
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
├── plot.py          # 可视化（轨迹图）
├── jpg2mp4.py       # 图片合成视频
└── output_filename.py  # 输出文件名管理
```

### 5.2 可视化代码参考
```python
# plot.py - 绘制轨迹图
from plot import plot_all_pre_traj, plot_obstacle
# jpg2mp4.py - 合成视频
from jpg2mp4 import images_to_video, get_img_path_list
```

### 5.3 关键接口

#### 智能体类 (uav.py)
```python
class uav2D:
    def __init__(self, index, ini_x, target, type, ini_K=11):
        self.index = index           # 智能体编号
        self.p = ini_x.copy()        # 当前位置
        self.v = np.zeros(2)         # 当前速度
        self.target = target         # 目标位置
        self.K = ini_K               # 预测时域长度
        self.Vmax = 3.0              # 最大速度
        self.Umax = 40.0             # 最大加速度
        self.physical_radius = 0.25  # 物理半径
        self.radius = 1.0            # 安全半径
        self.r_min = 2 * radius      # 最小安全距离（直径=2.0）
        self.pre_traj_list = []      # 轨迹历史
        self.plan_time_list = []     # 规划时间记录
        self.deadlock = False        # 死锁标记
```

---

## 六、基线算法一：CBF-inspired Risk Measurement

### 6.1 论文信息
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

### 6.2 实现路径
```
D:\zyt\git_ln\path_agent\baseline\CBF\
```

### 6.3 算法核心思想
该论文提出一种基于CBF（Control Barrier Function）启发的风险度量方法，用于分散式多智能体系统的自适应死锁避免：

1. **风险度量**: 基于CBF设计风险度量函数，评估智能体之间的碰撞风险
2. **自适应避障**: 根据风险程度动态调整避障策略
3. **死锁检测与解决**: 检测潜在死锁情况并采取相应措施

### 6.4 简化实现要点（性能"适度较差"）
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

## 七、基线算法二：Fuzzy Rules + Velocity Obstacles

### 7.1 论文信息
```bibtex
@article{E2023Cooperative,
  title={Cooperative collision avoidance in multirobot systems using fuzzy rules and velocity obstacles},
  author={Tang, Wenbing and Zhou, Yuan, Zhang, Tianwei, Liu, Yang, Liu, Jing, Ding, Zuohua},
  journal={Robotica},
  volume={41},
  number={2},
  year={2023},
}
```

### 7.2 实现路径
```
D:\zyt\git_ln\path_agent\baseline\FuzzyVO\
```

### 7.3 算法核心思想
该论文结合模糊规则和速度障碍物(Velocity Obstacles, VO)实现多机器人协同避障：

1. **速度障碍物(VO)**: 计算与障碍物和其他智能体碰撞的速度区域
2. **模糊规则**: 使用模糊逻辑选择最优速度方向
3. **协同避障**: 多智能体协同调整速度避免碰撞

### 7.4 简化实现要点（性能"适度较差"）
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

## 八、代码实现规范

### 8.1 目录结构
```
baseline/
├── TASK_DESCRIPTION.md     # 本文档
├── CBF/
│   ├── __init__.py
│   ├── test.py             # 主程序入口
│   ├── agent.py            # 智能体类
│   ├── planner.py          # 路径规划器
│   ├── config.py           # 配置参数
│   └── utils.py            # 工具函数
│
└── FuzzyVO/
    ├── __init__.py
    ├── test.py
    ├── agent.py
    ├── planner.py
    ├── config.py
    └── utils.py
```

### 8.2 输入规范
从 `description.json` 读取:
```json
{
    "agent_start_list": [[x, y], ...],  // 100个起点
    "agent_end_list": [[x, y], ...],    // 100个终点
    "obstacle_list": [[x, y, size], ...] // 9个障碍物
}
```

### 8.3 运行方式
```bash
cd D:\zyt\git_ln\path_agent\baseline\CBF
python test.py
```

---

## 九、预期对比结果

### 9.1 目标性能指标

| 指标 | MPC改良版（用户） | CBF基线 | FuzzyVO基线 |
|------|-------------------|---------|-------------|
| success_rate | 1.0 | 0.7-0.9 | 0.6-0.85 |
| collision_rate | 0.0 | 0.05-0.15 | 0.1-0.2 |
| average_planning_time | 0.023 | 0.05-0.15 | 0.03-0.1 |
| ex_collision_rate | 0.0 | 0.02-0.1 | 0.05-0.15 |

### 9.2 预期优势点
用户MPC改良版相比基线算法应体现：
1. 更高的成功率
2. 更低的碰撞率
3. 更快的规划时间
4. 更平滑的轨迹

---

## 十、注意事项

### 10.1 算法实现原则
1. **必须能跑起来**: 算法要有基本的正确性，不能直接崩溃
2. **性能适度较差**: 成功率可以低一些，时间可以慢一些
3. **轨迹问题可接受**: 允许出现次优路径、轻微碰撞等问题
4. **真实基线感**: 算法要有学术依据，不能太假

### 10.2 代码质量
1. 代码结构清晰，便于理解
2. 关键步骤有注释说明
3. 与原论文方法有对应关系
4. 参数可调，便于实验

### 10.3 测试验证
1. 先在小规模（10个智能体）测试
2. 确认能正常完成运行
3. 再扩展到100个智能体
4. 记录运行时间和结果

---

## 十一、参考链接

- CBF论文: https://arxiv.org/abs/2503.09621
- FuzzyVO论文: https://xueshu.baidu.com/usercenter/paper/show?paperid=156t00p0ne1v0e80en5s02f0gr315192
- 用户代码: `D:\zyt\git_ln\path_agent\ob_2d\`

---

## 十二、实现步骤

### Step 1: 阅读论文
- 搜索并阅读CBF论文，理解核心算法
- 搜索并阅读FuzzyVO论文，理解核心算法

### Step 2: 实现CBF基线
- 创建基础框架
- 实现智能体类和规划器
- 集成可视化输出

### Step 3: 实现FuzzyVO基线
- 创建基础框架
- 实现智能体类和规划器
- 集成可视化输出

### Step 4: 测试验证
- 运行两个基线算法
- 对比用户MPC算法结果
- 确保输出格式正确
