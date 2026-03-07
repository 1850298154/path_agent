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

**时间步长**: `h = 0.2`，每步最长移动距离 = `h × Vmax = 0.2 × 3.0 = 0.6`

---

## 三、数据格式详解（重要）

### 3.1 障碍物数据格式

**障碍物在 description.json 中的存储格式**：
```json
"obstacle_list": [
    [x, y, size],  // 障碍物0
    [x, y, size],  // 障碍物1
    ...
]
```

**关键说明**：
- `x, y`: 障碍物的**左下角坐标**（不是中心！）
- `size`: 障碍物的**边长**（正方形）

**示例**：
```
障碍物 [18.24, 50.56, 40.82]
- 左下角坐标: (18.24, 50.56)
- 右上角坐标: (18.24+40.82, 50.56+40.82) = (59.06, 91.38)
- 四个顶点（逆时针）:
  - 左下: (18.24, 50.56)
  - 右下: (59.06, 50.56)
  - 右上: (59.06, 91.38)
  - 左上: (18.24, 91.38)
```

### 3.2 障碍物绘制方法

**绘制原始障碍物（不膨胀）**：
```python
def draw_obstacle(obstacle):
    x, y, size = obstacle[0], obstacle[1], obstacle[2]
    # 四个顶点
    vertices = [
        (x, y),              # 左下
        (x + size, y),       # 右下
        (x + size, y + size),# 右上
        (x, y + size),       # 左上
    ]
    X = [v[0] for v in vertices] + [vertices[0][0]]  # 闭合
    Y = [v[1] for v in vertices] + [vertices[0][1]]
    plt.fill(X, Y, facecolor='forestgreen', alpha=0.3)
```

**绘制膨胀后的障碍物**：
```python
def draw_inflated_obstacle(obstacle, radius):
    x, y, size = obstacle[0], obstacle[1], obstacle[2]
    # 膨胀后的左下角坐标
    x_new = x - radius
    y_new = y - radius
    size_new = size + 2 * radius

    vertices = [
        (x_new, y_new),
        (x_new + size_new, y_new),
        (x_new + size_new, y_new + size_new),
        (x_new, y_new + size_new),
    ]
    # ... 绘制
```

### 3.3 障碍物碰撞检测

**智能体与障碍物的碰撞检测**：
```python
def check_external_collision(agent_pos, obstacles, radius):
    """
    检查智能体是否与障碍物碰撞
    膨胀方式：障碍物四边向外扩展 radius
    """
    for ob in obstacles:
        x, y, size = ob[0], ob[1], ob[2]
        # 膨胀后的边界
        left = x - radius
        right = x + size + radius
        bottom = y - radius
        top = y + size + radius

        # 检查点是否在膨胀后的正方形内
        if left <= agent_pos[0] <= right and bottom <= agent_pos[1] <= top:
            return True
    return False
```

### 3.4 智能体数据格式

```json
"agent_start_list": [[x1, y1], [x2, y2], ...],  // 100个起点
"agent_end_list": [[x1, y1], [x2, y2], ...],    // 100个终点
```

---

## 四、指标定义（重要）

### 4.1 成功率 (success_rate)
```
success_rate = (到达终点数量) / 智能体总数
```
**到达终点判定条件**: 智能体当前位置与目标点的距离 `< radius`（即 `< 1.0`）

### 4.2 碰撞率 (collision_rate) - 内部碰撞
```
collision_rate = (发生内部碰撞数量) / 智能体总数
```
**内部碰撞定义**: 两个智能体中心距离 `< 2 × radius`（即 `< 2.0`，直径）

**碰撞处理**: 发生内部碰撞后，该智能体**停止不动**，视为损毁

### 4.3 外部碰撞率 (ex_collision_rate)
```
ex_collision_rate = (与障碍物发生碰撞数量) / 智能体总数
```
**外部碰撞定义**: 智能体中心点落在障碍物膨胀区域内

**碰撞处理**: 发生外部碰撞后，该智能体**停止不动**，视为损毁

### 4.4 平均规划时间 (average_planning_time)
```
average_planning_time = 所有智能体每一步规划时间的总和 / (智能体数量 × 步数)
```

---

## 五、输出要求

### 5.1 输出文件结构
```
baseline/
├── CBF/
│   └── output/
│       └── <timestamp>/
│           ├── a_statistics.json   # 统计指标
│           ├── savefig/            # 轨迹图片
│           │   ├── episode-0.jpg
│           │   ├── episode-20.jpg
│           │   └── ...
│           └── a_video.avi         # 轨迹视频
└── FuzzyVO/
    └── ...
```

### 5.2 绘图频率建议
- **每20步画一张图**，不要每步都画（节省时间和空间）
- 最后一步（step-1）也要画一张
- 图片命名格式：`episode-{step}.jpg`

### 5.3 统计指标输出格式
```json
{
    "success_rate": 0.85,
    "collision_rate": 0.05,
    "average_planning_time": 0.05,
    "ex_collision_rate": 0.02
}
```

---

## 六、基线算法

### 6.1 CBF-inspired Risk Measurement

**实现路径**: `D:\zyt\git_ln\path_agent\baseline\CBF\`

**算法核心思想**：
1. 风险度量：基于距离评估碰撞风险
2. 自适应避障：根据风险程度调整避障策略
3. 简化实现：使用贪心避障策略

**简化要点**：
- 较大的安全距离参数
- 保守的速度选择
- 简单的排斥力避障

### 6.2 Fuzzy Rules + Velocity Obstacles

**实现路径**: `D:\zyt\git_ln\path_agent\baseline\FuzzyVO\`

**算法核心思想**：
1. 速度障碍物(VO)：计算会导致碰撞的速度区域
2. 模糊规则：根据危险程度调整速度
3. 简化实现：基础VO计算 + 简单模糊规则

**简化要点**：
- 简化的模糊规则
- 不考虑长时间预测
- 简单的避障权重计算

---

## 七、开发与运行策略

### 7.1 开发流程（快速迭代）
1. **小规模测试优先**: 先用 `5个智能体 + 50步` 测试，确保代码能跑通
2. **验证障碍物绘制**: 确保障碍物位置和形状正确
3. **验证碰撞检测**: 确保内部/外部碰撞检测正确
4. **扩展到全规模**: 测试通过后再扩展到 `100个智能体 + 863步`

### 7.2 测试参数配置
| 阶段 | 智能体数量 | 最大步数 | 说明 |
|------|-----------|----------|------|
| 开发测试 | 5 | 50 | 快速验证代码正确性 |
| 中等测试 | 20 | 200 | 验证算法扩展性 |
| 完整运行 | 100 | 863 | 最终对比实验 |

### 7.3 大规模运行方式
```bash
cd D:\zyt\git_ln\path_agent\baseline\CBF
python test.py > output/<timestamp>/run.log 2>&1 &
```

---

## 八、代码实现经验总结

### 8.1 障碍物处理经验
1. **坐标格式**：障碍物坐标是**左下角**，不是中心
2. **绘制方法**：需要转换为四个顶点后用 `plt.fill()` 绘制
3. **膨胀计算**：左下角坐标减去radius，边长加2倍radius
4. **碰撞检测**：判断点是否在膨胀后的矩形内

### 8.2 常见错误
| 错误 | 原因 | 正确做法 |
|------|------|----------|
| 障碍物位置偏移 | 把当作中心 | 是左下角坐标 |
| 碰撞检测不准 | 用圆近似障碍物 | 用矩形边界检测 |
| 绘图太多 | 每步都画图 | 每20步画一次 |

### 8.3 参考代码位置
```
ob_2d/
├── geometry.py      # 障碍物类定义（rectangle, polygon）
├── plot.py          # 绘图函数（plot_obstacle）
├── zrand.py         # 障碍物数据加载
└── SET.py           # 障碍物初始化（ini_obstacle_list, obstacle_list）
```

---

## 九、实现步骤

### Step 1: 快速实现CBF基线框架 ✅
- 创建基础目录结构 ✅
- 实现智能体类和简化规划器 ✅
- 用小规模测试跑通 ✅
- 验证障碍物绘制正确 ✅

### Step 2: 快速实现FuzzyVO基线框架 ✅
- 创建基础目录结构 ✅
- 实现智能体类和简化规划器 ✅
- 用小规模测试跑通 ✅
- 验证障碍物绘制正确 ✅

### Step 3: 大规模后台运行
- 运行100智能体863步
- 输出重定向到日志文件
- 生成最终对比结果

---

## 十、测试记录

### 10.1 小规模测试（5智能体，50步）

**CBF基线**：
| 指标 | 值 |
|------|-----|
| success_rate | 0.0 |
| collision_rate | 0.0 |
| ex_collision_rate | 0.0 |

**FuzzyVO基线**：
| 指标 | 值 |
|------|-----|
| success_rate | - |
| collision_rate | - |
| ex_collision_rate | - |

**说明**: 50步不足以让智能体到达目标（距离70-244，50步最多走30）

### 10.2 大规模测试（100智能体，863步）
待运行...

---

## 十一、参考链接

- CBF论文: https://arxiv.org/abs/2503.09621
- FuzzyVO论文: https://xueshu.baidu.com/usercenter/paper/show?paperid=156t00p0ne1v0e80en5s02f0gr315192
- 用户代码: `D:\zyt\git_ln\path_agent\ob_2d\`
