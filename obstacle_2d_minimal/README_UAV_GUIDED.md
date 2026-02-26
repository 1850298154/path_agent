# UAV分析结果引导障碍物路径规划

## 概述

此模块演示如何将UAV任务分配分析结果转换为障碍物路径规划场景。

## 工作流程

### 1. UAV分析结果

位置: `../uav_allocation_analysis/`

关键文件:
- `uav_positions_over_time.json` - 包含80个UAV的完整轨迹数据
  - 时间范围: 0-899秒
  - 时间步长: 1秒
  - UAV起点范围: [0.3, 0.3] 和 [12.3, 0.3] (两个基地)
  - 位置范围: X∈[0.3, 30.09], Y∈[0.3, 25.71]

### 2. 场景生成

运行脚本生成障碍物规划场景:

```bash
python generate_uav_guided_scenario.py
```

生成文件:
- `005/uav_guided_scenario/description.json` - 场景描述
  - agent_start_list: 智能体起点位置
  - agent_end_list: 智能体终点位置
  - obstacle_list: 障碍物列表
  - UnmannedSystem_list: 智能体类型列表

- `005/uav_guided_scenario/parameters.yaml` - 算法参数

### 3. 坐标映射

UAV分析坐标 → 障碍物地图坐标 (330x330)

缩放公式:
```
usable_size = 330 - 2 * margin  # margin = 30
scale_x = usable_size / (30.09 - 0.3)
scale_y = usable_size / (25.71 - 0.3)

scaled_x = 30 + (original_x - 0.3) * scale_x
scaled_y = 30 + (original_y - 0.3) * scale_y
```

示例映射:
| 原始坐标 | 缩放坐标 |
|-----------|-----------|
| [0.30, 0.30] | [30.00, 30.00] |
| [3.41, 23.89] | [58.15, 280.65] |

### 4. 运行测试

由于系统的复杂依赖关系，当前有两种运行方式:

#### 方式一: 使用原始test.py (需要完整依赖)

```bash
# 复制场景文件到004目录
cp -r 005/uav_guided_scenario/* 004/uav_guided_scenario/

# 运行测试
python test.py uav_guided_scenario 004/uav_guided_scenario/parameters.yaml not_show
```

#### 方式二: 使用测试脚本

```bash
python test_uav_guided.py uav_guided_scenario 004/uav_guided_scenario/parameters.yaml not_show
```

## 配置参数

### generate_uav_guided_scenario.py 中可修改的参数

```python
# 选择的UAV ID (从0-79中选择)
SELECTED_UAV_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# 地图尺寸
MAP_SIZE = 330.0

# 地图边距
MAP_MARGIN = 30
```

## 数据结构

### uav_positions_over_time.json

```json
{
  "time_step": 1.0,
  "makespan": 899.0,
  "uavs": {
    "0": {
      "init_pos": [0.3, 0.3],
      "positions": [[0.3, 0.3], [1.08, 1.58], ...],
      "times": [0.0, 1.0, 2.0, ...]
    },
    ...
  }
}
```

### description.json

```json
{
  "agent_start_list": [[30.0, 30.0], ...],
  "agent_end_list": [[58.15, 280.65], ...],
  "obstacle_list": [[50, 100, 30], ...],
  "UnmannedSystem_list": ["unicycle", "Mecanum", ...]
}
```

## 已知问题

1. **系统依赖复杂**: 障碍物路径规划系统有很多全局状态和模块依赖，需要特定的初始化顺序

2. **pkl文件格式**: 系统期望特定格式的pickle文件，直接生成比较困难

3. **datetime格式**: 系统使用特定的时间格式进行输出目录命名

## 建议改进方向

1. 简化系统依赖，解耦模块
2. 支持从JSON/YAML直接初始化，不依赖pkl文件
3. 统一坐标系统和配置格式

## 文件清单

| 文件 | 说明 |
|------|------|
| generate_uav_guided_scenario.py | 场景生成脚本 |
| generate_simple_pkl.py | 简化pkl生成脚本 |
| test_uav_guided.py | UAV引导测试脚本 |
| extract_uav_positions.py | UAV位置提取脚本 |
