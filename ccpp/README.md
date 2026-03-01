# CCPP (Coverage Path Planning) 平面覆盖扫描路径规划

## 简介

本模块实现了平面覆盖扫描路径点生成和无人机分配算法，用于生成多架无人机覆盖给定凸多边形区域的蛇形走位路径。

## 算法说明

### 1. 路径点生成算法 (PolygonRegion)

算法流程：

1. **找到入口边**：计算无人机起点到多边形各边的距离，选择最近的边作为入口边

2. **选择平行边**：选择入口边相邻的两条边中，截距跨度最小的边作为平行边

3. **生成平行线段**：根据扫描宽度 (2 × scout_range) 生成平行线段

4. **计算交点**：计算每条平行线与多边形各边的交点

5. **调整顺序**：调整点的顺序形成蛇形走位路径

### 2. 无人机分配算法 (UAVAssignment)

算法流程：

1. **路径分段**：将扫描路径按蛇形走位模式划分为多个任务段

2. **计算工作量**：计算每段的工作量（距离）

3. **平均分配**：将任务段平均分配给各无人机

4. **优化分配**：使用匈牙利算法优化无人机-任务分配，最小化总飞行距离

### 3. 负载均衡算法 (LoadBalancer)

根据区域大小按比例分配无人机数量，适用于多区域场景。

## 文件结构

```
ccpp/
├── __init__.py           # 模块入口
├── line_segment.py       # 线段类，用于交点计算
├── polygon_region.py     # 多边形区域类，生成蛇形走位路径
├── uav_assignment.py    # 无人机分配算法
├── ccpp_solver.py        # 主求解器
├── test_ccpp.py         # 测试脚本
├── test_outputs/        # 测试输出图片目录
└── README.md           # 本文档
```

## 使用方法

### 基本使用

```python
from ccpp import CCPPPlanner

# 创建规划器
planner = CCPPPlanner()

# 定义多边形顶点
polygon = [[0, 0], [100, 0], [100, 100], [0, 100]]

# 定义无人机位置
uav_positions = [[0, 0], [100, 100]]

# 定义扫描范围（半个范围，实际扫描宽度 = 2 * scout_range）
scout_range = 10

# 执行规划
assignments = planner.plan(polygon, uav_positions, scout_range)

# 获取分配结果
for i, path in enumerate(assignments):
    print(f"UAV {i + 1} path: {len(path)} points")
    for j, point in enumerate(path):
        print(f"  Point {j}: ({point[0]:.1f}, {point[1]:.1f})")
```

### 可视化

```python
# 可视化结果
planner.visualize(save_path="output.png")
```

### 高级使用

#### 单独使用路径生成

```python
from ccpp import PolygonRegion

# 创建多边形区域
polygon = [[0, 0], [100, 0], [100, 100], [0, 100]]
region = PolygonRegion(polygon, scout_range=10, uav_num=2)

# 设置起点
region.update_start_point([0, 0])

# 生成路径
region.initilize_boundpoint_list_edge()

# 获取路径点
path = region.get_scan_path()
print(f"Path has {len(path)} points")
```

#### 单独使用无人机分配

```python
from ccpp import UAVAssignment

# 定义边界点（已生成的扫描路径）
bound_points = [[0, 0], [100, 0], [100, 100], [0, 100]]

# 定义无人机位置
uav_positions = [[0, 0], [100, 100]]

# 执行分配
assignments = UAVAssignment.assign(uav_positions, bound_points)

# 获取分配结果
for i, path in enumerate(assignments):
    print(f"UAV {i + 1}: {len(path)} points")
```

#### 使用负载均衡算法

```python
from ccpp import LoadBalancer

# 分配无人机到多个区域
n_drone = 10
m_region = 5
region_sizes = [20, 15, 30, 10, 25]

assignments = LoadBalancer.assign_drones(n_drone, m_region, region_sizes)
print(f"Assigned drones: {assignments}")
# 输出: [2, 2, 3, 1, 2]
```

## 运行测试

```bash
cd D:\zyt\git_ln\path_agent
uv run python ccpp/test_ccpp.py
```

测试脚本会生成 10 组随机测试用例，每组包括：
- 一个随机生成的凸多边形（3-8 个顶点）
- 随机数量的无人机（1-5 架）
- 随机扫描范围（5-30）

测试结果保存在 `ccpp/test_outputs/` 目录下。

## 测试结果

所有 10 组测试用例均已验证通过：

| 测试用例 | 顶点数 | 无人机数 | 扫描范围 | 状态 |
|---------|--------|---------|---------|------|
| Test 1  | 7      | 1       | 9.8      | PASSED |
| Test 2  | 4      | 2       | 23.0     | PASSED |
| Test 3  | 5      | 2       | 15.1     | PASSED |
| Test 4  | 4      | 3       | 14.4     | PASSED |
| Test 5  | 4      | 3       | 23.9     | PASSED |
| Test 6  | 8      | 3       | 23.0     | PASSED |
| Test 7  | 8      | 4       | 9.6      | PASSED |
| Test 8  | 6      | 2       | 20.6     | PASSED |
| Test 9  | 6      | 3       | 7.6      | PASSED |
| Test 10 | 7      | 3       | 7.6      | PASSED |

## 算法特点

1. **蛇形走位**：生成平行线式的来回移动路径，确保全面覆盖

2. **智能入口**：自动选择距离无人机最近的边作为入口边

3. **优化分配**：使用匈牙利算法优化无人机-任务分配

4. **负载均衡**：支持基于区域大小的无人机分配

5. **通用性**：适用于任意凸多边形区域

## 依赖项

- numpy
- scipy
- matplotlib (用于可视化)
- shapely (原代码中使用，本简化版未使用)

## 参考

本算法基于原始项目 `hierarchical_multitasking_multi-agent_planning` 中的覆盖扫描路径规划算法进行抽取和简化。

原始文件位置：
- `src/factory/scout_area_planner/robotR.py` - 核心数据结构
- `src/factory/scout_area_planner/planner.py` - 路径规划算法
- `src/factory/scout_area_planner/worldR.py` - 世界模拟
- `src/factory/scout_area_planner/cpp_main.py` - 主入口
- `src/factory/scout_area_planner/load_balancer.py` - 负载均衡算法
