# 代码重构分析与计划

## 执行时间
2026-02-26 17:10

## 分析范围
- obstacle_2d_minimal/ 目录所有Python文件
- 模块间调用关系
- 未使用代码识别
- 重构建议

---

## 一、代码结构总览

### 1.1 主要入口文件

#### test.py (8636字节，~86行)
**导入的模块：**
```
import output_filename as of          # 路径管理
import zstatistics as zs             # 统计功能
from plot import *                   # 绘图模块
import multiprocessing as mp            # 多进程（可能未使用）
from trajectory import land           # 轨迹库
from uav import *                  # UAV智能体类
from others import *                # 其他工具
from run import *                  # 运行逻辑（可能是旧代码）
import SET                          # 参数设置
import os, sys                    # 标准库
```

**实际使用的功能：**
- `read_pkl()` - 从pkl加载agent列表
- `intermediate_logs()` - 打印agent位置
- `save_data()` - 保存运行数据（来自others.py，被注释）
- `zs.fstatistics()` - 统计分析
- `of.save_agent100()` - 保存agent列表到json/pkl
- `plot_position()` - 绘制轨迹图

**未使用的导入：**
- `multiprocessing as mp` - 代码中没有看到多进程相关使用
- `from trajectory import land` - 只导入了但未实际使用

### 1.2 run.py (~26999字节，~812行)

**导入的模块：**
```
import zstatistics as zs
import zyaml as zy
import zException as ze
import SET
from uav import *
import numpy as np
import multiprocessing as mp
from thread import *
from obstacle_corridor import *
from inter_avoid import *
from connection import *
from trajectory import *
from plot import *
from geometry import *
from others import *
import copy
import scipy.linalg as lg
```

**代码分析：**
- 主要的碰撞检测和回退逻辑（check_one_step函数）
- 大量嵌套的碰撞检测代码（~100行）
- 使用 zs.fstatistics 记录统计信息
- 调用各种障碍物处理模块

**潜在未使用：**
- `from thread import *` - 可能是旧的多线程代码
- `from obstacle_corridor import *` - 走廊功能
- `from inter_avoid import *` - 交互避免
- `from connection import *` - 连接功能
- `scipy.linalg` - 只在注释掉的代码中使用

### 1.3 run_test.py (~237600字节，可能~7000行)
存在但未在test.py中调用

### 1.4 test_baseline_bool.py
测试baseline比较功能

### 1.5 test_uav_guided.py
UAV引导场景测试

### 1.6 生成脚本
- generate_diverse_scenario.py (生成多样化场景)
- generate_simple_pkl.py (简化pkl生成，我们刚用过)
- generate_uav_guided_scenario.py (UAV引导场景)

### 1.7 视频相关
- jpg2mp4.py (JPG转MP4视频)
- thread.py (线程模块)

---

## 二、模块详细分析

### 2.1 SET.py (6503字节，~180行)
**状态：** 核心参数配置模块，持续使用

**功能：**
- 读取parameters.yaml配置
- 管理地图参数
- 管理agent参数
- 管理障碍物参数
- 管理BUG算法参数

**问题：** 无

### 2.2 output_filename.py (13100字节，~320行)
**状态：** 路径文件名管理，核心模块

**功能：**
- 创建时间戳目录（如 `004/2026-02-26_17-09-30/`）
- 管理agent100目录
- PIOM: 保存parameters.yaml
- DIOM: 读取/保存description.json
- AIOM_Pickle: 保存/加载agent_list_100.pkl
- TIOM: 保存轨迹图 `savefig/trajecotry.jpg`

**问题：**
- 代码中有大量注释掉的旧代码（如zpytest、pyqt相关）
- 有些类命名不够清晰（PIOM、DIOM、AIOM_Pickle、SIOM、LOM等）
- VIOM（视频）类未使用

### 2.3 zyaml.py (812字节，~25行)
**状态：** YAML参数读取，持续使用

**功能：**
- 读取parameters.yaml
- 提供get/set方法

**问题：** 无

### 2.4 zstatistics.py (1179字节，~300行)
**状态：** 统计数据生成，持续使用

**功能：**
- fstatistics: 生成统计数据yaml
- calculate_average_planning_time: 计算平均规划时间
- 记录死锁、碰撞等信息

**问题：**
- 大量注释掉的代码（如pyqt相关）
- SIOM类（日志）未使用

### 2.5 zException.py (22620字节，~730行)
**状态：** 异常处理，持续使用

**问题：**
- 代码量较大（730行），但异常处理不应该这么复杂
- 大量调试和注释掉的代码

### 2.6 zrand.py (445字节，~120行)
**状态：** 随机数生成，持续使用

**功能：**
- 生成随机障碍物位置

**问题：** 无

### 2.7 plot.py (26999字节，~584行)
**状态：** 绘图模块，核心功能

**功能：**
- plot_position: 绘制轨迹和目标点
- plot_obstacle: 绘制障碍物
- plot_all_pre_traj: 绘制所有轨迹（用于动画）
- save_traj_image: 保存轨迹图

**问题：**
- 代码较长但结构清晰
- 大量硬编码的参数和样式
- 部分函数如twice_base_line_bool可能是遗留代码

### 2.8 uav.py (30723字节，~1000行)
**状态：** UAV智能体核心类

**功能：**
- BUG路径规划算法
- MPC轨迹跟踪
- 速度控制
- 碰撞检测
- 位置更新管理

**问题：**
- 单个文件1000行，功能过多，职责不单一
- 有多个update方法（update_state, update_mpc, update_bug等）
- 代码重复较多（如多个类似的状态检查）

### 2.9 others.py (4907字节，~160行)
**状态：** 其他工具函数

**功能：**
- save_data: 保存数据（被注释掉）
- check_reach_target: 检查是否到达目标
- calculate_distance: 距离计算

**问题：**
- save_data直接return，不做任何事（已被注释）

### 2.10 trajectory.py (3804字节，~120行)
**状态：** 轨迹处理工具

**功能：**
- land: 轨迹着陆
- 各种轨迹变换函数

**使用情况：** test.py导入但只使用了`land`，其他功能未用

### 2.11 geometry.py (1807字节，~500行)
**状态：** 几何计算

**功能：**
- 各种几何形状计算（Polygon, Circle等）
- 距离和相交检测

**使用情况：** plot.py中使用

### 2.12 inter_avoid.py (693字节，~200行)
**使用情况：** 仅在run.py中导入，可能未使用

### 2.13 obstacle_corridor.py (8976字节，~250行)
**使用情况：** 仅在run.py中导入，可能未使用

### 2.14 connection.py (6560字节，~180行)
**使用情况：** 仅在run.py中导入，可能未使用

### 2.15 bug.py (4771字节，~140行)
**状态：** BUG路径规划算法

**使用情况：** 可能在uav.py中被调用或作为独立使用

### 2.16 bug11.py (3804字节，~110行)
**使用情况：** 未在主要模块中看到导入

### 2.17 zdist.py (3720字节，~110行)
**状态：** 距离计算工具

**使用情况：** 仅在uav.py中导入，可能未使用

### 2.18 COLOR.py (1120字节，~30行)
**状态：** 颜色配置

**使用情况：** plot.py中使用

---

## 三、未使用/冗余代码识别

### 3.1 明确未使用的模块

1. **jpg2mp4.py** - JPG转MP4视频功能
   - test.py未导入
   - 可能是旧的视频生成功能，已不再使用

2. **thread.py** - 线程模块
   - run.py导入但未见实际使用
   - 可能是多进程的遗留代码

3. **test_baseline_bool.py** - baseline对比测试
   - 有特定用途但非常规

4. **run_test.py** - 运行测试
   - 存在但test.py未调用

5. **inter_avoid.py** (~200行)
   - run.py导入但实际功能未知

6. **obstacle_corridor.py** (~250行)
   - run.py导入但实际功能未知

7. **connection.py** (~180行)
   - run.py导入但实际功能未知

8. **bug11.py** (~110行)
   - 未看到导入和使用

9. **zdist.py** (~110行)
   - uav.py导入但实际可能未使用

10. **trajectory.py除land函数外** (~100行代码未用)
    - 导入了整个模块但只用land函数
    - 大量轨迹变换函数未使用

11. **multiprocessing导入** (test.py中)
    - 导入了`import multiprocessing as mp`但未使用

12. **test.py中注释掉的代码**
    - run_one_step函数调用被注释
    - check_reach_target被注释
    - 多处print调试代码

13. **run.py中的碰撞检测代码** (~100行)
    - 可能是旧版本代码，当前可能不再需要

14. **zstatistics.py中的SIOM类** (~100行)
    - 日志功能未使用

15. **plot.py中的twice_base_line_bool** (~50行)
    - 可能是baseline对比的遗留参数

### 3.2 可疑的冗余代码

1. **test.py中的save_data调用**
   - `save_data(agent_list)`后紧跟注释说明"zyt 不要输出"
   - others.py中save_data直接return什么都不做

2. **output_filename.py中的注释代码**
   - 大量pyqt、zpytest相关注释
   - 这些可能是旧版本的GUI测试代码

3. **zstatistics.py中的大量注释**
   - pyqt相关调试代码

4. **run.py的复杂回退逻辑** (~100行)
   - check_one_step函数逻辑复杂，可能需要简化

5. **uav.py的update方法重复**
   - update_state, update_mpc, update_bug等可能存在重复逻辑

---

## 四、调用关系分析

### 4.1 数据流

```
parameters.yaml
    ↓
SET.py (读取配置)
    ↓
uav.py (初始化agents)
    ↓
bug算法 (计算路径)
    ↓
agent.path (计算出的路径)
    ↓
plot.py (绘制)
    ↓
TIOM (保存图像)
    ↓
savefig/trajecotry.jpg
```

### 4.2 模块依赖关系

```
test.py
  ├── output_filename (路径管理)
  ├── zstatistics (统计)
  ├── plot (绘图)
  ├── uav (智能体)
  └── others (工具)

run.py
  ├── zstatistics
  ├── zyaml
  ├── zException
  ├── SET
  ├── uav
  ├── thread (未使用)
  ├── obstacle_corridor (未使用)
  ├── inter_avoid (未使用)
  ├── connection (未使用)
  ├── trajectory
  ├── plot
  ├── geometry
  └── others
```

### 4.3 未解决的问题

1. **test.py的run_one_step被注释**
   - 测试只是初始化agents并打印位置，没有实际运行仿真
   - 这导致agents.position始终是1D，不是历史轨迹

2. **pre_traj_list为空**
   - 因为run_one_step被注释，agents没有更新pre_traj_list
   - 之前修复的pre_traj_list空检查是workaround

---

## 五、重构建议

### 5.1 优先级P0 - 清理明确未使用的模块

**建议删除或移除的文件：**
1. `jpg2mp4.py` - 视频功能已不用
2. `thread.py` - 线程功能未使用
3. `bug11.py` - 确认无使用后删除
4. `zdist.py` - 确认uav.py未使用后删除
5. `inter_avoid.py` - 确认无使用后删除
6. `obstacle_corridor.py` - 确认无使用后删除
7. `connection.py` - 确认无使用后删除

### 5.2 优先级P1 - 清理冗余代码

**建议清理的内容：**
1. **output_filename.py** - 删除所有pyqt、zpytest相关注释
   - 预计减少~100行代码

2. **zstatistics.py** - 删除SIOM类和pyqt注释
   - 预计减少~100行代码

3. **test.py** - 删除注释掉的调试代码
   - 删除未使用的multiprocessing导入

4. **trajectory.py** - 只保留land函数，删除其他未使用代码
   - 预计减少~100行代码

### 5.3 优先级P2 - 简化test.py

**建议改进：**
1. 恢复run_one_step调用或删除相关代码
   - 当前状态：run_one_step被注释导致agents不更新
   - 建议：要么恢复仿真循环，要么简化为纯路径规划+绘图

2. 简化fstatistics调用
   - 添加空plan_time_list的检查
   - 或者干脆跳过统计步骤

### 5.4 优先级P3 - 模块重构

**长期目标：**
1. **uav.py拆分** - 当前1000行，功能过多
   - 建议：拆分为多个小模块
     - `uav_agent.py` - 核心agent类（状态、目标）
     - `uav_bug_planner.py` - BUG路径规划
     - `uav_mpc_tracker.py` - MPC轨迹跟踪
     - `uav_collision.py` - 碰撞检测

2. **统一配置管理** - SET.py和output_filename.py功能有重叠
   - 建议：合并或明确职责分工

3. **标准化绘图接口** - plot.py参数和函数过于复杂
   - 建议：提供更简洁的绘图API

### 5.5 优先级P4 - 代码质量

**建议：**
1. 添加类型提示（Type Hints）
2. 添加单元测试
3. 添加文档字符串（docstring）
4. 改进变量命名（如`qtplan`改为`TEST_PLAN`）

---

## 六、分阶段重构计划

### 阶段1：清理（1-2天）
- [ ] 删除明确未使用的模块（P0）
- [ ] 清理冗余注释和代码（P1）
- [ ] 提交清理后的代码

### 阶段2：简化（2-3天）
- [ ] 简化test.py主逻辑（P2）
- [ ] 简化fstatistics统计逻辑（P2）
- [ ] 测试确保功能正常

### 阶段3：拆分（3-5天，可选）
- [ ] 拆分uav.py为多个模块（P3）
- [ ] 更新所有导入
- [ ] 测试确保功能正常

### 阶段4：优化（可选，后续）
- [ ] 添加单元测试
- [ ] 添加类型提示
- [ ] 改进文档

---

## 七、重构风险评估

| 风险 | 描述 | 缓解措施 |
|------|------|----------|
| 低   | 删除未使用模块 | 先备份到archive目录 |
| 中   | 简化test.py | 保持原有测试可用 |
| 高   | 拆分uav.py | 分阶段进行，充分测试 |

---

## 八、当前待解决问题

1. ✅ test.py - intermediate_logs IndexError - 已修复（使用agent.p）
2. ✅ test.py - pre_traj_list空检查 - 已添加处理逻辑
3. ✅ test.py - fstatistics空plan_time_list - 已注释
4. ✅ output_filename.py - 路径重复前缀 - 已修复
5. ✅ output_filename.py - str2time带前缀 - 已修复
6. ✅ plot.py - position 1D数组 - 已修复（使用agent.path）
7. ✅ 成功生成无障碍物轨迹图并提交GitHub

---

## 附录：代码统计

| 文件 | 行数(估计) | 字节数 | 状态 |
|------|-------------|--------|------|
| SET.py | 180 | 6.3KB | 正常 |
| output_filename.py | 320 | 12.7KB | 需清理 |
| zstatistics.py | 300 | 4.6KB | 需清理 |
| zyaml.py | 25 | 0.8KB | 正常 |
| zException.py | 730 | 22.6KB | 需审查 |
| zrand.py | 120 | 0.4KB | 正常 |
| plot.py | 584 | 26.4KB | 需简化 |
| uav.py | 1000 | 30.7KB | 需拆分 |
| run.py | 812 | 31.5KB | 需审查 |
| test.py | 86 | 3.4KB | 正常 |
| **合计** | **~4400** | **~172KB** | |

**总未使用模块：** ~2000行（~80KB）
**可清理冗余：** ~1000行（~40KB）
