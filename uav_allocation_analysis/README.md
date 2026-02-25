# CRITICALPATH算法 - UAV任务分配完整分析

## 项目概述

本项目展示了 **CRITICALPATH算法** 在异构多无人机协同任务分配与调度问题（H-MUTASP）中的完整分析结果。

- **总UAV数量**: 80
- **总任务数量**: 30
- **Makespan**: 898.10s
- **计算时间**: 0.578s
- **约束违规**: 0

---

## 文件清单

### 📊 数据文件

| 文件名 | 大小 | 说明 |
|--------|------|------|
| `result_criticalpath_new.json` | 99KB | CRITICALPATH算法调度结果（任务时间、UAV分配） |
| `precomputed_data.json` | 44KB | 任务和UAV详细信息（含位置、半径、技能等） |
| `env_data.yaml` | 6.2KB | 环境配置（任务位置、约束、基地配置） |
| `env_pydantic.py` | 2.7KB | 数据模型定义 |

#### result_criticalpath_new.json 结构
```json
{
  "makespan": 898.10,              // 最大完工时间
  "computation_time": 0.578,        // 计算时间
  "violations": 0,                   // 约束违规数
  "task_schedule": { ... },           // 任务调度 {task_id: {start, end}}
  "uav_schedule": { ... }            // UAV调度 {uav_id: [{task, target, start, end, travel_start, travel_end}]}
}
```

#### precomputed_data.json 关键字段
```json
// 任务信息
{
  "task_id": 0,
  "type": "capture",              // 任务类型: capture / attack / surveillance
  "center": [x, y],              // 任务中心位置
  "radius": 3.07,               // 任务半径
  "targetA_num": 11,             // A阶段所需UAV数
  "targetB_num": 10,             // B阶段所需UAV数
  "skills_A": [2],               // A阶段所需技能
  "skills_B": [3]                // B阶段所需技能
}

// UAV信息
{
  "uav_id": 0,
  "base": "base1",              // 所属基地
  "type": "uavA",              // UAV类型: uavA / uavB / uavC
  "skills": [1, 2],            // UAV技能列表
  "init_pos": [x, y]            // 初始位置
}
```

---

### 🖼️ 可视化图表

| 文件名 | 大小 | 说明 |
|--------|------|------|
| `criticalpath_complete_analysis.png` | 178KB | **完整综合分析图**（8个子图） |
| `criticalpath_allocation_full.png` | 727KB | UAV任务分配完整分析（前80个UAV甘特图） |
| `criticalpath_allocation_analysis.png` | 393KB | 简版UAV分配分析 |

#### criticalpath_complete_analysis.png 包含的子图

1. **任务位置分布图** - 显示30个任务的空间位置、半径和类型
   - 🔵 侦察 (surveillance) - 金色
   - 🔴 攻击 (attack) - 红色
   - 🟢 捕获 (capture) - 青色

2. **UAV初始位置分布** - 显示80个UAV从两个基地的初始分布

3. **每个UAV分配的任务数量** - 80个UAV各自的任务分配量统计

4. **每个任务分配的UAV数量** - 30个任务各自A/B阶段的UAV分配统计

5. **同时性约束** - 需同时执行的任务组
   - 组0: Task [4, 6]
   - 组1: Task [8, 9, 10]
   - 组2: Task [18, 19]

6. **每个任务的执行时间** - 30个任务的执行时长统计

7. **UAV移动轨迹** - 前20个UAV从基地到任务的移动路径

8. **UAV任务执行甘特图** - 全部80个UAV的时间线
   - 灰色虚线：旅行时间
   - 彩色实线：任务执行时间
   - 配色：A阶段/B阶段差异色

---

### 📝 分析报告

| 文件名 | 大小 | 说明 |
|--------|------|------|
| `complete_analysis_report.txt` | 2.7KB | 完整分析报告（含核心指标、任务位置、UAV位置） |
| `allocation_details_full.txt` | 39KB | 80个UAV的完整任务分配详情 |
| `allocation_details.txt` | 27KB | 简版任务分配详情 |

---

### 💻 Python代码

| 文件名 | 大小 | 说明 |
|--------|------|------|
| `plot_complete_analysis.py` | 14KB | **主脚本** - 生成完整综合分析图 |
| `plot_allocation_full.py` | 9.8KB | 生成UAV分配完整分析图 |
| `plot_allocation.py` | 7.8KB | 生成基础UAV分配分析图 |

#### 运行方式
```bash
# 进入目录
cd uav_allocation_analysis

# 生成完整分析图（推荐）
python plot_complete_analysis.py
```

---

## 配色方案

| 任务类型 | A阶段 | B阶段 |
|---------|--------|--------|
| 侦察 (surveillance) | #FFD700 金色 | #FFA500 橙色 |
| 攻击 (attack) | #FF6B6B 浅红 | #EE5A5A 深红 |
| 捕获 (capture) | #4ECDC4 青色 | #44A08D 深青 |

| UAV类型 | 颜色 |
|---------|------|
| UAV A | #e74c3c 红色 |
| UAV B | #3498db 蓝色 |
| UAV C | #2ecc71 绿色 |

---

## 算法对比结果

| 算法 | Makespan | 计算时间 | 综合得分 | 评价 |
|------|----------|---------|----------|------|
| **CRITICALPATH** | **898.10s** | 0.58s | 0.935 | 优秀 |
| SA | 912.91s | 16.18s | 0.777 | 良好 |
| GREEDY | 949.85s | 0.60s | 0.411 | 一般 |
| LPT | 968.85s | 0.46s | 0.298 | 较差 |

---

## 数据结构说明

### 任务类型 (Task Type)
- **surveillance**: 侦察任务
- **attack**: 攻击任务
- **capture**: 捕获任务

### UAV类型 (UAV Type)
- **uavA**: A型无人机，具备技能 [1, 2]
- **uavB**: B型无人机，具备技能 [2, 3]
- **uavC**: C型无人机，具备技能 [1, 3]

### 任务阶段 (Task Phase)
- **A阶段**: 任务的第一阶段，需特定技能的UAV执行
- **B阶段**: 任务的第二阶段，在A阶段完成后执行

### 约束类型
1. **能力匹配约束**: UAV技能必须覆盖任务所需技能
2. **优先级约束**: 后继任务必须在前置任务完成后才能开始
3. **同时性约束**: 特定任务组必须由不同UAV同时执行

---

## 生成日期

2026-02-26
