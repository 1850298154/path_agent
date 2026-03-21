# UAV-Task Capability Matrix 说明

## 图表概述

该矩阵图展示了 **80个UAV** 与 **30个任务** 之间的能力匹配关系，清晰呈现每个UAV能够执行哪些任务的哪些阶段。

## 坐标轴说明

- **横轴 (X轴)**: Task ID (0-29)，代表30个待执行的任务
- **纵轴 (Y轴)**: UAV ID (0-79)，代表80个无人机

## 颜色含义

矩阵中的颜色与 `criticalpath_allocation_full.png` 保持一致：

| 颜色 | 任务类型 | 说明 |
|------|----------|------|
| 🟡 金色 (Gold) | Surveillance (侦察) | 需要特定技能进行监视侦察任务 |
| 🔴 红色 (Red) | Attack (攻击) | 需要特定技能进行攻击任务 |
| 🔵 青色 (Cyan) | Capture (捕获) | 需要特定技能进行捕获任务 |
| ⬜ 灰色 | 无能力 | 该UAV无法执行该任务 |

## 单元格结构

每个单元格分为左右两部分：
- **左半部分**: 表示该UAV能否执行该任务的 **A阶段**
- **右半部分**: 表示该UAV能否执行该任务的 **B阶段**

## UAV类型与能力分布

### uavA (技能: [1, 2])
- **UAV ID**: 0-9, 40-49 (共20个)
- **可执行任务**:
  - Attack: A阶段 ✓, B阶段 ✓
  - Surveillance: A阶段 ✓, B阶段 ✗
  - Capture: A阶段 ✓, B阶段 ✗

### uavB (技能: [1, 3])
- **UAV ID**: 10-19, 50-59 (共20个)
- **可执行任务**:
  - Attack: A阶段 ✓, B阶段 ✓
  - Surveillance: A阶段 ✗, B阶段 ✓
  - Capture: A阶段 ✗, B阶段 ✓

### uavC (技能: [2, 3])
- **UAV ID**: 20-39, 60-79 (共40个)
- **可执行任务**:
  - Attack: A阶段 ✗, B阶段 ✗
  - Surveillance: A阶段 ✓, B阶段 ✓
  - Capture: A阶段 ✓, B阶段 ✓

## 技能与任务需求对应关系

| 任务类型 | A阶段所需技能 | B阶段所需技能 |
|----------|---------------|---------------|
| Surveillance | skill 2 | skill 3 |
| Attack | skill 1 | skill 1 |
| Capture | skill 2 | skill 3 |

## 关键观察

1. **Attack任务** (红色): 仅 uavA 和 uavB 可执行，uavC 无法执行任何Attack任务

2. **Surveillance/Capture任务** (金色/青色): uavC 可完整执行两个阶段，uavA 只能执行A阶段，uavB 只能执行B阶段

3. **互补性**: uavA 和 uavB 在 Surveillance/Capture 任务上形成互补关系，合作可完成完整任务

4. **资源分布**: uavC 数量最多(40个)，专门负责 Surveillance 和 Capture 任务
