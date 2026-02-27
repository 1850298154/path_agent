# 调度管理器 - 使用说明

## 问题说明

当前 `test.py` 只回放预先计算的轨迹，不进行实时路径规划。
需要在 UAV 调度完成后，才触发智能体路径规划。

## 架构设计

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    UAV 调度系统 (uav_positions_over_time.json) │
├─────────────────────────────────────────────────────────────────────────┤
│  监控 UAV 状态 → 确定完成后触发路径规划          │
└─────────────────────────────────────────────────────────────────────────┘
                          ↓ RPC 信号 (端口 12345)
                    ↓ 手动触发
                    ↓ 自动检测
┌─────────────────────────────────────────────────────────────────────────┐
│         调度管理器 (scheduler.py)            │
│  - 接收 UAV 完成信号                          │
│ - 检查状态                                       │
│ - 触发路径规划                                   │
└─────────────────────────────────────────────────────────────────────────┘
                          ↓ 触发命令
                    ↓ 保存测试配置
                    ↓ 执行 test.py (在 obstacle_2d_minimal/)
┌─────────────────────────────────────────────────────────────────────────┐
│         智能体路径规划 (test.py + run.py)            │
│  - 读取 pkl 文件中的轨迹                          │
│ - 调用 run_one_step 进行实时避障                   │
│ - 保存结果                                        │
└─────────────────────────────────────────────────────────────────────────┘
```

## 使用步骤

### 1. 启动调度管理器

```bash
cd uav_allocation_analysis
python scheduler.py
```

### 2. 使用命令

#### 自动 RPC 模式

```bash
# 启动 RPC 服务器（接收 UAV 完成信号）
调度> start_rpc

# RPC 服务器会自动检测 UAV 状态，全部完成后自动触发路径规划
```

#### 本地手动模式

```bash
# 启动本地手动模式
调度> start_manual

# 手动检查和触发
调度> check

# 查看状态
调度> status
```

### 3. 文件结构

```
uav_allocation_analysis/
├── scheduler.py            # 调度管理器主程序
├── SCHEDULER_README.md     # 调度管理器文档
├── run_scheduled.py        # 测试执行脚本
├── test_parameters.yaml     # 手动触发的测试配置
├── scheduled_results/       # 自动调度的测试结果
│   └── <timestamp>/       # 调度时间戳目录
│       ├── test_config.json # 测试配置
│       └── parameters.yaml   # 测试参数
├── uav_positions_over_time.json  # UAV位置数据
└── allocation_details.txt   # UAV分配详情
```

### 4. 配置文件格式

#### test_parameters.yaml

```yaml
# UAV 目标位置
agent_target:
  - [30, 30]   # UAV 0
  - [300, 30]  # UAV 1
  - [30, 300]  # UAV 2
  # ... 其他 UAV

# 路径规划参数
path_planning:
  # 测试场景描述
  scenario_name: "10智能体9障碍物路径规划"

  # 从调度管理器传递的参数
  scheduled_request_id: "auto"
  scheduled_timestamp: "2026-02-27_14-30-00"

# 障碍物配置（可选）
obstacles:
  - [ [50, 100, 30], [200, 50], ... ]
```

### 5. RPC 通信协议

#### 客户端发送

```
格式: <uav_id>:<status>

状态值:
- idle: 空闲
- to_start: 准备起飞
- in_flight: 飞行中
- reached: 到达目标（可触发路径规划）
- returning: 返航中

示例:
0:idle        # UAV 0 空闲
1:reached     # UAV 1 到达目标
```

#### 服务器广播

```
格式: <uav_id1>:<status>|<uav_id2>:<status>|...

示例:
0:reached|1:idle|2:in_flight|3:reached  # 3 个 UAV 完成状态
```

### 6. API 接口

#### 触发路径规划

```
POST /trigger
Content-Type: application/json

{
  "request_id": "manual_trigger_001",
  "test_config": {
    "agent_target": [...]
  }
}
```

#### 查询状态

```
GET /status
Response:
{
  "mode": "local_manual",
  "monitoring": true,
  "uav_status": {
    "IDLE": 0,
    "TO_START": 0,
    "IN_FLIGHT": 5,
    "REACHED": 3,
    "RETURNING": 0
  },
  "timestamp": 1234567890.123
}
```

## 工作流程

### 完整流程

1. **UAV 调度阶段**
   ```
   UAV 1 ──→ 到达目标 ──→ 返航 ──→ 空闲
   UAV 2 ─────────────────────────→→ 到达目标 ──────
   UAV 3 ─────────────────────────────→ 到达目标
   ```

2. **调度触发**
   ```
   调度器检测所有 UAV 到达
   → 等待 10 秒（可选延迟）
   → 触发路径规划
   ```

3. **路径规划执行**
   ```
   调度器生成测试配置
   → 调用 test.py
   → test.py 调用 run.py 的 run_one_step()
   → 实时避障计算
   → 保存结果
   ```

## 关键优势

1. **解耦**: UAV 系统和智能体系统完全分离
2. **灵活**: 支持多种触发方式（自动/手动/RPC）
3. **可控**: 可以精确控制何时开始路径规划
4. **可扩展**: 容易添加更多功能（如优先级、队列管理）

## 故障处理

- 如果 UAV 未完成就触发路径规划 → 路径规划会失败
- 如果 test.py 中的 run_one_step 有 bug → 影响结果

## 后续优化方向

1. 添加任务队列，支持批量调度
2. 添加优先级管理
3. 添加重试机制
4. 添加实时日志推送
