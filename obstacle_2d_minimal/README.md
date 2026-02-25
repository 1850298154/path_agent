# 2D 障碍物环境多智能体路径规划 (OB_2D)

这是一个从原项目中抽取出来的最小化独立版本，用于 2D 障碍物环境中的多智能体路径规划。

## 功能

- 多智能体协同导航
- 动态障碍物避障
- 碰撞检测与避免
- 路径规划与优化
- 统计分析与可视化

## 目录结构

```
obstacle_2d_minimal/
├── test.py                      # 主入口文件
├── run_test.py                  # 简单测试脚本
├── shared_util/                 # 共享工具模块
│   ├── io_filename.py           # 文件名和路径管理
│   └── sys_argument.py          # 系统参数解析
├── uav.py                      # 智能体类定义
├── geometry.py                  # 几何计算
├── plot.py                     # 绘图功能
├── run.py                      # 运行主循环
├── SET.py                      # 参数设置
├── ... (其他模块)
└── 004/                        # 测试数据目录
    └── 2026-02-26_06-00-00/
        ├── description.json      # 场景描述（起点、终点、障碍物）
        ├── parameters.yaml       # 模型参数
        └── agent100/           # 预生成轨迹数据
```

## 快速开始

### 1. 安装依赖

```bash
pip install numpy matplotlib opencv-python shapely cvxopt scipy
```

### 2. 运行测试

使用简单测试脚本：
```bash
python run_test.py
```

或直接运行主程序：
```bash
python test.py <时间戳> <参数文件路径> <显示模式>
```

示例：
```bash
python test.py 2026-02-26_06-00-00 004/2026-02-26_06-00-00/parameters.yaml not_show
```

### 3. 查看结果

运行完成后，结果会保存在 `004/<时间戳>/` 目录下：
- `savefig/` - 生成的图片
- 统计信息会在控制台输出

## 测试场景

当前测试场景：
- **智能体数量**: 10
- **障碍物数量**: 9 个圆形障碍物
- **地图尺寸**: 330 x 330
- **智能体速度**: Vmax = 3.0
- **智能体加速度**: Umax = 10.0

## 统计指标

| 指标 | 说明 |
|------|------|
| 成功率 (success_rate) | 成功到达终点的智能体比例 |
| 平均规划时间 | 每次重新规划的平均耗时 |
| 内部碰撞率 (collision_rate2) | 智能体之间的碰撞率 |
| 外部碰撞率 (ex_collision_rate2) | 与障碍物的碰撞率 |
| 死锁率 (deadlock_rate) | 发生死锁的比例 |

## 文件说明

### 核心文件

- `test.py` - 主程序入口
- `uav.py` - 智能体（UAV）类定义
- `run.py` - 仿真运行主循环
- `geometry.py` - 几何计算工具
- `plot.py` - 结果可视化

### 工具模块

- `output_filename.py` - 文件名和路径管理
- `SET.py` - 全局参数配置
- `zyaml.py` - YAML 配置文件读写
- `zrand.py` - 随机数生成
- `zstatistics.py` - 统计数据收集

### 算法模块

- `bug.py` / `bug11.py` - BUG 路径规划算法
- `inter_avoid.py` - 智能体间避障
- `obstacle_corridor.py` - 障碍物走廊生成
- `connection.py` - 连接约束处理

## 注意事项

1. 确保 Python 版本为 3.12 或以上
2. 所有依赖库已正确安装
3. 测试数据目录结构完整
4. Windows 系统下路径分隔符使用 `/` 或 `\\`

## 许可

本代码从原 MPC 项目中抽取，仅用于学习和研究目的。
