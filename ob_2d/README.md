# OB_2D - 多智能体路径规划系统

## 📋 项目概述

这是一个基于模型预测控制（MPC, Model Predictive Control）的多智能体路径规划仿真系统。系统在二维空间中模拟多个智能体（如无人机、机器人）从起点到终点的路径规划，同时满足：

- **碰撞避免**：智能体之间避免相互碰撞
- **障碍物避让**：智能体避开环境中的静态障碍物
- **速度约束**：智能体运动速度不超过最大限制
- **加速度约束**：智能体加速度控制在安全范围内

## 🎯 核心功能

1. **路径规划**：使用 MPC 算法进行在线路径规划
2. **障碍物处理**：支持方形障碍物的碰撞检测和避让
3. **统计分析**：记录规划成功率、碰撞率、死锁率等指标
4. **可视化**：生成轨迹图片和视频
5. **批量测试**：支持批量运行不同参数组合的测试

## 🚀 快速开始

### 1. 打开项目

在 VS Code 中打开项目目录：

```bash
# Windows
code D:\zyt\git_ln\path_agent

# 或者通过 VS Code 文件菜单 -> 打开文件夹
```

### 2. 安装依赖

首次打开项目后，VS Code 会自动提示安装依赖，也可以手动运行：

```bash
cd D:\zyt\git_ln\path_agent
uv sync
```

### 3. 运行方式

#### 🎯 方式一：VS Code 调试运行（推荐）

在 VS Code 中使用 F5 启动调试器运行程序。

可用的启动配置：
- **OB_2D - 默认运行**：使用默认参数运行，不显示图形
- **OB_2D - 显示图形**：运行并显示轨迹图形
- **OB_2D - 指定参数**：使用指定的参数文件运行

切换启动配置：
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `Debug: Select and Start Debug Configuration`
3. 选择想要的配置
4. 按 `F5` 启动

#### 🎯 方式二：命令行运行

```bash
cd D:\zyt\git_ln\path_agent\ob_2d

# 最简单的运行（使用默认参数）
uv run python test.py

# 带参数运行
uv run python test.py 2026-03-01_12-00-00 004/2026-03-01_12-00-00/parameters.yaml show_pict
```

#### 🎯 方式三：终端运行

```bash
cd D:\zyt\git_ln\path_agent\ob_2d
D:\zyt\git_ln\path_agent\.venv\Scripts\python.exe test.py
```

### 默认参数说明

不带参数运行时，程序使用以下默认值：

| 参数 | 默认值 |
|------|--------|
| 智能体数量 | 3 |
| 地图范围 | 50x50 |
| 障碍物数量 | 1 |
| 最大速度 | 6 |
| 最大加速度 | 8 |
| 时间步长 | 0.15 |
| 最大回合数 | 330 |

这些默认值在 `SET.py` 和 `zrand.py` 中定义，可以根据需要修改。

## 📁 项目结构

```
path_agent/
├── ob_2d/                      # 主代码目录
│   ├── test.py                   # 主程序入口
│   ├── run.py                    # 运行主循环（每步规划）
│   ├── uav.py                    # 智能体类定义
│   ├── SET.py                    # 全局参数设置
│   ├── zrand.py                  # 随机场景生成
│   ├── zyaml.py                  # 参数配置管理
│   ├── zstatistics.py             # 统计分析模块
│   ├── output_filename.py          # 文件输出管理
│   ├── plot.py                   # 绘图功能
│   ├── geometry.py                # 几何计算
│   └── bug*.py                   # Bug 路径规划相关算法
├── shared_util/                  # 共享工具库
│   ├── io_filename.py             # I/O 文件管理
│   ├── sys_argument.py            # 命令行参数解析
│   └── platform_interpreter.py     # 平台适配
├── pyproject.toml                # uv 项目配置
└── .venv/                      # Python 虚拟环境
```

## 📊 输出说明

程序运行后，结果保存在 `004/<时间戳>/` 目录下：

### 输出文件

| 文件/目录 | 说明 |
|-----------|------|
| `agent100/` | 智能体结果数据 |
| `agent_list[i].json` | 每个智能体的详细状态（位置、速度、轨迹等） |
| `agent_list_100.pkl` | 所有智能体的 pickle 序列化数据 |
| `a_statistics.json` | 统计结果（成功率、碰撞率、运行时间等） |
| `savefig/` | 轨迹图片（如果启用显示） |

### 关键指标

- **Success Rate**: 成功率，所有智能体到达目标点的比例
- **Collision Rate**: 智能体间碰撞率
- **External Collision Rate**: 智能体与障碍物碰撞率
- **Deadlock Crack Rate**: 死锁破解率
- **Average Planning Time**: 平均每次规划时间

## ⚙️ 参数配置

### 环境参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `Num` | 智能体数量 | 3 |
| `set_xlim` / `set_ylim` | 地图范围 | 50 |
| `ob_num` | 障碍物数量 | 1 |
| `Vmax` | 最大速度 | 6 |
| `Umax` | 最大加速度 | 8 |
| `radius` | 安全半径 | 0.6 |
| `episodes` | 最大回合数 | 330 |

### MPC 参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `K` | 预测视野长度 | 8 |
| `h` | 时间步长 | 0.15 |
| `epsilon` | 安全间隙 | 0.20 |

## 🔧 修改说明

为使项目能够独立运行，对以下文件进行了修改：

1. **`shared_util/sys_argument.py`**
   - 修复 `test.py` 无参数时获取日期会报错的问题
   - 当没有传入日期参数时，使用当前时间戳

2. **`shared_util/platform_interpreter.py`**
   - 修改为使用 `sys.executable`
   - 确保子进程使用与父进程相同的 Python 解释器（uv 虚拟环境）

3. **`ob_2d/output_filename.py`**
   - 修改为使用 `sys.executable`
   - 确保视频生成等子进程使用正确的 Python 环境

4. **`ob_2d/zrand.py`**
   - 添加默认起点/终点生成逻辑
   - 当配置文件不存在时，自动生成随机的起点和终点

## 📝 Python 依赖

主要依赖包：

| 包名 | 用途 |
|------|------|
| cvxopt | 凸优化求解器 |
| cvxpy | 凸优化框架 |
| numpy | 数值计算 |
| scipy | 科学计算（线性代数） |
| matplotlib | 数据可视化 |
| opencv-python | 视频处理 |
| pyyaml | 配置文件解析 |
| shapely | 几何运算 |

## 🛠️ VS Code 调试配置

项目已包含 `.vscode/launch.json` 配置文件，提供以下启动选项：

### 启动配置说明

| 配置名称 | 说明 | 参数 |
|---------|------|------|
| OB_2D - 默认运行 | 使用默认参数运行，不显示图形 | `[]` |
| OB_2D - 显示图形 | 运行并显示轨迹图形 | `["show_pict"]` |
| OB_2D - 指定参数 | 使用指定的参数文件运行 | `["2026-03-01_12-00-00", "ob_2d/004/2026-03-01_12-00-00/parameters.yaml", "show_pict"]` |

### 修改启动参数

如果需要自定义启动参数：

1. 打开 `.vscode/launch.json`
2. 修改 `"args"` 数组中的参数
3. 保存后重新运行

### 示例

```json
{
    "name": "OB_2D - 自定义参数",
    "program": "${workspaceFolder}/ob_2d/test.py",
    "cwd": "${workspaceFolder}",
    "console": "integratedTerminal",
    "args": [
        "2026-03-01_12-00-00",        // 日期时间戳
        "ob_2d/004/2026-03-01_12-00-00/parameters.yaml",  // 参数文件路径
        "show_pict"                     // 显示图形
    ]
}
```

## 🐛 故障排查

### 1. ModuleNotFoundError

如果遇到模块导入错误：

```bash
# 确保已安装依赖
cd D:\zyt\git_ln\path_agent
uv sync
```

### 2. 虚拟环境问题

如果子进程使用了错误的 Python 解释器：

- 检查 `sys_argument.py` 和 `platform_interpreter.py` 中的 `sys.executable` 配置

### 3. 文件路径错误

如果遇到文件找不到错误：

- 确保 `obstacle_list_list2D.json` 存在 `ob_2d` 目录下
- 确认输出目录 `004/` 有写权限

## 📚 参考资料

- 模型预测控制（MPC）：模型预测未来状态并优化控制输入
- 凸优化：将路径规划问题转化为凸优化问题求解
- CVXOPT/CVXPY：Python 凸优化库

## 📄 许可

本项目是从 MPC2 项目中抽取的独立模块，请遵循原项目的许可协议。
