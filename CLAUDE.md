# Claude 工作规范

## Git 提交规范

### 1. 提交消息格式 (Markdown)

每次 git commit 必须使用清晰的 Markdown 格式，包含：

```
<类型>: <简短标题>

## 背景
- 为什么做这个修改
- 之前有什么问题

## 改动内容
### 文件变更
- `路径/文件1`: 改动描述
- `路径/文件2`: 改动描述

### 参数变更
| 参数名 | 原值 | 新值 | 原因 |
|-------|------|------|------|
| xxx | xxx | xxx | xxx |

## 输出说明
### 输出路径
- 旧: `旧路径`
- 新: `新路径/2026-XX-XX_XX-XX-XX/`

### 输出内容变化
- 新增了什么
- 修改了什么
- 删除了什么

## 验证方法
- 如何验证改动是正确的
```

### 2. 提交类型

| 类型 | 说明 |
|------|------|
| `feat` | 新功能 |
| `fix` | 修复bug |
| `refactor` | 重构代码（不变功能）|
| `test` | 测试运行 |
| `docs` | 文档更新 |
| `config` | 配置修改 |

### 3. 示例

```
feat: 添加多样化UAV场景测试

## 背景
用户反馈"路径一直一样，不知道是否实时计算"，需要验证路径规划算法是否在工作。

## 改动内容
### 文件变更
- `obstacle_2d_minimal/generate_diverse_scenario.py`: 创建多样化场景生成器
- `obstacle_2d_minimal/004/diverse_consistent/parameters.yaml`: 创建新的测试配置
- `obstacle_2d_minimal/004/diverse_consistent/description.json`: 场景描述文件

### 参数变更
| 参数名 | 原值 | 新值 | 原因 |
|-------|------|------|------|
| agent.Num | 10 | 10 | 保持不变 |
| ini_x | 相同起点 | 不同起点 | 验证路径差异化 |
| target | 相同终点 | 不同终点 | 验证路径差异化 |

## 输出说明
### 输出路径
- 旧: `obstacle_2d_minimal/004/2026-02-26_11-25-00/`
- 新: `obstacle_2d_minimal/004/diverse_consistent/`

### 输出内容变化
- 新增: 10个智能体的路径数据
  - Agent 0: [30,30]→[45,280], 2个路径点
  - Agent 1: [30,30]→[100,290], 8个路径点（复杂避障）
  - Agent 4: [100,30]→[280,100], 10个路径点（复杂避障）
  - Agent 7: [300,30]→[150,50], 4个路径点
- 证明: 每个智能体有不同路径，算法在实时计算

## 验证方法
1. 检查 agent_list[i].json 中的 path 字段
2. 确认不同agent的路径点数和坐标不同
3. 确认复杂路径的agent确实在绕过障碍物
```

---

## 输出路径管理

### 1. 时间戳命名规范

**重要：每次运行测试，输出目录必须包含时间戳，避免覆盖！**

格式：`YYYY-MM-DD_HH-MM-SS`

示例：
```
obstacle_2d_minimal/
├── 004/
│   ├── 2026-02-26_11-25-00/      # 第一次运行
│   ├── 2026-02-26_14-55-00/      # 第二次运行
│   ├── 2026-02-26_15-10-43/      # 第三次运行
│   └── diverse_consistent/            # 静态配置文件（不包含时间戳）
└── 005/
    └── 2026-02-26_15-20-00/      # UAV引导场景
```

### 2. 新建输出目录的规则

```python
import datetime
import os

def get_timestamped_output_dir(base_dir, sub_dir):
    """创建带时间戳的输出目录"""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = os.path.join(base_dir, sub_dir, timestamp)
    os.makedirs(output_dir, exist_ok=True)
    return output_dir

# 使用
output_dir = get_timestamped_output_dir("obstacle_2d_minimal/004", "test")
# 输出: obstacle_2d_minimal/004/test/2026-02-26_16-30-45/
```

### 3. 删除旧输出前先确认

```python
def clean_old_output(output_dir, confirm=False):
    """清理旧输出目录"""
    if os.path.exists(output_dir):
        if not confirm:
            print(f"警告: 将删除 {output_dir}")
            response = input("确认删除? (y/n): ")
            if response.lower() != 'y':
                return False
        import shutil
        shutil.rmtree(output_dir)
    return True
```

## 代码重构纪律

### 重构原则
1. **原子性**: 每次只修改一个文件，修改后立即测试
2. **可验证**: 改动后运行测试，确认功能正常
3. **渐进式**: 逐步重构，不要贪多一次改很多文件
4. **文档记录**: 每个改动的目的、测试方法、测试结果都记录清楚

### 测试检查清单
每次修改后必须确认：
- [ ] 代码语法正确（Python可以import）
- [ ] 运行一次测试（使用默认参数）
- [ ] 输出文件正常生成
- [ ] 没有引入新的bug或错误
- [ ] 功能与改动前一致

### 测试记录模板

\`\`\`markdown
## <日期> <文件名>

### 改动内容
<描述改动的具体内容>

### 测试过程
1. 测试命令: \`cd obstacle_2d_minimal && python test.py "004/2026-02-26_XX-XX-XX-XX/parameters.yaml" not_show\`
2. 预期结果: <描述预期输出>
3. 实际结果: <实际输出情况>
4. 测试状态: ✅ 通过 / ❌ 失败

### 测试结论
<测试是否成功，如有问题记录>
\`\`\`

### 已完成的重构记录

| 日期 | 文件 | 类型 | 描述 | 状态 |
|------|------|------|------|
| 2026-02-26 | jpg2mp4.py | 删除 | 待开始 |
---

---

## 工作检查清单

### ✅ 工作完成时必须检查：

- [ ] Git commit 消息是 Markdown 格式
- [ ] Git commit 清楚说明了改了什么
- [ ] Git commit 说明了输出路径变化
- [ ] Git commit 说明了输出内容变化
- [ ] 输出目录使用了时间戳（除非是配置文件）
- [ ] 没有覆盖之前的测试结果
- [ ] 代码修改有明确的目的和说明

### ⚠️ 避免的错误：

| 错误 | 后果 | 正确做法 |
|------|------|----------|
| 覆盖之前的结果 | 无法对比，丢失数据 | 使用时间戳目录 |
| 提交消息不清 "update" | 不知道改了什么 | 用 Markdown 详细说明 |
| 没有说明输出路径 | 找不到结果 | 在 commit 中写明输出目录 |
| 不验证就提交 | 可能有 bug | 先验证再提交 |

---

## 当前工作记录

### 最近提交

| Commit ID | 时间 | 类型 | 标题 | 输出路径 |
|-----------|------|------|------|----------|
| 95f0278 | 2026-02-26 | feat | 多样化路径场景 - 验证路径实时计算 | `004/diverse_consistent/` |
| b650639 | 2026-02-26 | feat | 添加UAV引导障碍物路径规划功能 | `005/uav_guided_scenario/` |

### 待验证项

- [ ] 多样化场景的路径可视化图片是否生成
- [ ] Agent 路径差异是否足够明显

---

## 快速参考

### 常用命令

```bash
# 检查 git 状态
git status

# 提交（Markdown 格式）
git commit -m "$(cat <<'EOF'
feat: 标题

## 背景
- 说明原因

## 改动内容
### 文件变更
- `path/file`: 改动描述

## 输出说明
### 输出路径
- 新: `output/path/`

### 输出内容变化
- 说明
EOF
)"

# 查看最近提交
git log --oneline -5

# 对比两个提交
git diff HEAD~1 HEAD
```

### 路径速查

| 场景类型 | 目录路径 | 说明 |
|----------|----------|------|
| 基础测试 | `obstacle_2d_minimal/004/2026-02-26_11-25-00/` | 最初的测试 |
| UAV引导 | `obstacle_2d_minimal/005/uav_guided_scenario/` | 从UAV分析生成 |
| 多样化验证 | `obstacle_2d_minimal/004/diverse_consistent/` | 验证路径差异化 |

## 代码重构纪律

### 重构原则
1. **原子性**: 每次只修改一个文件，修改后立即测试
2. **可验证**: 改动后运行测试，确认功能正常
3. **渐进式**: 逐步重构，不要贪多一次改很多文件
4. **文档记录**: 每个改动的目的、测试方法、测试结果都记录清楚

### 测试检查清单
每次修改后必须确认：
- [ ] 代码语法正确（Python可以import）
- [ ] 运行一次测试（使用默认参数）
- [ ] 输出文件正常生成
- [ ] 没有引入新的bug或错误
- [ ] 功能与改动前一致

### 测试记录模板

\`\`\`markdown
## <日期> <文件名>

### 改动内容
<描述改动的具体内容>

### 测试过程
1. 测试命令: \`cd obstacle_2d_minimal && python test.py "004/2026-02-26_XX-XX-XX-XX/parameters.yaml" not_show\`
2. 预期结果: <描述预期输出>
3. 实际结果: <实际输出情况>
4. 测试状态: ✅ 通过 / ❌ 失败

### 测试结论
<测试是否成功，如有问题记录>
\`\`\`

### 已完成的重构记录

| 日期 | 文件 | 类型 | 描述 | 状态 |
|------|------|------|------|
| 2026-02-26 | jpg2mp4.py | 删除 | 待开始 |
