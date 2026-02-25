#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试脚本 - 2D 障碍物环境多智能体路径规划

使用说明:
    python run_test.py

测试场景:
    - 10 个智能体
    - 9 个圆形障碍物
    - 每个智能体从起点导航到终点
"""

import sys
import os
sys.stdout.reconfigure(encoding='utf-8')

print("=" * 60)
print("2D 障碍物环境多智能体路径规划测试")
print("=" * 60)
print()

# 设置测试参数
test_time = "2026-02-26_06-00-00"
param_yaml = os.path.join(os.path.dirname(__file__), "004/2026-02-26_06-00-00/parameters.yaml")
show_mode = "not_show"

print(f"测试时间: {test_time}")
print(f"参数文件: {param_yaml}")
print(f"显示模式: {show_mode}")
print()

# 运行测试
print("开始运行测试...")
print("-" * 60)

sys.argv = ['test.py', test_time, param_yaml, show_mode]

try:
    exec(open('test.py', encoding='utf-8').read())
    print()
    print("-" * 60)
    print("测试完成！")
    print("=" * 60)
except Exception as e:
    print(f"测试出错: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
