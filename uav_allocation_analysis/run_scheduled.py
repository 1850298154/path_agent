#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
调度测试运行脚本 - 支持从调度管理器触发路径规划
"""
import sys
import os
import json
import argparse
import time


def load_scheduled_config(config_path: str) -> dict:
    """加载调度配置"""
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        print(f"错误：无法加载配置文件 {config_path}: {e}")
        sys.exit(1)


def run_path_planning(config: dict) -> str:
    """运行路径规划测试"""
    test_config_path = config['test_config']
    test_args = [test_config_path]

    # 切换到脚本目录
    script_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'obstacle_2d_minimal')

    # 构建测试参数文件路径
    # 注意：调度管理器会生成 scheduled_results/<timestamp>/parameters.yaml

    # 运行 test.py
    import subprocess
    print(f"运行路径规划测试...")
    print(f"测试配置: {test_args}")

    # 直接调用 test.py，它已经是入口脚本
    result = subprocess.run(
        [sys.executable, 'test.py'] + test_args,
        cwd=script_dir,
        capture_output=True,
        text=True,
        timeout=600
    )

    print(f"返回码: {result.returncode}")
    if result.stdout:
        print(f"输出:\n{result.stdout}")
    else:
        print(f"标准错误:\n{result.stderr}")

    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(
        description='调度测试运行工具'
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument('config', '-c', type=str, default='test_parameters.yaml',
                       help='测试配置文件路径（调度管理器生成）')

    args = parser.parse_args()

    print("=" * 80)
    print("调度测试运行工具")
    print("=" * 80)

    # 加载配置
    config = load_scheduled_config(args.config)

    # 运行路径规划
    success = run_path_planning(config)

    print("=" * 80)
    if success:
        print("✓ 路径规划测试完成")
    else:
        print("✗ 路径规划测试失败")
        print("=" * 80)
        sys.exit(success and 0 or 1)


if __name__ == '__main__':
    main()
