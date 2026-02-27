"""
简单函数调用追踪脚本
追踪 import 阶段的函数调用
"""
import sys
import cProfile
import io
from collections import defaultdict
import time

# 自定义跟踪装饰器
call_stack = []
call_count = defaultdict(int)
call_depth = 0

def trace_calls(frame, event, arg):
    global call_depth

    if event == 'call':
        func_name = frame.f_code.co_name
        filename = frame.f_code.co_filename

        # 只追踪自定义模块
        if 'obstacle_2d_minimal' in filename or 'path_agent' in filename:
            module_name = filename.split('\\')[-1] if '\\' in filename else filename.split('/')[-1]

            call_key = f"{module_name}:{func_name}"
            call_count[call_key] += 1

            # 记录调用栈（最大深度限制）
            if call_depth < 10:
                call_stack.append(f"{'  ' * call_depth}{module_name}:{func_name}()")
                call_depth += 1

    elif event == 'return':
        call_depth = max(0, call_depth - 1)

    return trace_calls

# 先在模块级别导入
import output_filename as of
import zstatistics as zs
from trajectory import land
import SET
import run as run_module

def analyze_imports():
    """
    分析导入阶段调用的函数
    """
    print("="*80)
    print("函数调用追踪分析")
    print("="*80)

    # 启用系统追踪
    sys.settrace(trace_calls)

    start_time = time.time()

    try:
        print("\n[1] 导入 plot (带 *)...")
        # 模拟 plot.py 的导入
        import plot
        print(f"    ✓ 耗时: {time.time() - start_time:.3f}s")

        print("\n[2] 导入 uav (带 *)...")
        import uav
        print(f"    ✓ 耗时: {time.time() - start_time:.3f}s")

        print("\n[3] 导入 others (带 *)...")
        import others
        print(f"    ✓ 耗时: {time.time() - start_time:.3f}s")

        print("\n[4] 检查 run 模块...")
        print(f"    ✓ 耗时: {time.time() - start_time:.3f}s")

    except Exception as e:
        print(f"\n✗ 导入错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        sys.settrace(None)

    total_time = time.time() - start_time
    print(f"\n{'='*80}")
    print(f"导入总耗时: {total_time:.3f}s")
    print(f"{'='*80}\n")

    # 输出调用栈
    print("\n" + "="*80)
    print("调用栈 (前100个调用)")
    print("="*80)
    for i, call in enumerate(call_stack[:100], 1):
        print(f"{i:3d}. {call}")
    if len(call_stack) > 100:
        print(f"... 还有 {len(call_stack) - 100} 个调用")

    # 输出函数调用统计
    print("\n" + "="*80)
    print("函数调用统计 (按调用次数排序)")
    print("="*80)

    sorted_calls = sorted(call_count.items(), key=lambda x: x[1], reverse=True)
    for i, (func, count) in enumerate(sorted_calls[:50], 1):
        module, name = func.split(':') if ':' in func else ('unknown', func)
        print(f"{i:3d}. {count:4d}次  {module:20s} :: {name}")

    print(f"\n总计: {sum(call_count.values())} 次自定义函数调用")
    print("="*80)

if __name__ == '__main__':
    analyze_imports()
