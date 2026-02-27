"""
追踪导入阶段的函数调用
"""
import sys
import time
from collections import defaultdict

# 追踪数据
call_trace = []
module_calls = defaultdict(int)
call_depth = 0
start_time = None

def trace_calls(frame, event, arg):
    global call_depth

    if event == 'call':
        filename = frame.f_code.co_filename
        funcname = frame.f_code.co_name

        # 只追踪 obstacle_2d_minimal 目录中的文件
        if 'obstacle_2d_minimal' in filename or 'path_agent' in filename:
            module = filename.split('\\')[-1] if '\\' in filename else filename.split('/')[-1]

            # 记录函数调用
            key = f"{module}:{funcname}"
            module_calls[key] += 1

            # 记录调用栈（限制深度）
            if call_depth < 20:
                indent = '  ' * call_depth
                call_trace.append(f"{indent}{module}:{funcname}()")
                call_depth += 1

    elif event == 'return':
        call_depth = max(0, call_depth - 1)

    return trace_calls

def main():
    global start_time

    print("="*80)
    print("函数调用追踪 - 导入阶段")
    print("="*80)
    print()

    start_time = time.time()
    sys.settrace(trace_calls)

    try:
        print("[1/9] 导入 output_filename...")
        import output_filename
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[2/9] 导入 zstatistics...")
        import zstatistics
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[3/9] 导入 trajectory...")
        from trajectory import land
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[4/9] 导入 SET...")
        import SET
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[5/9] 导入 run...")
        import run
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[6/9] 导入 uav...")
        import uav
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[7/9] 导入 others...")
        import others
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[8/9] 导入 geometry...")
        import geometry
        print(f"    OK ({time.time() - start_time:.3f}s)")

        print("[9/9] 导入 plot...")
        import plot
        print(f"    OK ({time.time() - start_time:.3f}s)")

    except Exception as e:
        print(f"\n导入错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        sys.settrace(None)

    total_time = time.time() - start_time
    print()
    print("="*80)
    print(f"导入总耗时: {total_time:.3f}s")
    print(f"追踪的函数调用数: {sum(module_calls.values())}")
    print("="*80)

    # 输出函数调用统计
    print()
    print("="*80)
    print("函数调用统计 (按调用次数排序, Top 50)")
    print("="*80)

    sorted_calls = sorted(module_calls.items(), key=lambda x: x[1], reverse=True)
    for i, (func, count) in enumerate(sorted_calls[:50], 1):
        module, name = func.split(':')
        print(f"{i:2d}. {count:4d}次  {module:20s} :: {name}")

    # 输出调用栈（前100条）
    print()
    print("="*80)
    print("调用栈 (前100条)")
    print("="*80)
    for i, call in enumerate(call_trace[:100], 1):
        print(f"{i:3d}. {call}")

if __name__ == '__main__':
    main()
