"""
函数调用路径分析脚本
使用 cProfile 和自定义分析器来追踪函数调用路径
"""
import sys
import pstats
import io
from collections import defaultdict

# 颜色输出
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def analyze_profile(profile_file, top_n=50):
    """
    分析 profile 文件并输出函数调用路径
    """
    print(f"\n{Colors.HEADER}{'='*80}{Colors.ENDC}")
    print(f"{Colors.HEADER}函数调用路径分析{Colors.ENDC}")
    print(f"{Colors.HEADER}Profile 文件: {profile_file}{Colors.ENDC}")
    print(f"{Colors.HEADER}{'='*80}{Colors.ENDC}\n")

    p = pstats.Stats(profile_file)

    # 1. 按累积时间排序
    print(f"{Colors.OKCYAN}{Colors.BOLD}【1】按累积时间排序 (Top {top_n}){Colors.ENDC}")
    print(f"{Colors.HEADER}{'-'*80}{Colors.ENDC}")
    p.sort_stats('cumulative')
    p.print_stats(top_n)
    print()

    # 2. 按自身时间排序
    print(f"{Colors.OKCYAN}{Colors.BOLD}【2】按自身时间排序 (Top {top_n}){Colors.ENDC}")
    print(f"{Colors.HEADER}{'-'*80}{Colors.ENDC}")
    p.sort_stats('time')
    p.print_stats(top_n)
    print()

    # 3. 按调用次数排序
    print(f"{Colors.OKCYAN}{Colors.BOLD}【3】按调用次数排序 (Top {top_n}){Colors.ENDC}")
    print(f"{Colors.HEADER}{'-'*80}{Colors.ENDC}")
    p.sort_stats('ncalls')
    p.print_stats(top_n)
    print()

    # 4. 统计自定义模块的函数调用
    print(f"{Colors.OKCYAN}{Colors.BOLD}【4】自定义模块函数调用统计{Colors.ENDC}")
    print(f"{Colors.HEADER}{'-'*80}{Colors.ENDC}")

    stats = p.stats

    # 筛选自定义模块 (排除标准库和第三方库)
    custom_modules = defaultdict(list)
    for func_key, (cc, nc, tt, ct, callers) in stats.items():
        filename, line, funcname = func_key
        # 筛选条件: 文件路径包含 obstacle_2d_minimal 或相关目录
        if 'obstacle_2d_minimal' in filename or 'uav' in filename.lower():
            module_name = filename.split('\\')[-1] if '\\' in filename else filename.split('/')[-1]
            custom_modules[module_name].append({
                'func': f"{module_name}:{line}({funcname})",
                'ncalls': nc,
                'tottime': tt,
                'cumtime': ct,
                'percall': ct / nc if nc > 0 else 0
            })

    # 按模块分组显示
    for module, functions in sorted(custom_modules.items()):
        print(f"\n{Colors.OKGREEN}模块: {module}{Colors.ENDC}")
        print(f"{Colors.WARNING}{'-'*80}{Colors.ENDC}")
        # 按累积时间排序
        functions.sort(key=lambda x: x['cumtime'], reverse=True)
        for i, func in enumerate(functions[:10], 1):
            print(f"  {i:2d}. {Colors.BOLD}{func['func']}{Colors.ENDC}")
            print(f"      调用: {func['ncalls']:5d} 次 | "
                  f"自身时间: {func['tottime']:8.4f}s | "
                  f"累积时间: {func['cumtime']:8.4f}s | "
                  f"平均: {func['percall']:8.6f}s")

    # 5. 总体统计
    print(f"\n{Colors.OKCYAN}{Colors.BOLD}【5】总体统计{Colors.ENDC}")
    print(f"{Colors.HEADER}{'-'*80}{Colors.ENDC}")

    # 获取原始统计数据
    stats_stream = io.StringIO()
    p.stream = stats_stream
    p.print_stats()

    stats_stream.seek(0)
    stats_text = stats_stream.read()

    # 解析总函数调用数
    import re
    match = re.search(r'(\d+) function calls', stats_text)
    if match:
        total_calls = match.group(1)
        print(f"{Colors.OKGREEN}总函数调用数: {total_calls}{Colors.ENDC}")

    print(f"\n{Colors.HEADER}{'='*80}{Colors.ENDC}")
    print(f"{Colors.HEADER}分析完成{Colors.ENDC}")
    print(f"{Colors.HEADER}{'='*80}{Colors.ENDC}\n")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        profile_file = sys.argv[1]
    else:
        profile_file = 'profile_output.prof'

    top_n = int(sys.argv[2]) if len(sys.argv) > 2 else 50

    analyze_profile(profile_file, top_n)
