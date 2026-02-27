import pstats

p = pstats.Stats("profile_output.prof")
p.sort_stats("cumulative")

stats = p.stats
custom_funcs = []

for func_key, (cc, nc, tt, ct, callers) in stats.items():
    filename, line, funcname = func_key
    if "obstacle_2d_minimal" in filename:
        module = filename.split("/")[-1] if "/" in filename else filename.split("\\")[-1]
        custom_funcs.append({
            "module": module,
            "func": funcname,
            "line": line,
            "ncalls": nc,
            "tottime": tt,
            "cumtime": ct
        })

custom_funcs.sort(key=lambda x: x["cumtime"], reverse=True)

print("="*80)
print("fisrt_run 分支 - 导入阶段函数调用")
print("="*80)
print(f"找到 {len(custom_funcs)} 个自定义模块函数调用\n")
print("-"*80)

for i, func in enumerate(custom_funcs[:50], 1):
    print(f"{i:2d}. {func['module']:25s}:{func['func']:20s}  调用:{func['ncalls']:4d}  时间:{func['cumtime']:8.4f}s")
