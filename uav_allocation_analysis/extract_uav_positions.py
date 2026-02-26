"""
提取每个时间点UAV的位置信息

输出格式:
- UAV起点位置
- 每个时间点的UAV位置（按时间步长采样）
"""
import json
import numpy as np

# 时间采样步长（秒）
TIME_STEP = 1.0

def load_data():
    """加载数据文件"""
    with open('precomputed_data.json', 'r') as f:
        precomputed = json.load(f)
    with open('result_criticalpath_new.json', 'r') as f:
        result = json.load(f)
    return precomputed, result

def interpolate_position(start_pos, end_pos, start_time, end_time, query_time):
    """线性插值计算位置"""
    if end_time <= start_time:
        return start_pos
    t = (query_time - start_time) / (end_time - start_time)
    t = max(0, min(1, t))  # 限制在[0,1]
    return start_pos + t * (end_pos - start_pos)

def get_task_center(task_positions, task_id, target):
    """获取任务中心位置（简化为任务位置）"""
    pos = task_positions[str(task_id)]
    return np.array(pos)

def extract_uav_positions(precomputed, result, time_step=TIME_STEP):
    """提取每个时间点的UAV位置"""
    makespan = result['makespan']
    num_steps = int(np.ceil(makespan / time_step)) + 1
    times = np.arange(0, makespan + time_step, time_step)

    uav_init_pos = precomputed['uav_init_pos']
    task_positions = precomputed['task_positions']
    uav_schedule = result['uav_schedule']

    uav_positions = {}

    for uav_id_str, schedule in uav_schedule.items():
        uav_id = int(uav_id_str)
        init_pos = np.array(uav_init_pos[uav_id_str])

        # 收集所有时间点
        time_points = []

        # 添加初始状态
        time_points.append({
            'time': 0,
            'pos': init_pos.copy(),
            'state': 'init',
            'task_id': None,
            'target': None
        })

        # 处理每个调度项
        last_task_pos = init_pos

        for i, item in enumerate(schedule):
            task_id = item['task']
            target = item['target']
            travel_start = item['travel_start']
            travel_end = item['travel_end']
            start = item['start']
            end = item['end']

            # 获取任务位置
            task_pos = np.array(task_positions[str(task_id)])

            # 旅行阶段
            # 在旅行开始时
            time_points.append({
                'time': travel_start,
                'pos': last_task_pos.copy(),
                'state': 'travel_start',
                'task_id': task_id,
                'target': target
            })

            # 在旅行结束时（到达任务位置）
            time_points.append({
                'time': travel_end,
                'pos': task_pos.copy(),
                'state': 'arrive',
                'task_id': task_id,
                'target': target
            })

            # 任务执行阶段
            time_points.append({
                'time': start,
                'pos': task_pos.copy(),
                'state': 'task_start',
                'task_id': task_id,
                'target': target
            })

            time_points.append({
                'time': end,
                'pos': task_pos.copy(),
                'state': 'task_end',
                'task_id': task_id,
                'target': target
            })

            last_task_pos = task_pos

        # 按时间排序
        time_points = sorted(time_points, key=lambda x: x['time'])

        # 插值生成每个时间步的位置
        positions = []
        for t in times:
            pos = get_position_at_time(time_points, t)
            positions.append(pos)

        uav_positions[uav_id] = {
            'init_pos': init_pos.tolist(),
            'positions': [pos.tolist() for pos in positions],
            'times': times.tolist()
        }

    return uav_positions

def get_position_at_time(time_points, query_time):
    """获取指定时间的位置"""
    if query_time <= time_points[0]['time']:
        return time_points[0]['pos']

    for i in range(len(time_points) - 1):
        if time_points[i]['time'] <= query_time <= time_points[i + 1]['time']:
            return interpolate_position(
                time_points[i]['pos'],
                time_points[i + 1]['pos'],
                time_points[i]['time'],
                time_points[i + 1]['time'],
                query_time
            )

    return time_points[-1]['pos']

def save_positions(uav_positions, output_file='uav_positions_over_time.json'):
    """保存位置数据"""
    output = {
        'time_step': TIME_STEP,
        'makespan': max(data['times'][-1] for data in uav_positions.values()),
        'uavs': uav_positions
    }
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=2)
    print(f"位置数据已保存到: {output_file}")

def print_summary(uav_positions):
    """打印摘要信息"""
    print("\n" + "="*60)
    print("UAV位置数据摘要")
    print("="*60)

    for uav_id, data in sorted(uav_positions.items()):
        init_pos = data['init_pos']
        num_steps = len(data['positions'])
        print(f"\nUAV {uav_id}:")
        print(f"  起点位置: [{init_pos[0]:.2f}, {init_pos[1]:.2f}]")
        print(f"  时间步数: {num_steps}")
        print(f"  时间范围: {data['times'][0]:.1f}s - {data['times'][-1]:.1f}s")

        # 打印部分位置采样
        sample_indices = [0, len(data['positions'])//4, len(data['positions'])//2,
                         3*len(data['positions'])//4, len(data['positions'])-1]
        print(f"  位置采样:")
        for idx in sample_indices:
            pos = data['positions'][idx]
            t = data['times'][idx]
            print(f"    t={t:6.1f}s: [{pos[0]:7.2f}, {pos[1]:7.2f}]")

if __name__ == '__main__':
    print("正在加载数据...")
    precomputed, result = load_data()

    print("正在提取UAV位置...")
    uav_positions = extract_uav_positions(precomputed, result)

    print("正在保存数据...")
    save_positions(uav_positions)

    print_summary(uav_positions)

    print("\n完成！")
