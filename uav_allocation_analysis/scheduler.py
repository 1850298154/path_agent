"""
调度管理器 - 支持任务队列和顺序执行

增强功能：
1. 任务队列管理（支持添加/暂停/恢复/删除任务）
2. 每个任务状态追踪
3. 任务顺序执行（一个完成后才开始下一个）
4. RPC 客户管理
"""
import json
import time
import socket
import threading
import hashlib
import os
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Set, Tuple
from dataclasses import dataclass, field
from enum import Enum
from queue import Queue, PriorityQueue


# ==================== 配置和状态 ====================

class TaskStatus(str, Enum):
    """任务状态"""
    PENDING = "pending"           # 等待执行
    RUNNING = "running"           # 执行中
    COMPLETED = "completed"     # 已完成
    FAILED = "failed"            # 失败
    PAUSED = "paused"           # 已暂停


class TaskPriority(str, Enum):
    """任务优先级"""
    HIGH = "high"
    NORMAL = "normal"
    LOW = "low"


class UAVStatus(str, Enum):
    """UAV 状态"""
    IDLE = "idle"
    TO_START = "to_start"
    IN_FLIGHT = "in_flight"
    REACHED = "reached"
    RETURNING = "returning"


@dataclass
class UAVInfo:
    """UAV 信息"""
    id: int
    status: UAVStatus = UAVStatus.IDLE
    last_known_position: Optional[List[float]] = None
    target_position: Optional[List[float]] = None
    last_update: Optional[float] = None


@dataclass
class Task:
    """任务信息"""
    task_id: str
    name: str
    priority: TaskPriority
    config: dict              # 测试配置
    status: TaskStatus = TaskStatus.PENDING
    created_at: Optional[float] = None
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    error_message: Optional[str] = None
    result: Optional[dict] = None


@dataclass
class RPCClient:
    """RPC 客户连接"""
    socket: socket.socket
    connected: bool = False
    last_heartbeat: float = 0.0

    def __init__(self, address: str, port: int):
        self.address = address
        self.port = port

    def connect(self) -> bool:
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.address, self.port))
            self.connected = True
            print(f"[RPC] 已连接到 {self.address}:{self.port}")
            return True

        except Exception as e:
            print(f"[RPC] 连接失败: {e}")
            return False

    def disconnect(self):
        if self.socket:
            self.socket.close()
            self.connected = False
            print(f"[RPC] 已断开连接")

    def send_heartbeat(self) -> bool:
        """发送心跳包"""
        if not self.connected:
            return False

        try:
            heartbeat = b"heartbeat"
            self.socket.sendall(heartbeat)
            self.last_heartbeat = time.time()
            return True
        except Exception as e:
            print(f"[RPC] 发送心跳失败: {e}")
            return False

    def update_status(self, task_id: str, task: Task) -> bool:
        """更新任务状态到 RPC 服务器"""
        status_json = json.dumps({
            "task_id": task_id,
            "status": task.status.value,
            "timestamp": time.time()
        }).encode('utf-8')

        try:
            self.socket.sendall(status_json)
            print(f"[RPC] 已更新任务 {task_id} 状态为 {task.status.value}")
            return True
        except Exception as e:
            print(f"[RPC] 更新任务状态失败: {e}")
            return False


# ==================== 配置 ====================

class SchedulerConfig:
    """调度器配置"""
    def __init__(self,
                 uav_count: int = 80,
                 check_interval: float = 1.0,   # UAV 状态检查间隔
                 ready_timeout: int = 30,        # UAV 准备就绪超时
                 execution_timeout: int = 600,  # 任务执行超时
                 rpc_port: int = 12345,     # RPC 服务器端口
                 output_dir: str = "scheduled_results",
                 task_queue_file: str = "task_queue.json",
                 log_file: str = "scheduler.log"):
        self.uav_count = uav_count
        self.check_interval = check_interval
        self.ready_timeout = ready_timeout
        self.execution_timeout = execution_timeout
        self.rpc_port = rpc_port
        self.output_dir = output_dir
        self.task_queue_file = task_queue_file
        self.log_file = log_file


# ==================== 调度管理器 ====================

class UAVTaskManager:
    """UAV 任务和调度管理器"""

    def __init__(self, config: SchedulerConfig):
        self.config = config
        self.uavs: Dict[int, UAVInfo] = {i: UAVInfo(id=i) for i in range(config.uav_count)}
        self.uav_ready_count: Set[int] = set()
        self.current_task: Optional[Task] = None
        self.task_queue: PriorityQueue[Tuple[str, float, Task]] = PriorityQueue()  # 优先级队列
        self.rpc_client: Optional[RPCClient] = None
        self.running = False
        self.stop_requested = False
        self.last_queue_save = 0.0

        # 创建输出目录
        os.makedirs(config.output_dir, exist_ok=True)
        os.makedirs(os.path.join(config.output_dir, 'task_configs'), exist_ok=True)
        os.makedirs(os.path.join(config.output_dir, 'task_results'), exist_ok=True)

        # 加载任务队列
        self._load_task_queue()

        # 创建日志文件
        self._init_log_file()

    def _init_log_file(self):
        """初始化日志文件"""
        if not os.path.exists(self.config.log_file):
            with open(self.config.log_file, 'w', encoding='utf-8') as f:
                f.write(f"# 调度管理器日志 - {datetime.now()}\n\n")
        else:
            # 追加到现有日志
            with open(self.config.log_file, 'a', encoding='utf-8') as f:
                f.write("\n")

    def _log(self, level: str, message: str):
        """记录日志"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] [{level}] {message}\n"
        with open(self.config.log_file, 'a', encoding='utf-8') as f:
            f.write(log_entry)

    def _info(self, message: str):
        self._log("INFO", message)

    def _warning(self, message: str):
        self._log("WARNING", message)

    def _error(self, message: str):
        self._log("ERROR", message)

    def _save_task_queue(self):
        """保存任务队列"""
        queue_data = []
        # PriorityQueue 使用堆存储，每个元素是 (priority, created_at, task)
        temp_queue = []
        while not self.task_queue.empty():
            priority, created_at, task = self.task_queue.get()
            task_dict = {
                "task_id": task.task_id,
                "name": task.name,
                "priority": task.priority.value,
                "config": task.config,
                "status": task.status.value,
                "created_at": task.created_at,
            }
            queue_data.append(task_dict)
            temp_queue.append((priority, created_at, task))

        # 重建队列
        for item in temp_queue:
            self.task_queue.put(item)

        with open(self.config.task_queue_file, 'w', encoding='utf-8') as f:
            json.dump(queue_data, f, indent=2)

        self.last_queue_save = time.time()

    def _load_task_queue(self):
        """加载任务队列"""
        if not os.path.exists(self.config.task_queue_file):
            self._info("任务队列为空，创建新队列")
            return

        try:
            with open(self.config.task_queue_file, 'r', encoding='utf-8') as f:
                queue_data = json.load(f)

            for task_dict in queue_data:
                task = Task(**task_dict)
                task.created_at = datetime.fromisoformat(task_dict['created_at'])
                self.task_queue.put((task.priority.value, task.created_at, task))

            self._info(f"已加载 {len(queue_data)} 个任务到队列")

        except Exception as e:
            self._error(f"加载任务队列失败: {e}")

    def _check_uav_ready(self) -> Tuple[bool, str]:
        """检查 UAV 是否就绪"""
        ready_count = 0
        all_ready = True
        not_ready_list = []

        for uav_id, uav in self.uavs.items():
            # 检查就绪条件
            is_idle = (uav.status == UAVStatus.IDLE)
            has_target = (uav.target_position is not None)

            is_ready = is_idle and has_target

            if is_ready:
                ready_count += 1
                self.uav_ready_count.add(uav_id)
            else:
                all_ready = False
                not_ready_list.append(f"UAV {uav_id}: {uav.status.value}")

        all_ready_str = ", ".join(not_ready_list)

        return ready_count >= self.config.uav_count, all_ready_str

    def _create_uav_target_from_config(self, task_config: dict) -> Optional[List[float]]:
        """从配置创建 UAV 目标位置"""
        # 如果配置中有 agent_target，直接使用
        agent_target = task_config.get('agent_target')
        if agent_target and len(agent_target) > 0:
            return agent_target[0] if len(agent_target) == 2 else None
        return None

    def _update_uav_targets_from_config(self, task_config: dict):
        """从配置更新 UAV 目标位置"""
        agent_target = task_config.get('agent_target')
        if agent_target and len(agent_target) > 0:
            for idx, target in enumerate(agent_target):
                if idx < len(self.uavs):
                    self.uavs[idx].target_position = target

    def _add_task_to_queue(self, task_name: str, priority: TaskPriority, test_config: dict):
        """添加任务到队列"""
        task_id = hashlib.md5(f"{int(time.time()*1000)}".encode()).hexdigest()[:8]

        task = Task(
            task_id=task_id,
            name=task_name,
            priority=priority,
            config=test_config,
            status=TaskStatus.PENDING,
            created_at=time.time()
        )

        self.task_queue.put((priority.value, task.created_at, task))
        self._save_task_queue()

        return task_id

    def add_tasks_batch(self, task_configs: List[dict]) -> List[str]:
        """批量添加任务到队列"""
        task_ids = []
        for config in task_configs:
            task_id = self._add_task_to_queue(config['name'], TaskPriority.NORMAL, config)
            task_ids.append(task_id)

        self._info(f"已添加 {len(task_ids)} 个任务到队列")

        return task_ids

    def list_tasks(self):
        """列出所有任务"""
        # 显示当前任务
        if self.current_task:
            print(f"[当前] {self.current_task.task_id}: {self.current_task.name} ({self.current_task.status.value})")

        # 显示队列中的任务
        temp_queue = []
        if not self.task_queue.empty():
            print("[队列中]")
            while not self.task_queue.empty():
                priority, created_at, task = self.task_queue.get()
                print(f"  {task.task_id}: {task.name} (优先级: {task.priority.value}, 状态: {task.status.value})")
                temp_queue.append((priority, created_at, task))

            # 重建队列
            for item in temp_queue:
                self.task_queue.put(item)
        else:
            print("[队列中] 无任务")

    def start(self, mode: str = "auto"):
        """启动调度器"""
        if mode == "auto":
            # 自动模式：启动 RPC 服务器
            if self.config.rpc_port:
                self.rpc_client = RPCClient("127.0.0.1", self.config.rpc_port)
                self.rpc_client.connect()
            else:
                self._error("RPC 客端未配置")
                return False

            self.running = True
            self._info("调度器已启动（自动模式）")

            # 检查 UAV 并启动监控
            self._monitor_uav_status()

        else:
            # 手动模式
            self._info("调度器已启动（手动模式）")
            self.running = True

        return True

    def stop(self):
        """停止调度器"""
        self.stop_requested = True

        if self.rpc_client:
            self.rpc_client.disconnect()
            self._info("RPC 客户端已断开")

        self.running = False
        self._info("调度器已停止")

        return True

    def pause_task(self, task_id: str):
        """暂停任务"""
        if self.current_task and self.current_task.task_id == task_id:
            self.current_task.status = TaskStatus.PAUSED
            self._info(f"任务 {task_id} 已暂停")
            self.rpc_client.update_status(task_id, self.current_task)
            return True
        return False

    def resume_task(self, task_id: str):
        """恢复任务"""
        # 在暂停状态的任务中查找并恢复
        # 这里简化处理，直接重新添加到队列
        self._info(f"任务 {task_id} 恢复功能待实现（暂不支持暂停）")
        return False

    def remove_task(self, task_id: str):
        """删除任务"""
        # 从队列中移除指定任务
        temp_queue = []
        removed = False

        while not self.task_queue.empty():
            priority, created_at, task = self.task_queue.get()
            if task.task_id == task_id:
                self._info(f"任务 {task_id} 已从队列移除")
                removed = True
            else:
                temp_queue.append((priority, created_at, task))

        # 重建队列（不包含已删除的任务）
        for item in temp_queue:
            self.task_queue.put(item)

        if removed:
            self._save_task_queue()
        return removed

    def _monitor_uav_status(self):
        """监控 UAV 状态并触发任务"""
        check_count = 0

        while self.running and not self.stop_requested:
            time.sleep(self.config.check_interval)

            # 检查就绪状态
            ready_count, ready_str = self._check_uav_ready()

            if ready_count >= self.config.uav_count:
                self._info(f"所有 UAV 已就绪，检查任务队列...")

                # 如果有待执行任务，开始执行
                if not self.current_task and not self.task_queue.empty():
                    # 获取高优先级任务
                    priority, created_at, task = self.task_queue.get()
                    if task:
                        self.current_task = task
                        self.current_task.status = TaskStatus.RUNNING
                        self.current_task.started_at = time.time()

                        # 更新到 RPC
                        self.rpc_client.update_status(task.task_id, task)

                        # 执行任务
                        self._execute_task(task)

                        # 标记完成
                        self.current_task.status = TaskStatus.COMPLETED
                        self.current_task.completed_at = time.time()
                        self.current_task.result = {"status": "completed"}

                        # 更新到 RPC
                        self.rpc_client.update_status(task.task_id, self.current_task)

                        self._info(f"任务 {task.task_id} 已完成")
                        self.current_task = None  # 重置当前任务
                else:
                    self._warning("没有待执行任务")

                check_count += 1

                if check_count >= self.config.ready_timeout and ready_count == 0:
                    # 超时未就绪
                    self._warning(f"UAV 就绪超时 ({self.config.ready_timeout}秒)，跳过本次检查")
                    # 重置就绪计数
                    check_count = 0

            else:
                # 未就绪，等待
                self._info(f"等待 UAV 就绪... ({ready_str})")

    def _execute_task(self, task: Task):
        """执行单个任务"""
        test_config = task.config

        self._info(f"开始执行任务: {task.task_id} ({task.name})")

        # 生成输出目录
        output_path = os.path.join(
            self.config.output_dir,
            f"task_{task.task_id}_{datetime.now().strftime('%Y%m%d_%H-%M-%S')}"
        )
        os.makedirs(output_path, exist_ok=True)

        # 生成测试配置文件
        config_path = os.path.join(output_path, 'test_config.json')
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(test_config, f, indent=2)

        # 创建参数文件
        param_path = os.path.join(output_path, 'parameters.yaml')
        with open(param_path, 'w', encoding='utf-8') as f:
            f.write(f"agent.Num: {test_config.get('agent.Num')}\n")
            f.write(f"agent.Vmax: {test_config.get('agent.Vmax')}\n")
            f.write(f"agent.physical_radius: {test_config.get('agent.physical_radius')}\n")
            f.write(f"agent.radius: {test_config.get('agent.radius')}\n")
            f.write(f"agent.Umax: {test_config.get('agent.Umax')}\n")
            f.write(f"agent.Vmax: {test_config.get('agent.Vmax')}\n")

            # 从配置生成障碍物（如果配置中没有，使用默认）
            obstacles = test_config.get('obstacles')
            if not obstacles and len(obstacles) >= 0:
                f.write(f"\n# 障碍物配置:\n")
                for obs in obstacles:
                    f.write(f"obstacle_list.append([{obs[0]}, {obs[1]}, {obs[2]}])\n")
                f.write(f"ob.apart_num: {len(obstacles)}\n")
                f.write(f"ob.rate: {test_config.get('ob.rate', 0.05)}\n")
            else:
                f.write(f"\n# 使用 UAV 数据中的障碍物\n")
                # 从 uav_allocation_analysis 加载障碍物位置
                # 这里简化处理，实际应该从调度管理器传递

        # 调用 test.py 运行路径规划
        import subprocess
        test_args = [param_path]
        print(f"执行测试: {test_args}")

        result = subprocess.run(
            [sys.executable, 'test.py'] + test_args,
            cwd=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'obstacle_2d_minimal'),
            capture_output=True,
            text=True,
            timeout=self.config.execution_timeout
        )

        # 保存结果
        task.result = {
            "status": "completed" if result.returncode == 0 else "failed",
            "return_code": result.returncode,
            "output_path": output_path,
            "config_path": config_path,
            "timestamp": time.time()
        }

        return True

    def get_status(self) -> dict:
        """获取状态"""
        uav_status = {}
        for uav_id, uav in self.uavs.items():
            uav_status[uav_id] = uav.status.value

        return {
            "running": self.running,
            "uav_count": len(self.uavs),
            "uav_ready_count": len(self.uav_ready_count),
            "uav_status": uav_status,
            "current_task": self.current_task.task_id if self.current_task else None,
            "task_queue_size": self.task_queue.qsize(),
            "timestamp": time.time()
        }


# ==================== 命令行工具 ====================

def print_separator(char='=', length=80):
    """打印分隔线"""
    print(char * length)


def main():
    print_separator('=')
    print("增强调度管理器")
    print_separator('=')

    # 配置
    config = SchedulerConfig()

    # 创建调度管理器
    scheduler = UAVTaskManager(config)

    print_separator()
    print("可用命令:")
    print("  任务管理:")
    print("    add <config.json>          - 从配置文件批量添加任务")
    print("    add -n <name> -p <priority>  - 添加单个任务")
    print("    list                         - 列出所有任务")
    print("    pause <task_id>          - 暂停任务")
    print("    resume <task_id>           - 恢复任务")
    print("    remove <task_id>          - 删除任务")
    print()
    print("  调度控制:")
    print("    start [auto|manual]    - 启动调度器")
    print("    stop                        - 停止调度器")
    print("    status                      - 显示状态")
    print()

    # 命令行接口
    import sys
    if len(sys.argv) > 1:
        command = sys.argv[1]

        if command == 'add':
            if len(sys.argv) > 2:
                config_file = sys.argv[2]
                scheduler.add_tasks_batch([config_file])
            else:
                print("错误: 请指定配置文件")

        elif command == 'list':
            scheduler.list_tasks()

        elif command == 'pause':
            if len(sys.argv) > 2:
                task_id = sys.argv[2]
                scheduler.pause_task(task_id)
            else:
                print("用法: pause <task_id>")

        elif command == 'resume':
            if len(sys.argv) > 2:
                task_id = sys.argv[2]
                scheduler.resume_task(task_id)
            else:
                print("用法: resume <task_id>")

        elif command == 'remove':
            if len(sys.argv) > 2:
                task_id = sys.argv[2]
                scheduler.remove_task(task_id)
            else:
                print("用法: remove <task_id>")

        elif command == 'start':
            mode = sys.argv[2] if len(sys.argv) > 2 else "auto"
            if scheduler.start(mode):
                print(f"调度器已启动 ({mode} 模式)")
            else:
                print(f"启动失败: {mode}")

        elif command == 'stop':
            scheduler.stop()
            print("调度器已停止")

        elif command == 'status':
            status = scheduler.get_status()
            print(json.dumps(status, indent=2, ensure_ascii=False))

        else:
            print(f"未知命令: {command}")
            print("使用: add | list | pause | resume | remove | start | stop | status")

    else:
        scheduler._info(f"启动调度器（默认自动模式）")
        if scheduler.start():
            print("调度器已启动")

    print_separator()
