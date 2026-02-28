import os
import sys
class python_interpret:
    def __init__(self) -> None:
        # 使用 sys.executable 确保子进程使用与当前进程相同的 Python 解释器（uv 虚拟环境）
        self.pyn = sys.executable
