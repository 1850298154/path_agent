"""
    #### 使用命令 python try.py -h 获得帮助信息。

    * 需要将本库放到需要的程序的第一行。 
    
    * 否则其他输出会抢先在该argument.py初始化之前初始化。
    
    https://blog.csdn.net/feichangyanse/article/details/128559542
    https://blog.csdn.net/raelum/article/details/126245099
    """

print('sys_argument.py 模块初始化...')

import sys
import os
print('sys.argv = ', sys.argv)

# if sys.argv[0] != 'app_main.py':
if os.path.basename(sys.argv[0]) == 'zqt_model_process.py': # 专门为该进程 设置参数
    # 导入库
    import argparse

    # 1. 定义命令行解析器对象
    parser = argparse.ArgumentParser(
            prog="模型进程.py",
            description='模型求解，指定日期时间文件夹作为获取参数输入和结果输出',
            epilog="参数描述结束。",
        )

    # 2. 添加命令行参数
    parser.add_argument(
            '--datetime',  # 第一个"--"开头的作为parser.parse_args()的成员， 用来访问
            '-d',  # 第一个作为parser.parse_args()的成员， 用来访问
            type=str, # 默认str 
            required=True, # 默认false
            default='',
            help="日期时间：作为文件夹，eg：2023-09-16_13-16-47",
        )
    parser.add_argument('--batch', type=int, default=4)

    # 3. 从命令行中结构化解析参数
    _args = parser.parse_args() # Namespace(datetime='12dd', batch=4)
    print(_args)
        
    datetime = parser.parse_args().datetime
    print('datetime = ', datetime)


def get_datetime():
    """
        如果是 'app_main.py' 获取， 刷新时间
        如果是 'zqt_model_process.py' 获取， 从参数中获取时间
    """
    # if sys.argv[0]=='app_main.py':
    if os.path.basename(sys.argv[0]) == 'app_main.py':
        def _get_current_datetime_formatted_file_name():
            import datetime
            # 获取当前日期和时间
            current_datetime = datetime.datetime.now()
            # 格式化日期和时间为字符串，作为文件名
            file_name = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
            return file_name
        datetime = _get_current_datetime_formatted_file_name()
        print('app_main.py          :: ag.get_datetime() : ', datetime)
        return datetime
    elif os.path.basename(sys.argv[0]) == 'zqt_model_process.py':
        print('zqt_model_process.py :: ag.get_datetime() : ', datetime)
        return datetime
    elif os.path.basename(sys.argv[0]) == 'test.py':
        if len(sys.argv) > 1:
            return sys.argv[1]
        # 如果没有参数，返回当前时间
        import datetime
        current_datetime = datetime.datetime.now()
        datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
        print('test.py          :: ag.get_datetime() : ', datetime)
        return datetime

