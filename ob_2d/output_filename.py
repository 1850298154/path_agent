# OB/output_filename.py
"""
            《output_filename.py 文件为三个进程服务》
zyaml.py 
    一个操作： _______________________________
        1. 以当前日期，产生："003/日期时间/"的测试路径
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣

zpytest.py 
    在导入output_filename.py解析包时。
        因为自动获取从进程参数中无法得到参数，
        所以以当前时间作为参数创建一个测试目录。
        从始至终程序不会输出，为空目录。
        后期分析截图可以放总结。
    之后每一轮测试都会产生新的日期，作为参数1传给app.py进程(第二个参数)。
    两个操作，同类函数： _______________________________
        程序开始：
            1. 创建空的zpytest.py的测试目录。
        程序任务：
            2. 每个测试，app.py进程，创建空的测试目录，并作为参数传入。
                同时将进程输出的存入文件夹中。
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣

app.py
    在导入output_filename.py解析包时。自动获取从进程参数中，
        获取zpytest.py传入的要测试输出的时间日期路径。
    三个操作： _______________________________
        程序开始：
            1. 从进程参数获取时间、或自己以当前，创建路径："002/日期时间"文件夹。
        程序结束：
            2. 将agent_list结果输出到到自己测试路径下。
        终端持续输出：
            3. 实时的存入print.txt文件。（由zpytest.py完成）
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣
"""
import numpy as np
import json
import time
import datetime
import os
import sys
# 用于区分不同平台， 使用不同的测试文件夹和测试命令
# 使用 sys.executable 确保子进程使用与当前进程相同的 Python 解释器（uv 虚拟环境）
pyn = sys.executable

if os.name == 'nt':  # Windows系统
    print('                    Windows                  ')
    # test_plan = '002_nt'
    test_plan = '004'
    # TestDataSet = '003_nt_TestDataSet/'
    # TestDataSet = '003_nt/'
    TestDataSet = '004/'
else:  # Linux系统
    print('                    Linux                    ')
    # test_plan = '002'
    test_plan = '004'
    # TestDataSet = '003_TestDataSet/'
    # TestDataSet = '003/'
    TestDataSet = '004/'


############################################################
###############     002/ 测试输出文件夹,时间开头     #########
############################################################
def create_file(file_path):
    """创建文件或文件夹，结尾是'/'则创建文件夹
    """
    # 获取目录路径
    dir_path = os.path.dirname(file_path)
    # 如果目录不存在，则创建目录
    if not os.path.exists(dir_path):
        # print('create dir_path ', dir_path)
        os.makedirs(dir_path)

    # 如果文件不存在，则创建文件
    if not os.path.exists(file_path):
        # print('create file_path ', file_path)
        open(file_path, 'w').close()


def get_current_datetime_formatted_file_name():
    # 获取当前日期和时间
    current_datetime = datetime.datetime.now()

    # 格式化日期和时间为字符串，作为文件名
    file_name = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")

    return file_name


def datetime_from_sh():
    import sys

    # 获取脚本文件路径
    script_path = sys.argv[0]
    print("Script path : ", script_path)

    # 获取传递给脚本的参数
    args = sys.argv[1:]
    print("Script parameters : ", args)

    if len(sys.argv) == 1:
        # 只有命令 没有参数
        # 单独启动任意一个程序，没参数，都会创建文件
        return get_current_datetime_formatted_file_name()
    elif len(sys.argv) >= 2:
        # 有命令 且一定有第一个参数
        if (sys.argv[0] == 'zpytest.py'
                and sys.argv[1] == 'datetime'
                ):
            return get_current_datetime_formatted_file_name()
        return sys.argv[1]


datetime_from_sh_ret = datetime_from_sh()
path_dir = (
    test_plan
    + '/'
    # + get_current_datetime_formatted_file_name()
    # + datetime_from_sh()
    + datetime_from_sh_ret
    + '/')
create_file(path_dir)
print('The output path of this test result is : ', path_dir)
# input('這是一個測試這')


############################################################
###############     003 测试数据文件夹     ##################
############################################################


def ret_produce_test_data_set_path():
    dir = TestDataSet + datetime_from_sh_ret + '/'
    create_file(dir)
    return dir


# def ret_consume_test_data_set_path():
#     dir = TestDataSet + datetime_from_sh_ret
#     create_file(dir)
#     return dir


# def refrash():
#     # 不使用 global ， 无法访问外部变量， 创建新变量不会修改全局变量
#     global path_dir
#     path_dir = (
#         test_plan
#         + '/'
#         # + get_current_datetime_formatted_file_name()
#         + datetime_from_sh()
#         + '/')
#     create_file(path_dir)

#     print('The output path of this test result is :   ', path_dir)
#     # input('這是一個測試這')

# # # test
# # [(time.sleep(1), refrash(), print(path_dir)) for _ in range(3)]

############################################################
###############     002 测试结束保存结果       ##############
############################################################
def saveJSON(my_object, onlyfilename):
    # 将数据写入文件
    filename = path_dir + 'agent100/' + onlyfilename+'.json'
    filenameCanLoad = path_dir + 'agent100/' + 'Load----' + onlyfilename+'.json'
    # FileNotFoundError: [Errno 2] No such file or directory: '002/2023-08-12_09-47-07/agent100/Load----002/2023-08-12_09-47-07/agent100/get_obstacle_list----agent_list[0].json.json'
    create_file(filename)
    # create_file(filenameCanLoad)

    if (hasattr(my_object, '__dict__')):

        class NumpyEncoder(json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()  # 将数组转换为列表
                return super().default(obj)

        # # 使用自定义的JSON编码器
        # json_data = json.dumps(variables, ensure_ascii=False, cls=NumpyEncoder)

        # 获取类的变量和值
        variables = my_object.__dict__
        # 打印结果
        # json_data = json.dumps(variables, ensure_ascii=False)  # 这一步出错
        # json_data = json.dumps(
        #     variables, ensure_ascii=False, cls=NumpyEncoder)   # 这一步出错，但是加上解码后修正好了
        # print(json_data)
        # # 将多边形对象转换为字典
        # for key, value in variables.items():
        #     print('key')
        #     print(key)
        #     print(flush=True)
        #     print('variables.items()')
        #     print(variables.items())
        #     print(flush=True)
        #     print('value')
        #     print(value)
        #     print(flush=True)

        #     if isinstance(value, Polygon):  # 假设多边形对象为Polygon类
        #         variables[key] = value.to_dict()  # 使用to_dict()方法将多边形对象转换为字典

        # # 将数据写入文件
        with open(filename, 'w', encoding='utf-8') as file:
            for key, value in variables.items():
                # print(f"{key}: type is {type(value)}\n")
                # print(flush=True)
                # if isinstance(value, list):
                #     print('type(value[0])')
                #     print(type(value[0]))
                #     print(flush=True)
                #     print((value[0]))
                # if key == 'cache':
                #     print('cache pass')
                #     continue
                file.write(f"{key}: {value}\n")
                # try:
                #     # if key == 'obstacle_list':
                #     #     continue
                #     # if key == 'ob_corridor_list':
                #     #     continue
                #     # if key == 'cache':
                #     #     continue

                #     # file.write(f"{key}: type is {type(key)}\n")
                #     file.write(f"{key}: {value}\n")
                # except Exception as e:
                #     print('**********************************************')
                #     print("An exception has occurred : ", str(e))
                #     print('**********************************************')
                #     print(f"{key}: type is {type(value)}\n")
                #     # file.write(f"{key}: type is {type(key)}\n")
                #     file.write(f"{key}: type is {type(value)}\n")
        # # TypeError: Object of type polygon is not JSON serializable
        # with open(filenameCanLoad, 'w', encoding='utf-8') as file:
        #     # json.dump(variables, file, ensure_ascii=False)  # 这一步出错
        #     json.dump(variables, file, ensure_ascii=False,
        #               cls=NumpyEncoder)  # 这一步出错，但是加上解码后修正好了
    else:
        variables = my_object
        # 将数据写入文件
        with open(filename, 'w', encoding='utf-8') as file:
            file.write(f"{my_object}\n")


def save_pickle(agent_list, onlyfilename):
    # 打包存储UAV对象列表
    import pickle
    filename = path_dir + 'agent100/' + onlyfilename+".pkl"
    create_file(filename)
    with open(filename, "wb") as f:
        pickle.dump(agent_list, f)
    # # 加载并读取存储的UAV对象列表
    # with open("uav_list.pkl", "rb") as f:
    #     loaded_uav_list = pickle.load(f)


def str2time(time_str1, time_str2):
    from datetime import datetime

    # # 时间字符串
    # time_str1 = "2023-08-13_21-12-23"
    # time_str2 = "2023-08-13_21-15-30"

    # 将时间字符串转换为datetime对象
    time1 = datetime.strptime(time_str1, "%Y-%m-%d_%H-%M-%S")
    time2 = datetime.strptime(time_str2, "%Y-%m-%d_%H-%M-%S")

    # 计算时间差
    time_diff = time2 - time1

    # 打印时间差
    # print("Time difference : ", time_diff)
    return time_diff


def save_agent100(agent_list):
    # print(agent_list)
    # print(
    #     'agent props have been saved in : "002/agent100/agent_list[i].json"')
    print(
        # f'agent props have been saved in : "{test_plan}/agent100/agent_list[i].json"')
        f'agent props have been saved in : "{path_dir}/agent100/agent_list[i].json"')
    for i in range(len(agent_list)):
        saveJSON(agent_list[i],
                 f'agent_list[{i}]')
    save_pickle(agent_list, 'agent_list_100')

    end_time = get_current_datetime_formatted_file_name()
    print('The current time when the window is closed : ', end_time)
    print('The total program running time is : ',
          str2time(datetime_from_sh(), end_time))
