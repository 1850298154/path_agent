# OB/zyaml.py
"""
            《zyaml.py 文件为三个进程服务》
zyaml.py 
    一个操作： _______________________________
        程序任务：
            1. 批量产生测试数据集yaml文件。
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣
zpytest.py 
    是批量测试启动，会调用zyaml.py里面的函数。
    两个操作： _______________________________
        测试开始：
            1. 获取最新文件夹，
            2. 取出其文件夹中zyaml.py产生的yaml文件路径。
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣
    之后zpytest.py将yaml路径作为参数，
        启动并传给app.py进程(第二个参数)。

app.py
    在导入zyaml.py解析包时。自动获取从进程参数中，
    获取zpytest.py传入的要测试yaml路径。
    两个操作： _______________________________
        程序开始：
            1. 读取yaml数据，
            2. 将yaml文件拷贝到自己测试目录下。
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣
"""
import os
import math
import yaml
import output_filename as of


Num = 100
Num = 3
# Num = 10
# Num = 20
# Num = 0 # 不能为0
# Num = 1
temp_set_xlim = 10
temp_set_xlim = 100
temp_set_xlim = 15
temp_set_xlim = 20
temp_set_xlim = 100
temp_set_xlim = 150
temp_set_xlim = 200
temp_set_xlim = 250
temp_set_xlim = 50
# temp_square = 6
# 面积的20% 除以个数，求出一个ob面积后再开根号，上取整
ob_rate = 0.2
# ob_rate = 0.6
# ob_rate = 0.000001
# ob_rate = 0.01
# ob_rate = 0.00001
# ob_rate = 0 # 测试通过的
ob_num = 200
ob_num = 180
ob_num = 80
ob_num = 50
# ob_num = 5
ob_num = 1
# ob_num = 2
# ob_num = 3
# ob_num = 4
# ob_num = 5
# ob_num = 6
# ob_num = 7
# ob_num = 8
# ob_num = 9
# ob_num = 10
# ob_num = 11
# ob_num = 12
# ob_num = 20
# temp_square = (((temp_set_xlim**2)*ob_rate)/ob_num)**0.5
if ob_num != 0:
    temp_square = (((temp_set_xlim**2)*ob_rate)/ob_num)**0.5
else:
    temp_square=0 # 障碍物面积为0的时候， 障碍物列表直接为 空列表就行 []
# temp_square = math.ceil(temp_square)
# print('temp_square')
# print(temp_square)
# temp_square = 45  # 一个
# temp_square = 26  # 三个
# temp_square = 5
# temp_square = 45
episodes = 400
episodes = 450
episodes = 330
# episodes = 100
# episodes = 4
# episodes = 10
# episodes = 2
# episodes = 50

physical_radius=0.5
physical_radius=0.3
parameters = {
    'baseline_bool': False,
    'zr.set_xlim': temp_set_xlim,
    'zr.set_ylim': temp_set_xlim,
    'avoid.m': 10,
    # 'avoid.m': 1,
    # 'avoid.m': 3,
    # 'avoid.m': 30,
    # 'Num': 10,
    'Num': Num,
    # 'Vmax': 5, # U3 min 1.5096772374584018  加上OB_fallback  1.7877041623289285
    # 'Vmax': 10, # U3 min 1.33  加上OB_fallback
    # 'Vmax': 15, # U3 min 1.9997762671118342  加上OB_fallback  0.3329666474078957    0.9778705676935389  1.4188438054309165  1.1251312827118116
    # 'Vmax': 20, # U3 min 0.354206862507221
    # 'Umax': 20.5, # --------------------------
    
    # 'Vmax': 5, # U3 加上OB_fallback  -1
    # 'Umax': 1, # ---------------------  
    # 'Vmax': 28, # -1 1.5964642868692207 
    # 'Umax': 4, # ---------------------  
    # 'Vmax': 17, #  1.9651343307467621  1.9651343307467621
    # 'Umax': 4, # ---------------------  
    # 'Vmax': 28, #  0.36319146128866225
    # 'Umax': 400, # ---------------------   
    # 'Vmax': 28, #  1.6692824195069143
    # 'Umax': 10, # ---------------------   
    
    # 'Vmax': 20, # 0.9489122991581382 1.1129253653718423 
    # 'Umax': 10, # ---------------------  ↑  
    # 'Vmax': 20, #  -1 1.999063186538059 1.6716501096227765
    # 'Umax': 4, # ---------------------  ↑  v实际 = 4.393
    # 'Vmax': 20, #   1.2086314019449713  1.461056311554162
    # 'Umax': 6, # ---------------------    ↑
    # 'Vmax': 20, #   1.9802158222480264
    # 'Umax': 2, # ---------------------    ↑
    # 'Vmax': 200, #   1.960949806707488
    # 'Umax': 2, # ---------------------    ↑
    
    # 改了 撒点距离障碍物
    # 'Vmax': 28, #  1.6945367734164056 1.8475618453509648
    # 'Umax': 6, # ---------------------  
    # 'Vmax': 6*1.2, #  1.362350748013121 1.9763954674790318
    # 'Umax': 200, # ---------------------  
    # 'Vmax': 1, #     M=10: 
    # 'Umax': 200, # ---------------------  
    'Vmax': 6, #     M=10:  1.7563139084773223
    'Umax': 8, # ---------------------  
    
    # 一个障碍物
    # 'Vmax': 28, # 1.8348727666118223   1.2043935529420398
    # 'Umax': 6, # ---------------------  
    # 'Vmax': 6*1.2, #  1.814134386996746   M=10: 1.413254794166334
    # 'Umax': 200, # ---------------------  
    # 'Vmax': 1, #  -1
    # 'Umax': 200, # ---------------------  
    
    # # 穿障碍物 # 现在没有？？ 速度够大， 但是只加入了 1个障碍物
    # 'Vmax': 2800, # 
    # 'Umax': 4000, # ---------------------  

    # 'Vmax': 28, # 
    # 'Umax': 4, # ---------------------  
    

    'episodes': episodes,  # 测试3步就可以了
    # 'r_min': 0.3,
    # 'radius': 0.3,
    # 'radius': 0.6,
    'radius': 2*physical_radius,
    'physical_radius': physical_radius,
    # 'radius': 1.0,
    # 'physical_radius': 0.5,
    'buffer': 0.03,
    # -----------------------------
    'zr.ob_num': ob_num,
    'zr.ob_rate': ob_rate,
    'zr.lower_limit_Square_side_length': temp_square,
    'zr.upper_limit_Square_side_length': temp_square,
    'zr.more_inflated_size': -0.,  # 后续更正
    'bug.inflated_size': -0.,  # 后续更正
    'bug.bug_step_size': 0.5,
    # 'bug.num_tracks': 10,
    'bug.num_tracks': 2,  # -1
    # 'bug.num_tracks': 4+10//ob_num,  # -1
    # 'bug.num_tracks': 1+10//ob_num,  # -1
    # 'bug.num_tracks': 0+1//ob_num,  # -1
    # 'bug.num_tracks': 0+3//ob_num, # 1.5395006978672532  1.7003479209020491
    # 'bug.num_tracks': 0+5//ob_num, # 1.3429892838655244  1.8846724341763526
    # 'bug.num_tracks': 0+7//ob_num, # 1.4373036183464694  1.7513517208141716
}

# parameters = {
#     # 'zr.set_xlim': 100,
#     # 'zr.set_ylim': 100,
#     'zr.set_xlim': temp_set_xlim,
#     'zr.set_ylim': temp_set_xlim,
#     'zr.ob_rate': 0.2,
#     # 'zr.lower_limit_Square_side_length': 45,
#     # 'zr.upper_limit_Square_side_length': 45,
#     'zr.lower_limit_Square_side_length': temp_square,
#     'zr.upper_limit_Square_side_length': temp_square,
#     # 'zr.more_inflated_size': 0.3,
#     'zr.more_inflated_size': -0.,  # 后续更正
#     # 'bug.inflated_size': 0.3,
#     'bug.inflated_size': -0.,  # 后续更正
#     'bug.bug_step_size': 0.5,
#     'buffer': 0.03,
#     'avoid.m': 10,
#     # 'Num': 100,
#     'Num': Num,
#     'Vmax': 3,
#     'Umax': 2,
#     'episodes': 3,
#     # 'r_min': 0.3,
#     # 'radius': 0.3,
#     'radius': 0.6,
#     'bug.num_tracks': 10,
# }
if parameters['zr.lower_limit_Square_side_length'] > parameters['zr.set_xlim']:
    print('Obstacles larger than the map')
    exit()
# if parameters['bug.inflated_size'] < 2*parameters['radius']:
# if parameters['bug.inflated_size'] < 2*2*parameters['radius']:
if parameters['bug.inflated_size'] < 3*2*parameters['radius']:
    print('****************************************************************')
    print('radius larger than bug.inflated_size')
    print("parameters['bug.inflated_size'] = parameters['radius']")
    # 一定是两个半径， 一个agent直径。 这样 力平衡，向障碍物靠近半径的距离，就可以解开死锁。
    # parameters['bug.inflated_size'] = 2*parameters['radius']
    # parameters['bug.inflated_size'] = 2*2*parameters['radius']
    parameters['bug.inflated_size'] = 6*2*parameters['radius']
    print('****************************************************************')
parameters['zr.more_inflated_size'] = (
    # parameters['bug.inflated_size']
    # +
    # parameters['bug.num_tracks']  # n个轨道
    # * 2*parameters['radius']
    # - parameters['radius']
    # # 轨道宽度是，agent直径，两个半径
    # # 最后agent别跑出去了
    parameters['bug.inflated_size']
    +
    parameters['bug.num_tracks']  # n个轨道
    * parameters['radius']
    + parameters['radius']  # 加一个外延，不同障碍物最外面agent之间不会碰撞
    # - parameters['radius']
    # 轨道宽度是，agent直径，两个半径
    # 最后agent别跑出去了
)

# parameters['zr.more_inflated_size'] = (
#     parameters['bug.inflated_size']
#     +
#     parameters['bug.num_tracks']  # n个轨道
#     * 2*parameters['radius']
#     - parameters['radius']
#     # 轨道宽度是，agent直径，两个半径
#     # 最后agent别跑出去了
# )

# def save_input_para(Num, ini_x, ini_v, target, r_min, epsilon, h, K, episodes):
#     # zrand 不能引用， 形成循环了。
#     import yaml
#     import output_filename as of
#     import zrand as zr
#     global parameters
#     parameters = {
#         'Num': Num,
#         # 'ini_x': ini_x,
#         # 'ini_v': ini_v,
#         # 'target': target,
#         'r_min': r_min,
#         'epsilon': epsilon,
#         'h': h,
#         'K': K,
#         'episodes': episodes,
#         'zr.set_xlim': zr.set_xlim,
#         'zr.set_ylim': zr.set_ylim,
#     }

#     with open(of.path_dir + 'parameters.yaml', 'w') as file:
#         yaml.dump(parameters, file)


############################################################
###############     zpytest.py 取最新yaml文件夹     #########
############################################################
def get_the_latest_folder_TestDataSet():
    # import os

    def walk_dirs(top, max_depth):
        if len(top) > 0:
            if top[-1] == '/' or top[-1] == '\\':
                top = top[:len(top)-1]
        ret_top_folds = []
        for root, dirs, files in os.walk(top):
            # 计算当前目录的深度
            # current_depth = root[len(top):].count(os.sep)
            current_depth = root[len(top):].count('\\')
            current_depth += root[len(top):].count('/')
            # print('current_depth')
            # print(current_depth)
            # print('root')
            # print(root)
            if current_depth >= max_depth:
                # 如果当前深度超过最大深度，则不再递归该目录
                del dirs[:]

            # # 处理文件
            # print('处理文件')
            # for file in files:
            #     file_path = os.path.join(root, file)
            #     print(file_path)

            # 处理子目录
            # print('处理子目录')
            for dir in dirs:
                dir_path = os.path.join(root, dir)
                ret_top_folds.append(dir_path)
                # print(dir_path)
        return ret_top_folds

    # 示例调用
    # ret_top_folds = walk_dirs('/path/to/directory', 2)  # 递归遍历目录树的前两层
    ret_top_folds = walk_dirs(of.TestDataSet, 1)  # 递归遍历目录树的前两层
    ret_top_folds = sorted(ret_top_folds, key=lambda x: os.path.basename(x))
    # print(*ret_top_folds, sep='\n')
    if len(ret_top_folds) == 0:
        print('There is no folder for the test dataset')
        print(f'Check that the "{of.TestDataSet}" is empty underneath')
        exit()
    else:
        return ret_top_folds[-1]


############################################################
###############     zpytest.py 取测试的yaml文件     #########
############################################################
# def get_all_files(test_dataSet_dir):
def get_all_files():
    test_dataSet_dir = get_the_latest_folder_TestDataSet()
    all_files = []
    for root, directories, files in os.walk(test_dataSet_dir):
        for file_name in files:
            file_path = os.path.join(root, file_name)
            # all_files.append(file_path)
            if file_name.lower().endswith(('.yaml', '.yml')):
                all_files.append(file_path)
    # 按照文件名进行排序
    all_files = sorted(all_files, key=lambda x: os.path.basename(x))
    return all_files


# print(get_the_latest_folder(of.TestDataSet), 'aa')
# print('get_the_latest_folder(of.TestDataSet)')
# exit()

############################################################
###############     zyaml.py 先产生yaml文件     #############
############################################################
def produce_test_data_set():
    # import os
    # if os.name == 'nt':  # Windows系统
    #     print('                    Windows                  ')
    #     # dir = '003_nt_TestDataSet/'
    #     dir = '003/nt_TestDataSet/'
    # else:  # Linux系统
    #     print('                    Linux                    ')
    #     # dir = '003TestDataSet/'
    #     dir = '003/TestDataSet/'
    # dir  = of.TestDataSet
    dir = of.ret_produce_test_data_set_path()

    # Num = 10
    r_min = 0.3
    # Num = 100
    # map_list = list(range(150, 151, 40))
    # map_list = list(range(100, 200, 40))
    # m_list = list(range(1, 5+1, 2))
    # vmax_list = list(range(1, 3+1, 1))[::-1]
    # Num = 100
    # map_list = list(range(10, 20, 40))
    # m_list = list(range(3, 6+1, 1))
    # vmax_list = list(range(3, 3+1, 1))[::-1]
    # Num = 100
    # map_list = list(range(10, 20, 40))
    # m_list = list(range(101, 101+1, 1))
    # vmax_list = list(range(3, 3+1, 1))[::-1]
    # Num = 10
    # map_list = list(range(10, 20, 40))
    # m_list = list(range(5, 5+1, 1))
    # vmax_list = list(range(4, 6+1, 1))[::-1]
    # umax_list = list(range(0, 3+1, 1))[::-1]
    # umax_list = [umax+0.5 for umax in umax_list]
    # Num = 10
    # map_list = list(range(10, 20, 40))
    # m_list = list(range(5, 5+1, 1))
    # vmax_list = list(range(4, 7+1, 1))[::-1]
    # umax_list = list(range(1, 4+1, 1))[::-1]
    # Num = 10
    # map_list = list(range(8, 20, 40))
    # m_list = list(range(5, 5+1, 1))
    # vmax_list = list(range(4, 7+1, 1))[::-1]
    # umax_list = list(range(1, 4+1, 1))[::-1]
    # Num = 10
    # map_list = list(range(8, 20, 40))
    # m_list = list(range(10, 10+1, 1))
    # vmax_list = list(range(4, 7+1, 1))[::-1]
    # umax_list = list(range(1, 4+1, 1))[::-1]
    # Num = 10
    # map_list = list(range(8, 20, 40))
    # m_list = list(range(10, 10+1, 1))
    # vmax_list = list(range(1, 7+1, 2))[::-1]
    # umax_list = list(range(1, 12+1, 3))[::-1]
    # Num = 100
    # map_list = list(range(150, 150+1, 40))
    # m_list = list(range(10, 12+1, 2))
    # vmax_list = list(range(1, 3+1, 2))[::-1]
    # umax_list = list(range(1, 3+1, 2))[::-1]
    Num = 100
    # Num = 2
    map_list = list(range(150, 150+1, 40))
    # map_list = list(range(15, 15+1, 40))
    map_list = list(range(100, 100+1, 40))
    map_list = list(range(150, 150+1, 40))
    map_list = list(range(200, 200+1, 40))
    map_list = list(range(100, 100+1, 40))
    map_list = list(range(150, 150+1, 40))
    # map_list = list(range(100, 150+1, 50))
    # map_list = list(range(40, 40+1, 40))
    m_list = list(range(3, 12+1, 3))
    m_list = list(range(15, 15+1, 3))
    m_list = list(range(1, 1+1, 3))
    m_list = list(range(0, 0+1, 3))
    m_list = list(range(1, 1+1, 3))
    m_list = list(range(3, 9+1, 3))
    m_list = list(range(6, 9+1, 3))
    m_list = list(range(3, 9+1, 3))
    m_list = list(range(3, 3+1, 3))
    m_list = list(range(3, 9+1, 6))
    m_list = list(range(3, 3+1, 6))
    m_list = list(range(9, 9+1, 6))
    m_list = list(range(3, 9+1, 3))
    vmax_list = list(range(1, 3+1, 1))[::-1]
    vmax_list = list(range(3, 3+1, 1))[::-1]
    vmax_list = list(range(5, 5+1, 1))[::-1]
    vmax_list = list(range(3, 3+1, 1))[::-1]
    umax_list = list(range(3, 3+1, 2))[::-1]
    umax_list = list(range(3, 3+1, 2))[::-1]
    radius_list = list(range(3, 6+1, 3))[::-1]  # 自行除
    radius_list = list(range(3, 6+1, 3))[::-1]  # 自行除
    radius_list = list(range(6, 6+1, 3))[::-1]  # 自行除
    radius_list = list(range(10, 10+1, 3))[::-1]  # 自行除
    radius_percent = 0.01
    radius_percent = 0.1
    radius_list = [radius * radius_percent for radius in radius_list] # 1.0
    physical_radius_list = list(range(5,5+1,3))[::-1] # 和radius同步
    physical_radius_list = [radius * radius_percent for radius in physical_radius_list] # 0.5



    ob_num = 10
    ob_num = 1
    # ob_num = 0
    ob_rate = 0.3
    ob_rate = 0.2
    ob_rate = 0.3
    ob_rate = 0.2
    ob_num_list= list(range(1, 40+2, 20))  ################
    ob_num_list= list(range(1, 20+2, 20))  ################
    ob_num_list= list(range(20, 20+2, 20))  ################
    ob_num_list= list(range(1, 20+2, 20))  ################
    ob_num_list= list(range(20, 20+2, 20))  ################
    ob_num_list= list(range(1, 1+2, 20))  ################
    parameters['zr.ob_rate'] = ob_rate
    # square_list=[]
    # for ob_num in ob_num_list:
    #     if ob_num == 0:
    #         # temp_square = 0
    #         square_list.append(0)
    #     else:
    #         # temp_square = (((temp_set_xlim**2)*ob_rate)/ob_num)**0.5
    #         # square_list.append((((temp_set_xlim**2)*ob_rate)/ob_num)**0.5)
    #         square_list.append((((map_list[0]**2)*ob_rate)/ob_num_list[0])**0.5)
        # square_list.append(temp_square)

    # parameters['zr.lower_limit_Square_side_length'] = temp_square
    # parameters['zr.upper_limit_Square_side_length'] = temp_square

    # ob_num_list = list(range(5, 20+1, 5))[::-1]
    # ob_num_list = list(range(50, 50+1, 5))[::-1]
    # temp_square = (((temp_set_xlim**2)*0.2)/ob_num)**0.5
    # temp_square_list = [
    #     (((temp_set_xlim**2)*0.2)/ob_num)**0.5
    #     for ob_num in ob_num_list
    # ]
    # radius_list = list(range(3, 3+1, 3))[::-1]  # 自行除
    # radius_list = list(range(9, 12+1, 3))[::-1]  # 自行除
    # radius_list = list(range(7, 12+1, 2))  # 自行除

    # print(Num)
    # print(map_list)
    # print(m_list)
    # print(vmax_list)
    # print(umax_list)
    # print(radius_list)
    max_tracks = 10
    max_tracks = 0  # 没有轨道，
    max_tracks = 1  # 没有轨道，
    max_tracks = 10  # 没有轨道，
    max_tracks = 20  # 没有轨道，
    max_tracks = 0  # 没有轨道，
    max_tracks = 0  # 没有轨道，
    max_tracks = 10  # 没有轨道，
    track_base_list  =  [0,1]
    track_base_list  =  [1]
    # if ob_num == 0:
    #     parameters['bug.num_tracks'] = 0
    # else:
    #     parameters['bug.num_tracks'] = 0+max_tracks//ob_num
    
    map_list = list(range(250, 250+1, 40))
    track_base_list  =  [1] # 最底层， 不是 下限。 1+10//ob_num 中的 1 ，不是10
    m_list = list(range(10, 20+1, 10))[::-1]
    vmax_list = list(range(6, 6+1, 1))[::-1]
    umax_list = list(range(8, 8+1, 2))[::-1]
    ob_rate = 0.2
    ob_num_list= list(range(20, 20+2, 20))  ################

    map_list = list(range(250, 250+1, 40))
    max_tracks = 0  # 没有轨道，
    track_base_list  =  list(range(0, 17, 4)) # 最底层， 不是 下限。 1+10//ob_num 中的 1 ，不是10
    m_list = list(range(10, 10+1, 10))[::-1]
    vmax_list = list(range(6, 6+1, 1))[::-1]
    umax_list = list(range(8, 8+1, 2))[::-1]
    ob_rate = 0.2
    ob_num_list= list(range(1, 1+2, 20))[::-1]  ################
    # ob_num_list= list(range(20, 20+2, 20))[::-1]  ################

    map_list = list(range(250, 250+1, 40))
    max_tracks = 0  # 没有轨道，
    track_base_list  =  list(range(10, 11, 4)) # 最底层， 不是 下限。 1+10//ob_num 中的 1 ，不是10
    m_list = list(range(10, 10+1, 10))[::-1]
    vmax_list = list(range(3, 6+1, 3))[::-1]
    umax_list = list(range(8, 8+1, 2))[::-1]
    ob_rate = 0.2
    ob_num_list= list(range(1, 1+2, 20))[::-1]  ################
    # ob_num_list= list(range(20, 20+2, 20))[::-1]  ################

    print('ob_rate               ', ob_rate)
    print('map_list              ', map_list)
    # print('temp_square           ', temp_square)
    print('ob_num_list           ', ob_num_list)
    # print('square_list           ', square_list)
    print('track_base_list       ', track_base_list)
    print('bug.num_tracks        ', parameters['bug.num_tracks'])
    print('--------------------------------------------------')
    print('Num                   ', Num)
    print('m_list                ', m_list)
    print('vmax_list             ', vmax_list)
    print('umax_list             ', umax_list)
    print('radius_list           ', radius_list)
    print('physical_radius_list  ', physical_radius_list)
    test_cnt = 1
    for map in map_list:
        for m in m_list:
            for vmax in vmax_list:
                for umax in umax_list:
                    for radius,physical_radius in zip(radius_list,physical_radius_list):
                        for ob_num in ob_num_list:
                            for track_base in track_base_list:
                                
                                if ob_num == 0:
                                    # parameters['bug.num_tracks'] = track_base 
                                    parameters['bug.num_tracks'] = 0
                                else:
                                    parameters['bug.num_tracks'] = track_base +max_tracks//ob_num
                                    # parameters['bug.num_tracks'] = 0 +max_tracks//ob_num
                                # parameters['bug.num_tracks']=min(1,parameters['bug.num_tracks'])
                                parameters['zr.set_xlim'] = map
                                parameters['zr.ob_num'] = ob_num
                                # if ob_num == 0:
                                #     temp_square = 0
                                # else:
                                #     temp_square = (
                                #         ((map**2)*ob_rate)/ob_num)**0.5
                                # # temp_square = (((map**2)*0.2)/ob_num)**0.5
                                # parameters['zr.upper_limit_Square_side_length'] = temp_square
                                # parameters['zr.lower_limit_Square_side_length'] = temp_square
                                
                                if ob_num == 0:
                                    square = 0
                                else:
                                    square = (((map**2)*ob_rate)/ob_num)**0.5
                                parameters['zr.upper_limit_Square_side_length'] = square
                                parameters['zr.lower_limit_Square_side_length'] = square
                                parameters['zr.set_ylim'] = map
                                parameters['avoid.m'] = m
                                parameters['Num'] = Num
                                parameters['Vmax'] = vmax
                                parameters['Umax'] = umax
                                # episodes = int((map*1.414)/(0.2*vmax))
                                # 最长可能走x+y的长度， 就是2*map。 /0.2 == *5
                                # episodes = int((map*2)*5//(vmax))
                                # 因为agent在阻塞的时候会停下来等其他点通过，但应该不会超过3条边长吧
                                # episodes = int((map*3)*5//(vmax))
                                # 试一下无限大
                                # episodes = int((map*2*Num)*5//(vmax))
                                episodes = int((map*4)*5//(vmax))
                                episodes = int((map*2)*5//(vmax))
                                # episodes = int((map*1.5)*5//(vmax))
                                # episodes = 2
                                parameters['episodes'] = episodes
                                # parameters['r_min'] = r_min
                                # parameters['radius'] = r_min
                                parameters['radius'] = radius
                                # physical_radius=physical_radius_list[radius_list.index(radius)]
                                parameters['physical_radius'] =physical_radius 
                                # parameters['bug.inflated_size'] = (4 *
                                #                                    (parameters['radius']
                                #                                     + parameters['buffer']))
                                # parameters['bug.inflated_size'] = (1 *
                                #                                 (parameters['radius']
                                #                                     + parameters['buffer']))
                                zfill_cnt = "{:03d}".format(test_cnt)
                                test_cnt += 1
                                if parameters['zr.lower_limit_Square_side_length'] > parameters['zr.set_xlim']:
                                    print('Obstacles larger than the map')
                                    exit()
                                # if parameters['bug.inflated_size'] < 2*parameters['radius']:
                                if parameters['bug.inflated_size'] < 2*2*parameters['radius']:
                                    print(
                                        '****************************************************************')
                                    print('radius larger than bug.inflated_size')
                                    print(
                                        "parameters['bug.inflated_size'] = parameters['radius']")
                                    parameters['bug.inflated_size'] = (
                                        2*2*parameters['radius'])
                                    print(
                                        '****************************************************************')
                                parameters['zr.more_inflated_size'] = (
                                    # parameters['bug.inflated_size']
                                    # +
                                    # parameters['bug.num_tracks']  # n个轨道
                                    # * 2*parameters['radius']
                                    # - parameters['radius']
                                    # # 轨道宽度是，agent直径，两个半径
                                    # # 最后agent别跑出去了
                                    parameters['bug.inflated_size']
                                    +
                                    parameters['bug.num_tracks']  # n个轨道
                                    * parameters['radius']
                                    # + parameters['radius']  # 加一个外延，不同障碍物最外面agent之间不会碰撞
                                    # - parameters['radius']
                                    # 轨道宽度是，agent直径，两个半径
                                    # 最后agent别跑出去了
                                )
                                # of.create_file(dir)
                                # with open(dir + f'{zfill_cnt}_a{Num}_L{map}_m{m}_v{vmax}_u{umax}_e{episodes}.yaml', 'w') as file:
                                # with open(dir + f'{zfill_cnt}_a{Num}_L{map}_m{m}_v{vmax}_u{umax}_e{episodes}_sq{temp_square:.1f}_r{radius:.3f}_o{ob_num}_t{track_base}.yaml', 'w') as file:
                                with open(dir + f'{zfill_cnt}_a{Num}_L{map}_o{ob_num}_sq{square:.1f}_t{track_base}__m{m}_r{radius:.2f}_pr{physical_radius:.2f}_v{vmax}_u{umax}_e{episodes}.yaml', 'w') as file:
                                    yaml.dump(parameters, file)


############################################################
#####     zpytest.py 启动 app.py进程 时 自动拷贝yaml     ####
############################################################
def convert(read_parameters, parameters):
    parameters = {
        'avoid.m': 10,
        'buffer': 0.03,
        'baseline_bool': read_parameters['baseline_bool'],
        'Num': read_parameters['agent.Num'],
        'Vmax': read_parameters['agent.Vmax'],
        'Umax': read_parameters['agent.Umax'],
        'radius': read_parameters['agent.radius'],
        'physical_radius': read_parameters['agent.physical_radius'],
        
        'zr.set_xlim': read_parameters['map.set_xlim'],
        'zr.set_ylim': read_parameters['map.set_ylim'],

        'zr.ob_num': read_parameters['ob.num'],
        'zr.ob_rate': read_parameters['ob.rate'],
        'zr.lower_limit_Square_side_length': read_parameters['ob.lower_limit_Square_side_length'],
        'zr.upper_limit_Square_side_length': read_parameters['ob.upper_limit_Square_side_length'],
        'obstacle_list': read_parameters['obstacle_list'] if 'obstacle_list' in read_parameters else [],
        
        'zr.more_inflated_size': read_parameters['bug.upper_limit_inflated_size'],
        'bug.inflated_size': read_parameters['bug.lower_limit_inflated_size'],
        'bug.bug_step_size': read_parameters['bug.bug_step_size'],
        'bug.num_tracks': read_parameters['bug.num_tracks'],

        'episodes': read_parameters['mpc.max_episode'],
    }
    return parameters


def read_input_para(yaml_path):
    global parameters
    # with open(of.path_dir + 'parameters.yaml', 'r') as file:
    with open(yaml_path, 'r', encoding='utf-8') as file:
        read_parameters = yaml.safe_load(file)
    print('******************************** zyaml. read_input_para')
    print('read_parameters')
    print(read_parameters)
    print('parameters')
    print(parameters)
    print('convert parameters')
    parameters = convert(read_parameters, parameters) 
    print(parameters)
    print('******************************** zyaml. read_input_para')



import os

def is_file_in_directory(file_path, directory_path):
    relative_path = os.path.relpath(file_path, directory_path)
    return not (relative_path == os.pardir or relative_path.startswith(os.sep + os.pardir))

def copy_file(source_file, target_folder):
    # 判断源文件是否在目标文件夹的子目录下
    if not is_file_in_directory(source_file, target_folder):
        import shutil
        # 使用shutil.copy()函数进行拷贝操作
        shutil.copy(source_file, target_folder)
    # else:
    #     raise ValueError('The source file is not in the target folder or its subdirectories.')

# def copy_file(source_file, target_folder):
#     # python 将一个文件从一个文件夹下面拷贝到另一个文件夹下面
#     # 源文件路径
#     # 目标文件夹路径
#     import shutil
#     # 使用shutil.copy()函数进行拷贝操作

#     shutil.copy(source_file, target_folder)


############################################################
#####     不是zpytest.py启动，app.py自己启动 生成yaml文件并读取     ####
############################################################
def create_read_parameters():
    global parameters
    data = parameters
    filename = of.path_dir + 'parameters.yaml'
    of.create_file(filename)
    # 将数据保存为YAML文件
    with open(filename, 'w', encoding='utf8') as file:
        yaml.dump(data, file)

    # 从YAML文件中读取数据
    with open(filename, 'r', encoding='utf8') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    # 打印读取到的数据
    print('Print the read YAML data')
    [print(f"{key:<35} : {value}")
     for key, value in data.items()]


def get_yaml_path_from_sh():
    import sys

    # 获取脚本文件路径
    script_path = sys.argv[0]
    print("Script path : ", script_path)

    # 获取传递给脚本的参数
    args = sys.argv[1:]
    print("Script parameters : ", args)

    if len(sys.argv) <= 2:
        # return get_current_datetime_formatted_file_name()
        # 命令 最多只有 1个参数
        # 如下三种情况：
        # 1. 单独运行此文件 zyaml.py 进程 （产生yaml测试集）
        # 2. 只有一个app.py进程运行
        # 3. 批量测试 zpytest.py 进程
        print('****************************************************************')
        print('The shell does not specify a YAML file for testing')
        print('It is not a test that is started by zpytest.py')
        print('1. It may be that a test data set is being generated')
        print('2. Only one app.py process is running')
        print('3. Batch test zpytest.py process')
        print('****************************************************************')
        script_path = sys.argv[0]
        script_name = os.path.basename(script_path)
        print('script_path', script_path)
        print('script_name', script_name)
        if script_name == 'test.py':
            create_read_parameters()
        elif script_name == 'zyaml.py':
            pass
        elif script_name == 'zpytest.py':
            pass
        elif script_name == 'zstatistic.py':
            pass

    elif len(sys.argv) >= 3:
        # 批量测试 zpytest.py 启动 app.py 进程
        # 命令 一定有 2个参数
        # 第一个参数是：时间
        # 第二个参数是：要测试的yaml文件名
        yaml_path = sys.argv[2]
        read_input_para(yaml_path)
        copy_file(yaml_path, of.path_dir)


# 这个在 zpytest.py 每次启动一个进程 app.py 时，
# 导入包时，会自动解析调用如下函数，完成对yaml数据的自动拷。
get_yaml_path_from_sh()

if __name__ == '__main__':
    produce_test_data_set()
    pass
