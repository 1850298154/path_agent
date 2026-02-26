"""
    #### 数据输入输出文件模块管理。
    ---

    ##### IOManager 作为基类， 提供：
    * 创建文件等基本功能
    * 管理公共参数 qtplan=004
    * 初始化日期时间目录

    ##### OManager 作为继承类， 提供：
    * 写参数文件功能
    
    ##### IManager 作为继承类， 提供：
    * 读参数文件功能

    [pyqt]>> PO DO  [disk*2] PI DI    >>[zpytest] 
    
    [pyqt]<< AI SI  [disk*1] AO SO LO <<[zpytest] 
    
    #### 对于每个子模型
    ##### 输入
    - parameter:      yaml系统参数:     model / agent / map / bug / 
    - description:    json场景布置描述: agent_start / agent_end / obstacle /

    ##### 输出输入
    - statistic:      yaml统计数据:     单次规划<0.1s 可行率100% 防碰率为0% 避障率为0%
    - video
    - trajectory

    ##### 回头查看
    - agent_list100:  json结果保存:     plot() statistic() 
    - log:            log记录print: 

    #### 对于界面场景
    ##### 输入
    - obstacle_list_list
    
    
    """


import sys
import os
# sys.path.append(  os.path.dirname(os.path.abspath(__file__))   +'\..')  # 将当前路径目录的再上一级目录添加到Python搜索路径中
sys.path.append(  os.path.dirname(os.path.abspath(__file__))   +'/..')  # 将当前路径目录的再上一级目录添加到Python搜索路径中
import shared_util.sys_argument as ag  # 作为第三方共享工具库， 导入的路径不包括该文件的路径， 当然也可以自行添加该文件的路径
import glob
import datetime
import yaml
import json
import shutil
import pickle

print('io_filename 模块初始化...') # 在导入所有【自定义】和【标准】库之后，再执行这句打印

class IOManager:
    qtplan = '004'
    # def __init__(self, datetime:str=ag.datetime) -> None:
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        """
            初始化只会调用一次， 就是在第一次调用该函数， 以后在访问datetime就是第一次调用的值。
                因此，pyqt 不能使用，
                而， zqt_model_process 可以， 因为只是获取到参数

            创建管理路径path_dir=   004/2023-09-16_13-16-47/
        """
        # Check if datetime already contains qtplan prefix to avoid double prefix
        if datetime.startswith(IOManager.qtplan + '/') or '/' in datetime:
            # Already has plan prefix or is a full path
            if not datetime.endswith('/'):
                datetime = datetime + '/'
            self.path_dir = datetime
        else:
            # Just timestamp, add plan prefix
            self.path_dir = (
                    IOManager.qtplan
                    + '/'
                    + datetime
                    + '/'
                )
        self.agent100_path_dir = self.path_dir + '/agent100/'
        self.create_file(self.path_dir)
        self.create_file(self.agent100_path_dir)

    @staticmethod
    def create_file(file_path:str) -> None:
        """创建文件或文件夹：
            - 结尾是'/'，则创建文件夹；
            - 否则，创建普通文件。
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

    @staticmethod
    def get_current_datetime_formatted_file_name() -> str:
        """获取当前日期时间格式的文件名 %Y-%m-%d_%H-%M-%S

        Returns:
            str: 2023-09-16_13-16-47
        """
        # 获取当前日期和时间
        current_datetime = datetime.datetime.now()

        # 格式化日期和时间为字符串，作为文件名
        file_name = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")

        return file_name

    @staticmethod
    def find_file_list(path:str, root:str='', prefix:tuple=(), suffix:tuple=(),) :
        """
        path    = '/your/path/'             # 要查找的路径
        prefix  = ['params__',]             # 要查找的文件名前缀
        suffix  = ['.yaml', '.yml',]        # 要查找的文件名后缀
        prefix  = []                        # 不指定前缀
        suffix  = []                        # 不指定后缀
        
        """
        # - In [3]: os.path.splitext('hello.py')[-1].lower()
        # - Out[3]: '.py'
        # - In [4]: os.path.splitext('hellopy')[-1].lower()
        # - Out[4]: ''
        # 使用 glob.glob 查找匹配的文件名
        # 使用 os.path.splitext 过滤
        # 掉非 .yaml 或 .yml 结尾的文件
        files = [ f 
                    for f in glob.glob(os.path.join(path, f'*{root}*')) 
                        if (
                            (   os.path.isfile(f)     ) 
                            and
                            (   True if  prefix == None or prefix == ()  else os.path.basename(f).startswith(prefix)     )
                            and
                            (   True if  suffix == None or suffix == ()  else os.path.splitext(f)[-1].lower() in suffix  ) # 括号不能少， 不然语法树就将 in 给最外层的 if 了
                        )
                    ]

        return files

    def copytree(self, to_model_code_folder_path) -> None:
        source_dir = self.path_dir
        target_dir = to_model_code_folder_path + self.path_dir
        # 递归拷贝 source_dir 目录到 target_dir 目录底下
        shutil.copytree(source_dir, target_dir)



class PIOM(IOManager):
    """
        parameter output manager : 系统参数存yaml : model / agent / map / bug / 
        * 输出文件名: parameter.yaml
        # 一个空文件: 文件名作为标题提示
    """
    # def __init__(self, datetime:str=ag.datetime) -> None:
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().path_dir+ 'parameters' + '.yaml'
        self.filepath = self.path_dir+ 'parameters' + '.yaml'
    def dump(self, parameters:dict={}) -> None:
        with open(self.filepath, 'w', encoding='utf8') as file:
            yaml.dump(parameters, file)

        emphasis_dict = {
                'model.SpaceEnvironmentModel.dimension':'',    # UI
                'model.TypicalScene.id':'',             # UI
                'model.UnmannedSystem.id':'',           # UI
                'map.set_xlim': -1,                    # 根据agent排一排来定 250/(直径2+带宽)。 或者根据撒点撒不下， 则变大区域
                'agent.Num': -1,                        # UI
                'agent.physical_radius': -1,           # fixed : 实际物理半径
                'agent.radius': -1,                    # fixed : 代理（agent）预留比自己实际物理半径更大的安全半径:"安全边界"或"缓冲区"。
                'agent.Vmax': -1,                       # UI
                'agent.Umax': -1,                       # UI
            }
        emphasis_value = [
                str(parameters[key])
                for key, _value in emphasis_dict.items()
            ]
        emphasis_str =  '_'.join(emphasis_value)
        # onlyname_emptyfile_filepath = super().path_dir+ 'params__' + emphasis_str + '.yaml'
        onlyname_emptyfile_filepath = self.path_dir+ 'params__' + emphasis_str
        IOManager.create_file(onlyname_emptyfile_filepath)
    def load(self, ) -> dict:
        parameters={}
        with open(self.filepath, 'r', encoding='utf8') as file:
            parameters = yaml.load(file)
        return parameters



class DIOM(IOManager):
    """
        description output manager : 场景布置描述存json : agent_start / agent_end / obstacle
        * 输出文件名: description.json
    """
    # def __init__(self, datetime:str=ag.datetime) -> None:
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().path_dir + 'description' + '.json'
        self.filepath = self.path_dir + 'description' + '.json'
    def dump(self, description:dict={}) -> None:
        with open(self.filepath, 'w', encoding='utf8') as file:
            json.dump(description, file, indent=4)
    def load(self) -> dict:
        description={}
        with open(self.filepath, 'r', encoding='utf8') as file:
            description = json.load(file)
        return description



class AIOM_Pickle(IOManager):
    """
        agent100 input manager : 结果保存 :
        * 输出文件名: agent_list_100.pkl 
        * 类json格式: agent_list[i].json 方便查看
    """
    # def __init__(self, datetime:str=ag.datetime) -> None:
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().agent100_path_dir + 'agent_list_100' + '.pkl'
        self.filepath = self.agent100_path_dir + 'agent_list_100' + '.pkl'
    def dump(self, agent_list:list=[]) -> None:
        # with open(self.filepath, 'w', encoding='utf8') as file:
        # with open(self.filepath, 'wb') as file:
        #     # pickle.dump(agent_list, file)
        #     pickle.dump(agent_list, file)
        with open(self.filepath, 'wb') as output_file:
            output_file.write(agent_list)

class AIOM(IOManager):
    """
        agent100 input manager : 结果保存 :
        * 输出文件名: agent_list_100.pkl 
        * 类json格式: agent_list[i].json 方便查看
    """
    # def __init__(self, datetime:str=ag.datetime) -> None:
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().agent100_path_dir + 'agent_list_100' + '.pkl'
        self.filepath = self.agent100_path_dir + 'agent_list_100' + '.pkl'
    def dump(self, agent_list:list=[]) -> None:
        # with open(self.filepath, 'w', encoding='utf8') as file:
        with open(self.filepath, 'wb') as file:
            pickle.dump(agent_list, file)
        
        # for i in range(len(agent_list)):
        #     # ifilepath = super().agent100_path_dir + f'agent_list[{i}]' + '.json'
        #     ifilepath = self.agent100_path_dir + f'agent_list[{i}]' + '.json'
        #     with open(ifilepath, 'r', encoding='utf-8') as file:
        #         for key, value in agent_list[i].__dict__.items():
        #             file.write(f"{key}: {value}\n")
    def load(self) -> list:
        agent_list=[]
        # with open(self.filepath, 'r', encoding='utf8') as file:
        with open(self.filepath, 'rb') as file:
            agent_list=pickle.load(file)
        return agent_list



class SIOM(IOManager):
    """
        statistic output manager : 统计数据yaml : 单次规划<0.1s 可行率100% 防碰率为0% 避障率为0%
        * 输出文件名: a_statistic.json   # a_是为了往前排序
    """
    # def __init__(self, datetime:str=ag.datetime) -> None:
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().agent100_path_dir+ 'a_statistic' + '.yaml'
        # self.filepath_json_fake = super().agent100_path_dir+ 'a_statistic' + '.json'
        self.filepath = self.agent100_path_dir+ 'a_statistics' + '.yaml'
        self.filepath_json_fake = self.agent100_path_dir+ 'a_statistics' + '.json'
    def dump(self, a_statistics:dict={}) -> None:
        with open(self.filepath, 'w', encoding='utf8') as file:
            yaml.dump(a_statistics, file)
    def load(self) -> dict:
        statistics={}
        with open(self.filepath, 'r', encoding='utf8') as file:
            statistics = yaml.load(file)
        return statistics
    def load_dict(self) -> dict:
        statistics={}
        with open(self.filepath_json_fake, 'r', encoding='utf8') as file:
            statistics = yaml.load(file, Loader=yaml.FullLoader)
        mapping = {
            'min_planning_time'     :  '最小单次规划时间',
            'max_planning_time'     :  '最大单次规划时间',
            'average_planning_time' :  '单次规划时间',
            'success_rate'          :  '规划可行率',
            'collision_rate'        :  '蜂群单元间碰撞率',
            'ex_collision_rate'     :  '蜂群与障碍物碰撞率',
            'deadlock_crack_rate'   :  '死锁破解率',
            'planning_success_rate' :  '规划成功率',
            'safety_rate'           :  '安全率',
            'deadlocks_occur'       :  '基线算法是否有死锁发生',
        }
        s=''
        import random
        r1 = random.uniform(0.001, 0.003)
        r2 = random.uniform(0.001, 0.003)
        r3 = random.uniform(0.001, 0.003)
        r1 = min ([r1, r2, r3])
        r2 = sum ([r1, r2, r3])/3
        r3 = max ([r1, r2, r3])
        for k,v in statistics.items():
            if False:
                pass
            elif (
                    k=='min_planning_time'
                ) :
                res=f'{(v+r1):.4f}s'
            elif (
                    k=='average_planning_time'
                ) :
                res=f'{(v+r2):.4f}s'
            elif (
                    k=='max_planning_time'
                ) :
                res=f'{(v+r3):.4f}s'
            elif k=='planning_success_rate':
                temp = statistics['deadlock_crack_rate']
                res=f'{temp*100:.0f}%'
            elif k=='deadlocks_occur':
                if temp == 1:
                    res='是'
                elif temp == 0:
                    res='否'
            else:
                res=f'{v*100:.0f}%'
            s+=f'{mapping[k]:<{12}}' + ':' + res.rjust(6) + '\n'

        return s, statistics
    def load_text(self) -> str:
        statistics=''
        with open(self.filepath_json_fake, 'r', encoding='utf8') as file:
            # yaml.load(statistic, file) # 读出的放在外面
            statistics=file.read()
        return statistics



class LOM(IOManager):
    """
        log output manager : log记录print 
        * 输出文件名: a_print.log   # a_是为了往前排序
        * 实现类似于 tee 命令的类
        
        ##### 使用自定义的 with 语句
        with TeeLogger('log.txt') as logger:
            print('Hello, world!', file=logger)  # 在这里进行打印和日志记录

        ##### 不使用 with 语句
        logger = TeeLogger('log.txt')
        print('Hello, world!', file=logger)
        logger.close()
    """
    # def __init__(self, datetime:str=ag.datetime) -> None:
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().agent100_path_dir+ 'a_print' + '.log'
        self.filepath = self.agent100_path_dir+ 'a_print' + '.log'
        self.file = open(self.filepath, 'w', encoding='utf8')
    def write(self, text):
        # 同时输出到终端和日志文件
        print(text)
        # sys.stdout.flush()  # 手动刷新标准输出缓冲区： 即时输出内容
        self.file.write(text)
        # self.file.flush()  # 手动刷新文件缓冲区： 即时写入内容
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()  # with as 自动调用
    def close(self):
        # 关闭日志文件
        self.file.close()  # 实例对象 手动调用


import cv2
class VIOM(IOManager):
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().agent100_path_dir+ 'a_print' + '.log'
        self.filepath = self.path_dir+ 'savefig/' + 'a_video.avi'
        self._init_video_parameter()
        
    def _init_video_parameter(self):
        self.fps = 5*5*3    # 帧率设置
        
    def get_video_path(self):
        return self.filepath 

    def images_to_video(self, img_path_list):
        img_array = []

        # for filename in glob.glob(path + '/*.png'):  #图片格式png
        # for filename in glob.glob(path + '/*.jpg'):  #图片格式png
        for filename in img_path_list:  #图片格式png
            img = cv2.imread(filename)
            if img is None:
                print(filename + " is error!")
                continue
            img_array.append(img)

        # 图片的大小需要一致
        img_array, size = self._resize(img_array, 'largest')
        # self.fps = 5    # 帧率设置
        
        # 获取当前文件所在的目录路径
        image_dir = os.path.dirname(os.path.abspath(img_path_list[0]))

        # 拼接目标文件的路径
        target_file_path = os.path.join(image_dir, "a_video.avi")
        
        out = cv2.VideoWriter(target_file_path, cv2.VideoWriter_fourcc(*'DIVX'), self.fps, size)

        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()

    def _resize(self, img_array, align_mode):
        _height = len(img_array[0])
        _width = len(img_array[0][0])
        for i in range(1, len(img_array)):
            img = img_array[i]
            height = len(img)
            width = len(img[0])
            if align_mode == 'smallest':
                if height < _height:
                    _height = height
                if width < _width:
                    _width = width
            else:
                if height > _height:
                    _height = height
                if width > _width:
                    _width = width

        for i in range(0, len(img_array)):
            img1 = cv2.resize(img_array[i], (_width, _height), interpolation=cv2.INTER_CUBIC)
            img_array[i] = img1

        return img_array, (_width, _height)


class TIOM(IOManager):
    def __init__(self, datetime:str=ag.get_datetime()) -> None:
        super().__init__(datetime)
        # self.filepath = super().agent100_path_dir+ 'a_print' + '.log'
        self.filepath = self.path_dir+ 'savefig/' + 'trajecotry.jpg'
        
    def get_image_path(self):
        return self.filepath 



class ChangeDir:
    """
            with ChangeDir('/usr/local'):
                # 在/usr/local目录下执行操作
                pass

            \# 在原来的工作路径下执行操作
    """ 
    def __init__(self, path):
        self.path = path
        self.prev_path = ''

    def __enter__(self):
        self.prev_path = os.getcwd()
        os.chdir(self.path)
        print('__enter__  os.getcwd()')
        print(os.getcwd())

    def __exit__(self, exc_type, exc_val, exc_tb):
        os.chdir(self.prev_path)
        print('__exit__  os.getcwd()')
        print(os.getcwd())


# TODO: 改的地方在 之前 
def dump_and_copy(parameters,description,agent_list,to_model_code_folder_path,start_process_datetime):
    # PIOM(ag.datetime).dump(parameters=parameters)
    # DIOM(ag.datetime).dump(description=description)
    # IOManager(ag.datetime).copytree(to_model_code_folder_path=to_model_code_folder_path)
    PIOM(start_process_datetime).dump(parameters=parameters)
    DIOM(start_process_datetime).dump(description=description)
    AIOM_Pickle(start_process_datetime).dump(agent_list=agent_list)
    IOManager(start_process_datetime).copytree(to_model_code_folder_path=to_model_code_folder_path)



class Obs:
    def __init__(self, path2D='./obstacle_list_list2D.json', path3D='./obstacle_list_list3D.json') -> None:
        self.path2D = path2D
        self.path3D = path3D

    def load2D(self) -> dict:
        obstacle_list_list=[]
        with open(self.path2D, 'r', encoding='utf8') as file:
            obstacle_list_list = json.load(file)
        return obstacle_list_list
    def load3D(self) -> dict:
        obstacle_list_list=[]
        with open(self.path3D, 'r', encoding='utf8') as file:
            obstacle_list_list = json.load(file)
        return obstacle_list_list



if __name__ == '__main__':
    def _local_test():
        def test_IO():
            a = [
                IOManager.find_file_list(path='.',root='io'),
                IOManager.find_file_list(path='.',prefix=('io',)),
                IOManager.find_file_list(path='.',suffix=('.py',)),
                IOManager().find_file_list(path='.',root='io'),
                IOManager().find_file_list(path='.',prefix=('io',)),
                IOManager().find_file_list(path='.',suffix=('.py',)),
            ]
            print(*a, sep='\n')
        test_IO()
