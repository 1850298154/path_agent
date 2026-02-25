import traceback
e1_Seg_episode = 1000000
e2_SVM_episode = 2000000
e3_MPC_episode = 3000000
e4_LFB_episode = 4000000  # last fallback


def traceback():
    print(traceback.format_exc())


# class MyCustomException(Exception):
#     def __init__(self, message):
#         self.message = message

#     def __str__(self):
#         # 获取详细的异常信息，包括位置

#         # return f"MyCustomException: {traceback.format_exc()}"
#         return f"MyCustomException: {self.message}"


class Seg_Except(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        # 获取详细的异常信息，包括位置

        # return f"MyCustomException: {traceback.format_exc()}"
        return f"Seg_Except: {self.message}"


class SVM_Except(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        # 获取详细的异常信息，包括位置

        # return f"MyCustomException: {traceback.format_exc()}"
        return f"SVM_Except: {self.message}"


class MPC_Except(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        # 获取详细的异常信息，包括位置

        # return f"MyCustomException: {traceback.format_exc()}"
        return f"MPC_Except: {self.message}"


# try:
#     # 某些可能引发 MyCustomException 的代码
#     raise MyCustomException('a')
#     # raise MyCustomException('这是自定义异常的内容')

# except MyCustomException as e:
#     # 打印自定义异常的内容
#     print(e)
#     print('------------------------')
#     print(str(e))
#     # traceback_details = traceback.format_exc()
#     # print(traceback_details)
