# OB
import subprocess
import output_filename as of
import zstatistics as zs
import zyaml as zy
import os
import sys


# def test_get_all_files():
#     # 使用示例
#     test_dataSet_dir = "./003TestDataSet/"  # 替换为你的目录路径
#     files = get_all_files(test_dataSet_dir)
#     print(files)
#     exit(-100)


def test_one_dataset(yaml_file, pyn):
    # 要执行的CMD命令
    time_folder_name = of.get_current_datetime_formatted_file_name()
    cmd_command = (
        # 参数0
        # 'python3 app.py '
        # 'python app.py '
        pyn
        # + ' '+' app.py '
        + ' '+' test.py '
        # 参数1~n
        + ' ' + time_folder_name
        + ' ' + yaml_file
        # 重定向， 将 标准输出 和 错误输出 都进行捕获。
        + ' ' + ' 2>&1 '
    )
    # print_text_path = '002/'+time_folder_name+'/'+"print.txt"
    print_text_path = of.test_plan+'/'+time_folder_name+'/'+"print.txt"
    of.create_file(print_text_path)
    # 执行CMD命令，并实时输出结果到终端和文件
    with open(print_text_path, "w", encoding='utf8') as file:
        process = subprocess.Popen(cmd_command, shell=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, universal_newlines=True)
        for line in process.stdout:
            print(line.strip())
            file.write(line)


def main():
    # import os
    # if os.name == 'nt':  # Windows系统
    #     print('                    Windows                  ')
    #     # dir = '003_nt_TestDataSet/'
    #     dir = '003_nt_TestDataSet/'
    #     pyn = 'python'
    # else:  # Linux系统
    #     print('                    Linux                    ')
    #     # dir = '003TestDataSet/'
    #     dir = '003_TestDataSet/'
    #     pyn = 'python3'
    dir = of.TestDataSet
    pyn = of.pyn

    test_dataSet_dir = dir
    # test_dataSet_dir = "./003TestDataSet/"  # 替换为你的yaml文件所在的目录路径
    print("[print(a) for a in sys.argv]")
    [print(a) for a in sys.argv]
    if len(sys.argv) == 1:
        # files = zy.get_all_files(test_dataSet_dir)
        files = zy.get_all_files()  # zpytest.py 每次测试都会选择【最新】的文件夹作为yaml数据集输入。
        # print(*files, sep='\n')
        # exit()
        print('The set of test files is as follows : ')
        print(*files, sep='\n')
    # elif sys.argv[1] == 'one':
    else:
        print(' ** Any parameter that initiates a single test  ** ')
        files = ['']
    # print(files[0])
    test_cnt = 1
    start_date_time = of.get_current_datetime_formatted_file_name()
    for yaml_file in files:
        zfill_cnt = "{:03d}".format(test_cnt)
        print(f'This is the    {zfill_cnt}  th    round of testing')
        print('yaml_file is :  ', yaml_file)
        print(flush=True)
        test_cnt += 1

        test_one_dataset(yaml_file, pyn)

        print(flush=True)
        import time
        time.sleep(0.5)
    end_date_time = of.get_current_datetime_formatted_file_name()
    zs.statistics_n_round_test(start_date_time, end_date_time)


if __name__ == '__main__':
    main()
    # # 用于区分不同平台， 使用不同的测试文件夹和测试命令
    # if os.name == 'nt':  # Windows系统
    #     print('                    Windows                  ')
    #     test_plan = '002_nt'
    #     # TestDataSet = '003_nt_TestDataSet/'
    #     TestDataSet = '003_nt/'
    #     pyn = 'python'
    # else:  # Linux系统
    #     print('                    Linux                    ')
    #     test_plan = '002'
    #     # TestDataSet = '003_TestDataSet/'
    #     TestDataSet = '003/'
    #     pyn = 'python3'
    # test_one_dataset('', pyn)  # win 使用这个
    # test_one_dataset('', 'python')  # win 使用这个
    # test_one_dataset('', 'python3') # ubuntu 使用这个
