import random
import subprocess
import time
# 假设你的数据保存在一个列表中，这里用data_list表示
data_list = ['65 37', '61 69', '63 111', '59 142', '157 158']

def run_program_with_seed(seed):
    # 使用给定的种子运行程序，并返回输出结果
    command = '.\judge\SemiFinalJudge.exe -m .\judge\maps\map1.txt  -d ./script/output.txt .\script\main.exe -l NONE -s ' + str(seed)  # 替换'your_program'为你的程序名称
    process = subprocess.Popen(command, stdout=subprocess.PIPE)

    time.sleep(0.1)
    process.kill()
    # output, _ = process.communicate()

    file_path = './script/output.txt'  # 文件路径
    with open(file_path, 'r') as file:
        # 读取文件内容
        file_contents = file.read()
    return file_contents.strip().split('\n')

def compare_data(output, data):
    # 比较程序输出和你的数据是否相同
    # 这里假设数据是字符串列表
    goodsInfo = []
    for g in output[-8:-3]:
        good = g.split(' ')
        goodsInfo.append(good[0] + ' ' + good[1])
    if set(goodsInfo) == set(data):
        return True
    else:
        return False

def find_matching_seed(data):
    # 遍历可能的随机种子，并找到匹配的种子
    for seed in range(100000):  # 假设随机种子范围是0到9999
        output = run_program_with_seed(seed)
        if compare_data(output, data):
            return seed

    return None

# print('\n')
# print(run_program_with_seed(1))
# exit()

matching_seed = find_matching_seed(data_list)
if matching_seed is not None:
    print("Found matching seed:", matching_seed)
else:
    print("Matching seed not found.")
