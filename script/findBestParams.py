import random
import subprocess
import time
import json
import itertools
import numpy as np
import heapq
import subprocess
from concurrent.futures import ThreadPoolExecutor
import tqdm

def run_program_with_seed(seed):
    # 使用给定的种子运行程序，并返回输出结果
    # main.exe: 输出"OK"就行
    command = '..\judge\SemiFinalJudge.exe -m ..\judge\maps\map1.txt  -d ./output.txt ..\\build\main.exe -l NONE -s ' + str(seed)  # 替换'your_program'为你的程序名称
    process = subprocess.Popen(command, stdout=subprocess.PIPE)

    output, _ = process.communicate()

    # 将字节字符串解码为普通的字符串，然后解析为 JSON 格式
    response_str = output.decode('utf-8')
    data = json.loads(response_str)

    # 获取 score 值
    score = data['score']
    return score

def enumerateParameters():
    # 定义一个字典，键为参数名，值为对应的取值范围
    params = {
        # 泊位聚类超参
        # "CLUSTERNUMS": range(2, 5, 1),
        "FINAL_FRAME": range(14000, 14700, 200),

        # 购买策略超参
        "maxRobotNum": range(12, 15, 1),
        # "maxShipNum": range(1, 3, 1),
        "landDistanceWeight": range(10, 20, 5),
        "deliveryDistanceWeight": range(10, 20, 5),
        "CentralizedTransportation": range(0, 2, 1),
        "robotFirst": range(0, 2, 1),

        # 机器人调度超参
        # "robot2goodWeight": np.arange(1, 1.51, 0.25),
        # "good2berthWeight": np.arange(1, 1.51, 0.25),
        # "TTL_ProfitWeight": np.arange(1, 1.51, 0.25),
        # "TTL_Bound": range(400, 501, 100),
        "PartitionScheduling": range(0, 2, 1),
        # "startPartitionScheduling": range(0, 1, 2), # 0 or maxRobotNum
        "DynamicPartitionScheduling": range(0, 2, 1),
        "robotReleaseBound": np.arange(0.5, 0.81, 0.15),
        # "DynamicSchedulingInterval": range(50, 300, 50),
        "FinalgameScheduling": range(0, 2, 1),

        # 船调度超参
        # "MAX_SHIP_NUM": range(1, 3, 1),
        # "SHIP_WAIT_TIME_LIMIT": range(0, 31, 10),
        # "GOOD_DISTANCE_LIMIT": range(50, 151, 50),
        # "EARLY_DELIVERT_FRAME_LIMIT": range(1000, 3001, 1000),
        # "EARLY_DELIVERY_VALUE_LIMIT": range(2000, 6000, 2000)
    }

    # 获取参数名和对应的取值范围
    param_names = list(params.keys())
    param_values = [params[name] for name in param_names]
    length = 1
    for val in param_values:
        length *= len(val)

    # 使用itertools.product来生成所有可能的参数组合
    all_combinations = itertools.product(*param_values)
    # print(all_combinations)

    # 将结果写入文件，耗时
    # with open('output.txt', 'w') as file:
    #     # 写入参数名作为标题行
    #     file.write(', '.join(param_names) + '\n')
        
    #     # 遍历每个参数组合
    #     for combination in all_combinations:
    #         # 将参数组合转换为字符串，并写入文件
    #         line = ', '.join(map(str, combination)) + '\n'
    #         file.write(line)
    # print("所有参数组合已成功写入到文件中。")

    return param_names, all_combinations, length


param_names, combinations, length = enumerateParameters()
print(f"参数组合数有：{length}个")
combinations = list(combinations)
random.shuffle(combinations)

min_heap = [] # 维护一个topK的小根堆
k = 30 # topK
for combination in tqdm.tqdm(combinations):
    # print(combination)
    with open("../param/param_now.txt", 'w') as fparam:
        for key,val in zip(param_names, combination):
            fparam.write(key + ' ' + str(val) + '\n')
    
    score = run_program_with_seed(10)

    if len(min_heap) < k:
        heapq.heappush(min_heap, (score, combination))
    # 如果当前得分高于堆中最小得分，替换
    elif score > min_heap[0][0]:
        heapq.heapreplace(min_heap, (score, combination))
    top_k = sorted(min_heap, reverse=True, key=lambda x: x[0])

    with open("./scores.txt", 'w') as fscore:
        fscore.write(', '.join(param_names) + ', ' + 'score' + '\n')
        for score,params in top_k:
            line = ', '.join(map(str, params)) + ', ' + str(score) + '\n'
            fscore.write(line)



