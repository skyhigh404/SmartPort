import random
import subprocess
import time
import json

def run_program_with_seed(seed):
    # 使用给定的种子运行程序，并返回输出结果
    # main.exe: 输出"OK"就行
    command = '..\judge\SemiFinalJudge.exe -m ..\judge\maps\map1.txt  -d ./output.txt .\main.exe -l NONE -s ' + str(seed)  # 替换'your_program'为你的程序名称
    process = subprocess.Popen(command, stdout=subprocess.PIPE)

    output, _ = process.communicate()

    # 将字节字符串解码为普通的字符串，然后解析为 JSON 格式
    response_str = output.decode('utf-8')
    data = json.loads(response_str)

    # 获取 score 值
    score = data['score']
    return score



print('\n')
print(run_program_with_seed(10))
exit()
