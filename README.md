# 智慧船舶
## 编译
不要改动 `CMakeLists.txt`，windows 平台下编译指令如下：
```
cmake -G "MinGW Makefiles" -B ./build
cmake --build ./build
```

## 提交到平台
关闭 utils.h 文件下 `DEBUG` 宏后运行 package.bat，会在上一级目录打包一个 zip 文件。


## 与判题器交互
```
cd build
..\judge\PreliminaryJudge.exe -m ..\judge\maps\map1.txt  -d ./output.txt .\main.exe
..\judge\SemiFinalJudge.exe -m ..\judge\maps\map1.txt  -d ./output.txt .\main.exe
```

## LOG 函数使用示例
```
LOGI("Ship ", 1 ," capacity: ", 70);
// 输出 Ship 1 capacity: 70
```

## 注意事项
由于平台上没有写权限，打包上传时要关闭 `DEBUG` 宏。

## 当前存在的bug
1. 机器人状态维护有问题，当机器人当前状态为1并且发出移动指令时，下一帧到来时机器人因为碰撞，导致停留在上一帧位置而路径变短，最后无法到达预定地点，状态出错。
2. 机器人没有到达泊位放下货物（在泊位外面）就执行pull和改变targetid、carryingItemId，导致出错；
3. 由于第一点的问题，机器人有时候手里拿着货物（carryingItem = 1）去找货物，导致targetId = goodId，最后机器人拿着goodId去访问泊位越界出错；

## 正常状态：
- 机器人分配货物时：carryingItem == 0 and carryingItemID == -1 and targetId == 货物id
- 机器人拿着货物去找泊位时：carryingItem == 1 and carryingItemID == 货物id and targetId == 泊位id
- 机器人移动时：state == 1 and 移动位置和其他机器人不重合；
- 机器人放下货物时：carryingItem == 1 and carryingItemID == 货物id and targetId == 泊位id

## 异常状态：
- carryingItem == 1 and carryingItemId == -1 and targetId == -1