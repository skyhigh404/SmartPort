# 智慧船舶
## 编译
不要改动 `CMakeLists.txt`，windows 平台下编译指令如下：
```
cmake -G "MinGW Makefiles" -B ./build
cd build && mingw32-make
```

## 与判题器交互
```
cd build
..\judge\PreliminaryJudge_win_1.exe -m ..\judge\maps\map1.txt  -d ./output.txt .\main.exe
```

由于平台上没有写权限，打包上传时要关闭 `DEBUG` 宏。