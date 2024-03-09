# 智慧船舶
## 编译
不要改动 `CMakeLists.txt`，windows 平台下编译指令如下：
```
cmake -B ./build
mingw32-make
```

由于平台上没有写权限，打包上传时要关闭 `DEBUG` 宏。