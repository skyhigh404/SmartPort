@echo off
setlocal

:: 设置要打包的目录，默认为当前目录
set "DIRECTORY=%~1"
if "%DIRECTORY%"=="" set "DIRECTORY=."

:: 设置输出的zip文件名
set "OUTPUT_FILE=../smartPort.zip"

:: 初始化一个临时文件列表
set "FILE_LIST=temp_file_list.txt"
if exist "%FILE_LIST%" del "%FILE_LIST%"

:: 查找当前目录下的.cpp和.h文件以及CMakeLists.txt，并记录到临时文件列表中
for %%f in (%DIRECTORY%\*.cpp, %DIRECTORY%\*.h, %DIRECTORY%\CMakeLists.txt) do (
    echo %%f >> "%FILE_LIST%"
)

:: 使用zip工具打包文件列表中的文件
:: 注意：这里假设你已经安装了zip工具，比如7-Zip，并且它可以通过命令行使用
zip -@ "%OUTPUT_FILE%" < "%FILE_LIST%"

:: 删除临时文件列表
del "%FILE_LIST%"

echo 打包完成: %OUTPUT_FILE%

endlocal
