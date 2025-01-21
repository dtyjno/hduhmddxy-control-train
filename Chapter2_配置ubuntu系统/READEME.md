## Chapter2_配置ubuntu系统
### 0. 写在前面

0. ##**务必先确认与章节文件夹并列的README文件中的考核范围，自查内容不检查**##

0. 提供的文档仅供参考，请谨慎甄别内容是否与自己的需求相符
1. 配置仿真资料少，问题多，易崩溃。
2. 解决方案：备份系统或重装系统
3. 记录下系统配置方式，总有一天你会用到的

### 验收内容：
1. 全部完成，展示配置过程中的学习笔记
   
### 1. 我该选择什么系统 不考核
安装ubuntu24.04 （装新不装旧）

双系统：  
1. 如果硬盘空间有富余（>200G+）首选双系统，无性能损失，配置简便，一键重装
    备份：建议安装timeshift

windows虚拟机：  
1. vmware 可行（学会创建系统还原点）  
2. wsl（适用于Windows的Linux子系统） 配置费时费力，安装软件时在ubuntu系统会出现的问题之上还会出现wsl特有的问题，爱折腾首选  
    wsl系统配置见wsl.md

    [wsl安装注意事项](wsl.md)

### 1.5 换源，中文输入法······ 不考核
### 2.安装ros2————了解软件安装流程 不考核

安装习惯不好，仿真配置坐牢  
建议挂梯子访问资源  


示例1：  
ros2完善资料很多，直接找博客  
1. 百度搜索“ROS2 安装”  
2. 选择第一个页面  
3. 开始ctrl-c/v  

博客质量良莠不齐，有的是直接复制官网教程，也有的顺利解决了他的系统在安装过程中出现的疑难杂症，可以作为参考

看到博客更改系统文件要三思而后行，至少要知道自己在干什么    
实在找不到资料这种方法也可以应急  

示例2:
1. 搜索“ROS2 安装最新版”
wget http://fishros.com/install -O fishros && . fishros
评价：有轮椅是好事，但后面内容没有轮椅坐

示例3：
1. 找到ros2官网
2. 选择Releases：ubuntu24.04对应的ros2版本为 Jazzy (latest) https://docs.ros.org/en/jazzy/Installation.html
3. 选择Ubuntu包安装（了解源安装与二进制安装区别），跟着做。
提示：仔细研读官方文档的每一句话，比如知道什么是可选项什么必装，以及不要漏依赖，如安装过程中大段文字中的蓝字需要仔细查看  

示例4：
GitHub 搜索 ros2  进入项目  
找到安装方式  

安照官方文档安装页面乌龟例程验证安装  

安装疑难解答：https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html

可选：vscode下ros2编译运行

### 3. ubuntu系统 支持cuda的opencv c++版 安装（不考核但必要，可以考核后再安装）
[opencv安装示例](opencv_install.md)
### 4. 图传代码安装，安装结果不考核但必要，安装运行的过程为考核要求内容。
用于无人机上识别桶的图像识别代码位于./ros2CV.tar，目前用于在仿真中测试识别到桶的控制代码，若无需要则忽略  
github库（不建议，不支持gazebo，需要去除无用代码/更改CMakeList.txt）：https://github.com/ctoLuu/ros2-yolov5    

下文为./ros2CV.tar安装教程：  

如果使用ros2-humble(ubuntu22.04)而不是ros2-jazzy(ubuntu24.04)请将`ros2CV_ws\src\topic\src`与`ros2CV_ws\src\topic\src`内所有文件中的
```c
#include "cv_bridge/cv_bridge.hpp"
```
更改为
```c
#include "cv_bridge/cv_bridge.h"
```

验证安装：

```sh
#理解所有命令的具体功能

cd ~/
mkdir ros2_ws
#将ros2CV解压后目录放入当前文件夹
cd ros2CV_ws
colcon build --packages-select ros2_interfaces
source install/setup.sh
# 为什么要分两次编译
colcon build
#不成功问图像组
#colcon build --packages-select topic
#colcon build --packages-select topic2
cd src
chrom +x image_pub.sh
chrom +x ······
./image_pub.sh #转到~/ros2_ws/ros2CV_ws/src/image_pub.sh查看具体内容，理解该脚本的内容
./image_sub_big.sh #同上
#观察终端输出是否有关于cuda报错，若未安装支持cuda的opencv请忽略
#如有报错则需要检查配置重新安装（不排除图象识别代码出错）
```
