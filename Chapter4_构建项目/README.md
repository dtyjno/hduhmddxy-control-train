## Chapter4_构建项目 不考核
### 0. 写在前面
0. ##**务必先确认与章节文件夹并列的README文件中的考核范围，自查内容不检查**##

1. 执行命令验证环境安装

2. 可以选择安装上机代码或测试代码：  

    安装上机代码或已经给出的测试代码  
    
    安装测试代码`ros2_ws/arduoplot_ws`流程如下文所描述  

    上机代码:GitHub网址https://github.com/Hz1234567890/offboard    
        如安装上机代码且需要测试识别圆桶功能仅需按照ros_gz_sim_ardupilot软件包获取gazebo地图与gazebo中摄像头配置  


### 1. 示例代码结构介绍 

`ros2_ws` 项目是一个基于 ROS 2 框架的无人机控制项目，主要用于仿真和实际控制无人机。项目结构和功能如下：

### 项目结构
px4_ros_com：无人机控制代码   
ros2_interfaces：定义图像识别所用的ros2消息类型
ros_gz_sim_ardupilot：gazebo地图与gazebo中摄像头配置  

\Chapter4_编写无人机控制代码\ros2_ws\ardupilot_ws\src\px4_ros_com\src\examples\offboard_control\OffboardControl.cpp  
- OffboardControl.cpp

：该文件包含了无人机的离线控制代码。
- PosControl.h 和 PosControl.cpp
：这些文件定义和实现了位置控制类 

    PosControl
，用于无人机的 PID 控制。

- PID.cpp 和 PID.h：这些文件定义和实现了 PID 控制器，用于无人机的姿态和位置控制。

#### 功能介绍
1. **仿真环境配置**：项目支持 APM-SITL 和 Gazebo 仿真环境，用户可以在仿真环境中测试和调整控制算法。
2. **离线控制**：通过 OffboardControl 类实现无人机的离线控制，支持位置、速度和姿态的控制。
3. **PID 控制**：项目实现了 PID 控制器，用于无人机的精确控制。用户可以通过调整 PID 参数来优化控制效果。
4. **自动调参**（未实现）：支持自动调参功能，通过 auto_tune 方法实现 PID 参数的自动调整。

#### 使用方法
1. **安装依赖**：确保已安装 ROS 2 和相关依赖库。
2. **构建项目**：在 `ros2_ws` 目录下运行 `colcon build` 命令构建项目。
3. **运行仿真**：使用 `ros2 launch` 命令启动仿真环境，例如 `ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py`。
4. **运行控制代码**：在仿真环境中运行控制代码，例如 `ros2 run px4_ros_com offboard_control`。

通过以上步骤，用户可以在仿真环境中测试和调整无人机的控制算法，并将其应用于实际无人机的控制。
将
### 安装依赖
- px4_ros_com
```sh
#!/bin/bash
#为树莓派从头开始安装依赖项 已安装ros2
#安装mavros-humble
#添加清华源
sudo apt install curl gnupg2
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
#发行版Ubuntu 22.04 LTS (jammy)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt-get install ros-humble-mavros 
sudo apt-get install ros-humble-mavros-extras

#clone git仓库
cd ~/
mkdir ardupilot_ws
cd ardupilot_ws
#git clone https://github.com/dtyjno/drone.git
mv drone src # 更改为将附件中ardupilot_ws文件夹及其中内容移动到src文件夹下

#GeographicLib 用于gps定位
cd ~/Downloads
wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz
tar xfpz v2.3.tar.gz 
cd geographiclib-2.3 
#创建一个单独的构建目录并输入它，例如，
mkdir BUILD
cd BUILD 
#配置软件，指定源目录的路径，使用
#../configure 
#make
#make install
 cmake ..  
  make 

# colcon无法找到  geographiclib-config.cmake
  sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.22/Modules/
  
  
cd ~/ardupilot_ws
rosdep install --from-paths src --ignore-src -r -y

```
ros_gz_sim_ardupilot

Chapter3的仿真环境
https://ardupilot.org/dev/docs/sitl-with-gazebo.html

ros2_interfaces

无

### 启动程序

理解各个脚本内的内容  
以gazebo.sh为例，安装Chapter4内项目后：
```sh
cd ~/ros2_ws/ardupilot_ws;
source /opt/ros/jazzy/setup.bash;
source install/local_setup.bash; #找到ros_gz_sim_ardupilot
source ~/ros2_ws/ws/install/local_setup.bash; #找到ardupilot的gazebo插件
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}; #为gazebo找到插件添加全局变量
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/ardupilot_ws/src/ros_gz_sim_ardupilot/models:$HOME/ros2_ws/ardupilot_ws/src/ros_gz_sim_ardupilot/worlds:${GZ_SIM_SYSTEM_PLUGIN_PATH}; #指定gazebo的模型路径和地图路径
ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py; #脚本启动gz_sim_server， gz_sim_gui,和gazebo消息到ros2消息的桥接
```

运行script/中脚本以启动   
当前环境为ubuntu24.04+ros2jazzy，如有需要自行更改脚本以适配依赖
```sh
ardupilot.sh              #sitl
gazebo.sh                 #启动gazebo，发布图像
ros2_mavros.sh            #mavros
ros2_colcon.sh            #编译控制代码
ros2_offboard.sh          #启动控制代码
ros2_land.sh              #发布降落命令
# 如安装图象识别代码（见Chapter2实践部分）
image_sub_gazebo_small.sh #接收图像，发布yolo识别后位置
```

