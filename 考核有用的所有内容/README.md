# 从各个章节中摘录的考核内容

### 1. 任务1 · 搭建c/c++编译环境

- 要求，完成c/c++编译

可以选择在windows/ubuntu系统完成  
安装vscode，或终端使用cmake或g++，  
也可以选择codeblocks（易配置），clion等集成开发环境

下面列举部分使用说明

#### 1. 使用vscode编译
- 安装vscode，获取c编译器（如按照vscode官网推荐配置安装配置msys2）
- 安装 c/c++，code runner等插件
- pso.cpp 中1，2，4行添加注释
```c
1// extern "C" // C++中调用C语言函数
2// {
3#include "PID.h"
4// }
```
- 转到'运行与调试'，  
- windows使用‘C/C++: g++.exe 生成活动文件’运行（可调试），  
- linux使用‘C/C++: g++ 生成活动文件’运行（可调试）。

#### 2. 终端输入g++命令
cd 到项目目录
```sh
g++ -g src/* -o pso 
```
#### 3. 使用cmake编译

- pso.cpp 中1，2，4行去除注释
```c
extern "C" // C++中调用C语言函数

#include "PID.h"

```
终端输入
```sh
cd ~/.../pid_pso_test 
mkdir build
cd build
# make clean
cmake ..
make
```

编译完成文件为`build/pid_pso`, 在终端输入`./pid_pso`（运行当前目录的pid_pso程序）运行。

### 2. 任务2 · 了解项目基本架构

搜索资料了解C/C++项目基本架构及原因（本项目将源文件与头文件混合是为了vscode调试的便利）

向gpt询问C++项目的基本架构的回答：

一个典型的C/C++项目通常由多个文件和目录组成，每个文件和目录都有特定的用途。以下是一个基本的C++项目架构的示例，以及每个部分的解释：

#### 目录结构

```
MyCppProject/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── MyClass.h
│   └── Utils.h
├── src/
│   ├── main.cpp
│   ├── MyClass.cpp
│   └── Utils.cpp
├── tests/
│   ├── CMakeLists.txt
│   └── test_MyClass.cpp
└── build/
```

#### 目录和文件解释

1. **根目录**：
   - `CMakeLists.txt`：CMake构建系统的配置文件，定义了如何构建项目。
   - `README.md`：项目的说明文件，通常包含项目简介、安装和使用说明。
   - `build/`：构建目录，存放生成的二进制文件和中间文件。

2. **include/**：
   - 存放头文件（.h 或 .hpp），这些文件声明了类、函数和变量的接口。
   - `MyClass.h`：声明 `MyClass` 类。
   - `Utils.h`：声明一些实用函数。

3. **src/**：
   - 存放源文件（.cpp），这些文件实现了头文件中声明的类和函数。
   - `main.cpp`：项目的入口点，包含 `main` 函数。
   - `MyClass.cpp`：实现 `MyClass` 类。
   - `Utils.cpp`：实现实用函数。

4. **tests/**：
   - 存放测试代码，通常使用单元测试框架（如 Google Test）。
   - `CMakeLists.txt`：测试目录的CMake配置文件。
   - `test_MyClass.cpp`：测试 `MyClass` 类的单元测试。

通过这种结构，可以清晰地组织C++项目的代码和资源，便于开发、维护和扩展。

### 3. 任务3 · 编写pid程序
#### 基本要求
- 稳定性， 快速性， 准确性。  
![自动控制原理-基本要求1](image-1.png)  
![自动控制原理-基本要求2](image-2.png)  
#### 动态性能
![自动控制原理-动态性能](image.png)  
#### 稳态性能
若时间趋于无穷时，系统的输出量不等于输入量或输入量的确定函数，则系统存在稳态误差。  
稳态误差是系统精度或抗扰动能力的一种度量。  


#### 尝试编写pid程序，并根据以上要求调整程序和参数


编写过程省略


标准实现见参考资料或上网搜索/AI

CMakeLists.txt文件示例：
```cmake
cmake_minimum_required(VERSION 3.10)
# 设置C和C++编译器路径
# set(CMAKE_C_COMPILER "C:/msys64/ucrt64/bin/gcc.exe")
# set(CMAKE_CXX_COMPILER "C:/msys64/ucrt64/bin/g++.exe")

project(pid_pso VERSION 1.0 LANGUAGES C CXX)

# 设置C标准和C++标准
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置包含目录
include_directories(${PROJECT_SOURCE_DIR}/src)
# include_directories(${PROJECT_SOURCE_DIR}/external/some_external_library/include)

# 设置源文件
set(SOURCES
    src/PID.c
    src/FuzzyPID.cpp
    src/pso.cpp
)

# 添加可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接外部库
# target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/external/some_external_library/lib/libexternal.a)
```



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

### 1. 项目概述
当前项目架构：  
无人机使用Ardupilot飞控，通过串口连接到树莓派。 

控制部分：树莓派中运行控制代码，发布ros2消息，通过mavros桥接转换为mavlink协议通过串口与飞控通信实现实行控制。    

图像识别部分：树莓派读取树莓派摄像头的数据，发布ros2消息到以太网，地面站（你的计算机）接收ros2消息，通过yolo识别出对象的宽高与位置，再通过ros2发布由树莓派上的控制程序接收，转化为飞机的目标姿态。  
```
飞机{{飞控}-串口连接-{机载计算机(树莓派)}}-以太网（ros2）-地面站{图传}  
                |missionplanner    |-ros2-摄像头
```
仿真项目架构：  
飞控改为用ardupolot-sitl模拟，安装mavros桥接，控制代码接收与发布ros2消息，  
通过gazebo观察飞机状态，输出摄像头数据。  
```
地面站{mavlink{ardupolot-sitl}-mavros-ros2{控制代码}}-ros2{图传代码}  
                                                     | gazebo获得图像数据-ros2
```
总结：
- 项目概述
![项目概述](项目概述.png)
- 仿真概述
![仿真概述](仿真概述.png)


### 3. 启动仿真
#### 使用 SITL
sim_vehicle.py
提供了一个启动脚本，用于自动为当前代码分支构建 SITL 固件版本、加载仿真模型、启动仿真器、设置环境和车辆参数，以及启动 MAVProxy GCS。可以指定许多脚本启动参数，键入此参数可查看完整列表：
`sim_vehicle.py --help`(先输入`export PATH=$PATH:$HOME/ardupilot/Tools/autotest`或将该命令写入~/.bashrc自动运行)
```
#用到的列表
  -v VEHICLE, --vehicle=VEHICLE
                        vehicle type (飞行器类型选择ArduCopter四旋翼)
  -f FRAME, --frame=FRAME
                        set vehicle frame type
                        ArduCopter: gazebo-
                            iris
                        （选择机架类型：gazebo-iris）
Simulation options:
    --model=MODEL       Override simulation model to use（新开终端，与gazebo通信）
Compatibility MAVProxy options (consider using --mavproxy-args instead):
  --map               load map module on startup（新开一个map窗口显示地图）
  --console           load console module on startup （新开一个console窗口显示飞机状态）
```
```sh
#标准启动命令
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```
启动完后应新开三个窗口，如图所示,左下角为原来的窗口，ubuntu系统中新开的终端与原来的终端样式一致
![SITL](SITL.png)

#### gazebo与APM SITL综合使用
- 启动gazebo
    ```bash
    gz -h

    sim:           Run and manage the Gazebo Simulator.

    gz sim -h

    -r                           Run simulation on start.
    -v [ --verbose ] [arg]       Adjust the level of console output (0~4).控制台输出的输出等级
                                The default verbosity is 1, use -v without

                                arguments for level 3.

    ```
- 构建插件省略
- 配置 Gazebo 环境
    Gazebo 使用许多环境变量来查找插件和模型 在运行时。这些可以在用于运行 Gazebo 的终端中设置，也可以设置 在 .bashrc 或 .zshrc 文件中：

    - 在终端中
        ```sh
        export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
        export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
        ```
    - 在 .bashrc 或 .zshrc 中
        ```sh
        echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
        echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gaz
        ```
综合启动

Iris quadcopter
Run Gazebo
```sh
gz sim -v4 -r iris_runway.sdf
```
Run SITL
```sh
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```
给飞控发送模式切换，解锁和起飞距离起飞点5米高度命令  
Arm and takeoff
```
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

完成起飞操作后的界面
![gz](gz.png)

### 0.5. 了解多旋翼基本控制原理

首先对无人机系统进行分析。无人机通常被视为刚体运动,其运动状态可以通过六自由度方程进行描述。多旋翼无人机的飞行动力学模型具有强非线性特性。这代表着其运动行为无法通过简单的线性关系来描述，需要使用更为复杂的非线性方程来进行建模和分析。

使用商业自驾仪，可以利用开源飞控中建立的精确的多旋翼无人机动力学模型预测无人机动态响应，还可以将状态估计与传感器融合，集成多种传感器（如GPS、惯性测量单元IMU、视觉传感器等）进行实时状态估计，并运用卡尔曼滤波、粒子滤波等算法进行数据融合，从而直接获取无人机的精确位置、速度、姿态信息。


### 1. 学习接收mavros转换后的ros2消息

可以搜索mavros使用方法，也可以从示例代码中查找（InertialNav，Motors，PosControl）,或者从mavros源码/github查找 
```sh
ros2 topic list  
ros2 topic echo /mavros/···
```

### 2. 控制程序的结构

新建ros2工作区：
```txt
-ros2_ws 项目名称
|-install
|-
|-
|-src
    |-你的包名
        |-CMakeLists.txt
        |-packages.xml
        |-src
            |-你的源文件
    |-其他包
```
### 3. 编写起飞程序

了解起飞流程，见C3：gazebo与APM SITL综合使用  

main编写如ros2学习内容

```cpp

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl(); // 构造函数
    // 起飞函数s
    void takeoff(int halt);
    // 设置模式函数
    void set_mode(const std::string &mymode);
    // 油门锁解锁函数
    void arm();
private:
    // 节点声明
    //rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
    //rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_subscription_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_motors_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
    rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback(void);
}

OffboardControl::OffboardControl()
{
   RCLCPP_INFO(node->get_logger(), "Starting Motors Example"); // 输出日志
   // 节点初始化

   // 解锁电机服务
   arm_motors_client_ = node->create_client<mavros_msgs::srv::CommandBool>(ardupilot_namespace + "cmd/arming");
   // 模式切换服务
   mode_switch_client_ = node->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace + "set_mode");
   // 家的位置服务
   set_home_client_ = node->create_client<mavros_msgs::srv::CommandHome>(ardupilot_namespace + "cmd/set_home");
   // 状态订阅
   // * /mavros/state [mavros_msgs/msg/State] 1 subscriber
   //state_subscription_ = node->create_subscription<mavros_msgs::msg::State>("/mavros/state",
   //																	 std::bind(&Motors::state_callback, this, std::placeholders::_1));
   // 家的位置订阅
   // * /mavros/home_position/home [mavros_msgs/msg/HomePosition] 1 subscriber
   //home_position_subscription_ = node->create_subscription<mavros_msgs::msg::HomePosition>("mavros/home_position/home",
    //																				std::bind(&Motors::home_position_callback, this, std::placeholders::_1));
    // 节点初始化结束
    // 创建一个以100ms为周期的循环，周期执行timer_callback函数
    // include <chrono>; using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
}
```
- 同步与异步，阻塞和非阻塞概念
自己网上找教程https://blog.csdn.net/wangpaiblog/article/details/117236684

```cpp
// 异步阻塞式代码编写
//void OffboardControl::timer_callback(){
构造函数中直接执行	init();
//}

// 初始化运动节点
void OffboardControl::init()
{
    RCLCPP_INFO(this->get_logger(), "获取PID参数");

    RCLCPP_INFO(this->get_logger(), "开始初始化舵机");
    // 初始化舵机操作，更改为接收到状态/有效位置继续执行
    while (!servo_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the Servo service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Servo Service not available, waiting again...");
    }

    // servo_controller(12, 1050);
    RCLCPP_INFO(this->get_logger(), "结束初始化舵机");
    // 创建一个Rate对象，设置为每秒20次循环
    rclcpp::Rate rate(10);

    // 重新设置家地址
    set_home_position();

    RCLCPP_INFO(this->get_logger(), "Initializing...");
    // 设置无人机模式为GUIDED
    set_mode("GUIDED");

    std::string key;
    while (true)
    {
        RCLCPP_INFO(this->get_logger(), "解锁前所有准备已完成，按下回车解锁无人机");
        // 读取一整行输入
        std::getline(std::cin, key);

        // 检查输入是否为空（即用户只按下了回车键）
        if (key.empty())
        {
            RCLCPP_INFO(this->get_logger(), "开始解锁");
            break; // 跳出循环
        }
    }

    arm();
}

// 油门锁解锁函数
void OffboardControl::arm()
{
    // 解锁无人机
    auto arming_cl = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto srv_arm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    srv_arm->value = true;

    while (!arming_cl->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "Arm Command send");
    auto arm_result = arming_cl->async_send_request(srv_arm);
    rclcpp::sleep_for(std::chrono::seconds(3));
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Arm success");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Arm Failed");
    }
}

void OffboardControl::takeoff(int halt){
    // 发布起飞的指令，向上飞行10米
    RCLCPP_INFO(this->get_logger(), "开始发送takeoff指令");
    auto takeoff_cl = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    auto srv_takeoff = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    srv_takeoff->altitude = halt;
    RCLCPP_INFO(this->get_logger(), "Takeoff command send");

    auto takeoff_result = takeoff_cl->async_send_request(srv_takeoff);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), takeoff_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Takeoff send ok");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed Takeoff");
    }
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Takeoff timedelay end"); 
}

// setMode
void OffboardControl::set_mode(const std::string &mymode)
{
    auto cl = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    auto srv_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    srv_set_mode->base_mode = 0;
    srv_set_mode->custom_mode = mymode;

    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "SetMode Command send");
    auto result = cl->async_send_request(srv_set_mode);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(get_logger(), mymode.c_str(), "mode set successfully");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to set", mymode.c_str(), " mode");
    }
}

```
```cpp
// 异步非阻塞：使用状态机
OffboardControl::create_wall_timer(){
    takeoff(local_frame_z,5);
}

// 起飞 local_frame_z: 当前高度(m) takeoff_altitude=5 起飞高度(m)
// 1. 设置家的位置
// 2. 进入GUIDED模式
// 3. 等待稳定的offboard模式
// 4. 请求解锁
// 5. 起飞
// 6. 若未解锁则回到第3步
bool Motors::takeoff(float local_frame_z,float takeoff_altitude){
    static bool is_takeoff = false;
    static uint8_t num_of_steps = 0;
    switch (state_)
    {
    case State::init :
        set_home_position();
        state_ = State::send_geo_grigin;
        break;
    case State::send_geo_grigin :
        RCLCPP_INFO(node->get_logger(), "Entered guided mode");
        switch_mode("GUIDED");
        state_ = State::wait_for_stable_offboard_mode;				
        
        break;
    case State::wait_for_stable_offboard_mode :
        if (++num_of_steps>10){
            rclcpp::sleep_for(50ms);
            arm_motors(true);
            state_ = State::arm_requested;
        }
        break;
    case State::arm_requested :
        if(!armed){//_arm_done
            rclcpp::sleep_for(200ms);
            arm_motors(true);
            command_takeoff_or_land("TAKEOFF");
        }
        else{
            //RCLCPP_INFO(this->get_logger(), "vehicle is armed");
            state_ = State::takeoff;
        }
        break;
    case State::takeoff:
        //RCLCPP_INFO(this->get_logger(), "vehicle is start");
        //arm_motors(true);			
        if(local_frame_z - home_position.z < 2){
            command_takeoff_or_land("TAKEOFF",takeoff_altitude);
            rclcpp::sleep_for(300ms);
            
        }else{
            is_takeoff = true;
            //RCLCPP_INFO(this->get_logger(), "takeoff done");
            state_ = State::autotune_mode;
        }
        break;
    case State::autotune_mode:
        if(!armed){
            is_takeoff = false;
            num_of_steps = 0;
            RCLCPP_INFO(node->get_logger(), "vehicle is not armed");
            state_ = State::wait_for_stable_offboard_mode;
        }
        break;
    default:
        break;
    }
    return is_takeoff;
}
```