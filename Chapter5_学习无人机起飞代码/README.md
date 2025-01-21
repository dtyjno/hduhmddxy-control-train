## Chapter4.5_学习无人机控制代码
### 0. 写在前面
0. ##**务必先确认与章节文件夹并列的README文件中的考核范围，自查内容不检查**##

### 0.5. 了解多旋翼基本控制原理

首先对无人机系统进行分析。无人机通常被视为刚体运动,其运动状态可以通过六自由度方程进行描述。多旋翼无人机的飞行动力学模型具有强非线性特性。这代表着其运动行为无法通过简单的线性关系来描述，需要使用更为复杂的非线性方程来进行建模和分析。

使用商业自驾仪，可以利用开源飞控中建立的精确的多旋翼无人机动力学模型预测无人机动态响应，还可以将状态估计与传感器融合，集成多种传感器（如GPS、惯性测量单元IMU、视觉传感器等）进行实时状态估计，并运用卡尔曼滤波、粒子滤波等算法进行数据融合，从而直接获取无人机的精确位置、速度、姿态信息。

以下两本参考书籍可以不用读，仅作为提高了解多旋翼的基本原理用，不考核   
[多旋翼飞行器设计与控制_全权](多旋翼飞行器设计与控制_全权.pdf)  
[多旋翼飞行器设计与控制实践_全权](多旋翼飞行器设计与控制实践_全权.pdf)   

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
- ros2多线程 不考核
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
### 可选·了解gazebo
ros_gz_sim_ardupilot基于ros2与gazebo官方通信示例改编（# ROS + Gazebo Sim demos）

地图为  
Chapter4_构建项目\ros2_ws\ardupilot_ws\src\ros_gz_sim_ardupilot\worlds\iris_runway.sdf  

飞机模型为  
Chapter4_构建项目\ros2_ws\ardupilot_ws\src\ros_gz_sim_ardupilot\worlds\iris_runway.sdf与Chapter4_构建项目\ros2_ws\ardupilot_ws\src\ros_gz_sim_ardupilot\models\iris_with_standoffs  

了解gazebo gui中的操作

尝试添加障碍物

看看能不能在已有的相机之上添加其他传感器并运用到控制代码中

尝试用rviz模拟

