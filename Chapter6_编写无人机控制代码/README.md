## Chapter6_编写无人机控制代码 不考核
### 0. 写在前面
0. ##**务必先确认与章节文件夹并列的README文件中的考核范围，自查内容不检查**##

要求：
0. 构建自己的项目（推荐）或在已有项目上更改
1. 实现与飞控的消息交互（接收位置消息，发送解锁与起飞命令，发送速度与航点信息
2. 通过先前构建的pid代码在仿真中控制飞机
3. 验收目标是实现起飞->飞到目标点，如（10，0，0）->飞回起飞点上方->降落
    也可以尝试完成比赛要求（位于2025中国大学生飞行器设计创新大赛竞赛规则）的任务。

### 0.5. 了解多旋翼基本控制原理

首先对无人机系统进行分析。无人机通常被视为刚体运动,其运动状态可以通过六自由度方程进行描述。多旋翼无人机的飞行动力学模型具有强非线性特性。这代表着其运动行为无法通过简单的线性关系来描述，需要使用更为复杂的非线性方程来进行建模和分析。

使用商业自驾仪，可以利用开源飞控中建立的精确的多旋翼无人机动力学模型预测无人机动态响应，还可以将状态估计与传感器融合，集成多种传感器（如GPS、惯性测量单元IMU、视觉传感器等）进行实时状态估计，并运用卡尔曼滤波、粒子滤波等算法进行数据融合，从而直接获取无人机的精确位置、速度、姿态信息。

以下两本参考书籍可以不用读，仅作为提高了解多旋翼的基本原理用   
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

以下列举部分控制过程，你也可以思考自己的实现(可以不考虑旋转角)，或参考https://github.com/Hz1234567890/offboard的上机代码

接收数据：
接收目标位置主题为[InertialNav.h](../Chapter4_构建项目/ros2_ws/ardupilot_ws/src/px4_ros_com/src/examples/offboard_control/InertialNav.h)
```c++
pose_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(ardupilot_namespace+"local_position/pose", qos,
		std::bind(&InertialNav::pose_callback, this, std::placeholders::_1),sub_opt);

// 接收位置数据
void InertialNav::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
	position = (msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);//位置
	orientation = (msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);//四元数
}
```
发布数据：
发布目标速度主题为[OffboardControl.h](../Chapter4_构建项目/ros2_ws/ardupilot_ws/src/px4_ros_com/src/examples/offboard_control/PosControl.h)
```c++
ardupilot_namespace = "mavros"
twist_stamped_publisher_=node->create_publisher<geometry_msgs::msg::TwistStamped>(ardupilot_namespace+"setpoint_velocity/cmd_vel", 5);
// 发布速度控制指令
// linear_x, linear_y, linear_z 为速度(m/s), angular_z为角速度(度/s)
//
// ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
void PosControl::send_velocity_command_world(double linear_x, double linear_y, double linear_z, double angular_z)
{
	geometry_msgs::msg::TwistStamped msg;
	msg.twist.linear.x = linear_x;
	msg.twist.linear.y = linear_y;
	msg.twist.linear.z = linear_z;
	msg.twist.angular.z = angular_z;
	msg.header.stamp = node->now();
	msg.header.frame_id = "base_link";
	twist_stamped_publisher_->publish(msg);
}
```

pid控制，先自己尝试实现将C1完成的pid算法嵌入到send_velocity_command_world中的接口，看看控制效果  
```c++
// 输入位置 pid控制速度
Vector3f PosControl::input_pos_xyz(Vector3f now, Vector3f target)
{
		Vector3f f;
        //更改为你的pid代码函数
		f.x = pid_x.update_all(now.x, target.x, dt, max_speed_xy, InertialNav::velocity.x);
		f.y = pid_y.update_all(now.y, target.y, dt, max_speed_xy, InertialNav::velocity.y);
		f.z = pid_z.update_all(now.z, target.z, dt, max_speed_z, InertialNav::velocity.z);
		return f;
}

// pid飞行到指定位置（相对于起飞点/世界坐标系）
// x:前后方向位置(m) y:左右方向位置(m) z:高度(m) yaw:偏航角(°) accuracy=DEFAULT_ACCURACY:精度(m)
bool PosControl::trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, double accuracy, double yaw_accuracy)
{
	(void)yaw_accuracy;
	static bool first = true;
	static Vector4f pos_target_temp;
	static Vector4b direction;
	if (first)
	{
		_pos_target = pos_target;
		pos_target_temp = pos_target;
		RCLCPP_INFO(node->get_logger(), "trajectory_setpoint: x:%f y:%f z:%f", _pos_target.x, _pos_target.y, _pos_target.z);
		reset_pid();
		direction = {pos_now.x < _pos_target.x, pos_now.y < _pos_target.y, pos_now.z < _pos_target.z, pos_now.yaw < _pos_target.yaw};
		first = false;
	}
	if (is_equal(pos_target, pos_target_temp, 0.1f))
	{
		// send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target));
        // 发送速度，值为pid返回值
		send_velocity_command_world(input_pos_xyz_yaw(pos_now, _pos_target, true, direction));
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "change point");
		first = true;
	}
	if (abs(pos_now.x - _pos_target.x) <= accuracy &&
			abs(pos_now.y - _pos_target.y) <= accuracy &&
			abs(pos_now.z - _pos_target.z) <= accuracy
			// && abs(pos_now.yaw - _pos_target.yaw)<=yaw_accuracy
	)
	{
		RCLCPP_INFO(node->get_logger(), "at_check_point");
		first = true;
		return true;
	}
	return false;
}
```
main函数
```c++
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<OffboardControl>("/mavros/",yolo_,servocountroller_);
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
```
任务执行：

控制任务设计位于：  
\Chapter4_编写无人机控制代码\ros2_ws\ardupilot_ws\src\px4_ros_com\src\examples\offboard_control\OffboardControl.cpp

```c++
void OffboardControl::timer_callback(void)
{
    switch (fly_state_)
	{
	case FlyState::init:
		FlyState_init();
		break;
	case FlyState::takeoff:
		if (is_takeoff)
		{
			if (get_z_pos() > 2.5 + get_z_home_pos())
			{
				RCLCPP_INFO(this->get_logger(), "goto_shot_area start, totaltime=%fs", get_cur_time());
				fly_state_ = FlyState::goto_shot_area;
			}
			else
			{
				// 设定速度
				_pose_control->send_velocity_command_world(0, 0, 0.5, 0);
			}
		}
		else
		{
			// RCLCPP_INFO(this->get_logger(), "Takeoff failed");
		}
		break;
	case FlyState::goto_shot_area:
        if(trajectory_setpoint_world(30, 0, 5)) //x = 30,y=0,z=5,
    }
}
```
主要类的定义
```c++
class OffboardControl : public OffboardControl_Base
{
public:
	OffboardControl(const std::string ardupilot_namespace, std::shared_ptr<YOLO> yolo_, std::shared_ptr<ServoController> servo_controller_) : OffboardControl_Base(ardupilot_namespace),
    ······
    {
		InertialNav::position.x = DEFAULT_X_POS;
		Motors::home_position.x = DEFAULT_X_POS;
		// rclcpp::Rate rate(1s);
		while (!mode_switch_client_->wait_for_service(std::chrono::seconds(1)))
		{
			if (!rclcpp::ok() || is_equal(get_x_pos(), DEFAULT_X_POS))
			{
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;
	std::string ardupilot_namespace_copy_;
	std::shared_ptr<YOLO> _yolo;
	std::shared_ptr<ServoController> _servo_controller;
	std::shared_ptr<InertialNav> _inav;
	std::shared_ptr<Motors> _motors;
	std::shared_ptr<PosControl> _pose_control;

	enum class FlyState
	{
		init,
		// request,
		takeoff,
		goto_shot_area,
		findtarget,
		goto_scout_area,
		scout,
		land,
		end
	} fly_state_;
	class GlobalFrame
	{
	public:
		float lat;
		float lon;
		float alt;
	};
	float timestamp_init = 0;
	GlobalFrame start_global{0, 0, 0};
};
#endif // OFFBOARDCONTROL_H
```
具体实现：保存世界坐标系姿态（旋转角，四元数），通过当前坐标系命令控制飞机    

略  

### 启动程序

运行script中脚本
```sh
ardupilot.sh              #sitl
gazebo.sh                 #启动gazebo，发布图像
ros2_mavros.sh            #mavros
image_sub_gazebo_small.sh #接收图像，发布yolo识别后位置
#编译你的控制代码
#启动你的控制代码
```

### 3. pid调参

根据你先前的调参经验，尝试更改参数优化代码，引入更高级的控制算法或尝试自动调参。  
具体方案需要你自己探索。

要求gazebo中飞机尽可能稳定飞行

仿真与现实存在较大差距，调参经验在后续的真机飞行会常常用到

### 可选·gazebo的使用入门
ros_gz_sim_ardupilot基于ros2与gazebo官方通信示例改编（# ROS + Gazebo Sim demos）

地图为  
Chapter4_构建项目\ros2_ws\ardupilot_ws\src\ros_gz_sim_ardupilot\worlds\iris_runway.sdf  

飞机模型为  
Chapter4_构建项目\ros2_ws\ardupilot_ws\src\ros_gz_sim_ardupilot\worlds\iris_runway.sdf与Chapter4_构建项目\ros2_ws\ardupilot_ws\src\ros_gz_sim_ardupilot\models\iris_with_standoffs  

了解gazebo gui中的操作

尝试添加障碍物

看看能不能在已有的相机之上添加其他传感器并运用到控制代码中

尝试用rviz模拟

