TUNE_ID_t PosControl::get_autotuneID()
{
	TUNE_CFG_PARAM_t tuneCfgParam;
	tuneCfgParam.acterType = NEGATIVE_NATION;
	tuneCfgParam.ampStdDeviation = 0.1f;

	tuneCfgParam.cTrlType = CONTROLER_TYPE_PID;
	tuneCfgParam.cycleStdDeviation = 100.0f;
	tuneCfgParam.hysteresisNum = 0;
	tuneCfgParam.maxOutputStep = 0.1f;
	tuneCfgParam.minOutputStep = -0.1f;
	tuneCfgParam.setpoint = 0.0;
	// TUNE_ID_t tune_id;

	// tune_id = TUNE_New(&tuneCfgParam);
	return TUNE_New(&tuneCfgParam);
}

// 在一个固定时长进行循环的函数中调用函数TUNE_Work
float autotuneWORKCycle(float feedbackVal, TUNE_ID_t tune_id, bool &result, uint32_t delayMsec)
{

	// float feedbackVal;
	// uint32_t delayMsec = delayMsec;
	// std::chrono::nanoseconds delayDuration = std::chrono::milliseconds(delayMsec);
	// static std::chrono::nanoseconds delayDuration = std::chrono::milliseconds(100);
	static TUNE_STAT_t tuneStat = TUNE_INIT; /*整定状态*/

	// while(1)
	// {
	// rclcpp::sleep_for(delayDuration); // 循环间隔时间

	// feedbackVal = GetFeedBackVal();//获取实时反馈值
	if (tuneStat != TUNE_SUCESS || tuneStat != TUNE_FAIL)
	{
		result = false;
		float outputVal;
		tuneStat = TUNE_Work(tune_id, feedbackVal, &outputVal, delayMsec);
		// PWM_SetDuty(PWM_CH[k],outputVal);//输出输出值，控制响应单元执行
		return outputVal;
	}
	else
	{
		result = true;
		// PWM_SetDuty(PWM_CH[k],0.0f);
		// 此处已计算出PID值，将其更新到PID参数中
		float paramP, paramI, paramD;
		TUNE_GedPID(tune_id, &paramP, &paramI, &paramD);
		RCLCPP_INFO(node->get_logger(), "id:%d,paramP%f, paramI%f, paramD%f\n\n\n\n\n", tune_id, paramP, paramI, paramD);
		// PID_Release(tune_id);//释放PID资源
		return 0;
	}
	// }
}
// PID自整定 x/y/z/yaw:是否自整定对应的轴 tune_x/y/z/yaw默认为true
// delayMsec:自整定周期
// 返回值：是否自整定完成 false:未完成 true:完成
bool auto_tune(Vector4f pos_now, Vector4f pos_target, uint32_t delayMsec, bool tune_x, bool tune_y, bool tune_z, bool tune_yaw)
{
	static bool first = true, result_x = false, result_y = false, result_z = false, result_yaw = false;
	static TUNE_ID_t id_x, id_y, id_z, id_yaw;
	if (first)
	{
		result_x = !tune_x;
		result_y = !tune_y;
		result_z = !tune_z;
		result_yaw = !tune_yaw;
		PID::Defaults pid_defaults = {1.0, 0.0, 0.0};
		!result_x ? set_pid(pid_x, pid_defaults) : set_pid(pid_x, pid_x_defaults);
		!result_y ? set_pid(pid_y, pid_defaults) : set_pid(pid_y, pid_y_defaults);
		!result_z ? set_pid(pid_z, pid_defaults) : set_pid(pid_z, pid_z_defaults);
		!result_yaw ? set_pid(pid_yaw, pid_defaults) : set_pid(pid_yaw, pid_yaw_defaults);
		TUNE_Init();
		!result_x ? id_x = get_autotuneID() : id_x = 0;
		!result_y ? id_y = get_autotuneID() : id_y = 0;
		!result_z ? id_z = get_autotuneID() : id_z = 0;
		!result_yaw ? id_yaw = get_autotuneID() : id_yaw = 0;
		printf("id_x:%d,id_y:%d,id_z:%d,id_yaw:%d\n", id_x, id_y, id_z, id_yaw);
		// send_velocity_command_world((pos_target - pos_now));
		first = false;
	}
	Vector4f feedbackVal = input_pos_xyz_yaw_without_vel(pos_now, pos_target);
	Vector4f outputVel = {
		!result_x ? autotuneWORKCycle(feedbackVal.x, id_x, result_x, delayMsec) : feedbackVal.x,
		!result_y ? autotuneWORKCycle(feedbackVal.y, id_y, result_y, delayMsec) : feedbackVal.y,
		!result_z ? autotuneWORKCycle(feedbackVal.z, id_z, result_z, delayMsec) : feedbackVal.z,
		!result_yaw ? autotuneWORKCycle(feedbackVal.yaw, id_yaw, result_yaw, delayMsec) : feedbackVal.yaw};
	RCLCPP_INFO(node->get_logger(), "vx=%f,vy=%f,vz=%f,yaw=%f", outputVel.x, outputVel.y, outputVel.z, outputVel.yaw);
	send_velocity_command_world(outputVel);
	if (result_x && result_y && result_z && result_yaw)
	{
		reset_pid();
		TUNE_Release(id_x);
		TUNE_Release(id_y);
		TUNE_Release(id_z);
		TUNE_Release(id_yaw);
		result_x = false;
		result_y = false;
		result_z = false;
		result_yaw = false;
		first = true;
		return true;
	}
	return false;
}
