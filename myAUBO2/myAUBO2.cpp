// myAUBO2.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include"rsdef.h"

#define M_PI 3.1415926535
#define ROBOT_ADDR "169.254.95.142"
#define ROBOT_PORT 8899

RSHD g_rshd = -1;

//该位置为机械臂的初始位置（提供6个关节角的关节信息（单位：弧度））
double initPos[6] = {
	-0.000172 / 180 * M_PI,
	-7.291862 / 180 * M_PI,
	-75.694718 / 180 * M_PI,
	21.596727 / 180 * M_PI,
	-89.999982 / 180 * M_PI,
	-0.00458 / 180 * M_PI };


bool my_login(RSHD &rshd, const char *addr, int port)
{
	bool result = false;
	rshd = RS_FAILED;
	//初始化接口库
	if (rs_initialize() == RS_SUCC)
	{
		//创建上下文
		if (rs_create_context(&rshd) == RS_SUCC)
		{
			//登陆机械臂服务器
			if (rs_login(rshd, addr, port) == RS_SUCC)
			{
				result = true;
				//登陆成功
				std::cout << "login succ" << std::endl;
			}
			else
			{
				//登陆失败
				std::cerr << "login failed" << std::endl;
			}
		}
		else
		{
			//创建上下文失败
			std::cerr << "rs_create_context error" << std::endl;
		}
	}
	else
	{
		//初始化接口库失败
		std::cerr << "rs_initialize error" << std::endl;
	}
	return result;
}

bool my_robotStartup(RSHD rshd)
{
	bool result = false;

	//工具的动力学参数和运动学参数
	ToolDynamicsParam tool_dynamics = { 0 };
	//机械臂碰撞等级
	uint8 colli_class = 6;
	//机械臂启动是否读取姿态（默认开启）
	bool read_pos = true;
	//机械臂静态碰撞检测（默认开启）
	bool static_colli_detect = true;
	//机械臂最大加速度（系统自动控制，默认为30000)
	int board_maxacc = 30000;
	//机械臂服务启动状态
	ROBOT_SERVICE_STATE state = ROBOT_SERVICE_READY;

	if (rs_robot_startup(rshd, &tool_dynamics, colli_class, read_pos, static_colli_detect, board_maxacc, &state)
		== RS_SUCC)
	{
		result = true;
		std::cout << "call robot startup succ, robot state:" << state << std::endl;
	}
	else
	{
		std::cerr << "robot startup failed" << std::endl;
	}

	return result;
}

bool my_moveJ(RSHD rshd)
{
	bool result = false;

	RobotRecongnitionParam param;
	rs_get_robot_recognition_param(rshd, 1, &param);

	//该位置为机械臂的初始位置（提供6个关节角的关节信息（单位：弧度））
	/*
	double initPos[6] = {
		-0.000172 / 180 * M_PI,
		-7.291862 / 180 * M_PI,
		-75.694718 / 180 * M_PI,
		21.596727 / 180 * M_PI,
		-89.999982 / 180 * M_PI,
		-0.00458 / 180 * M_PI };
	*/

	//首先运动到初始位置
	if (rs_move_joint(rshd, initPos) == RS_SUCC)
	{
		result = true;
		std::cout << "movej succ" << std::endl;
	}
	else
	{
		std::cerr << "movej failed!" << std::endl;
	}

	return result;
}

bool my_moveJ2(RSHD rshd,wayPoint_S wp4moveJ)
{
	bool result = false;

	RobotRecongnitionParam param;
	rs_get_robot_recognition_param(rshd, 1, &param);

	//运动到位置
	if (rs_move_joint(rshd, wp4moveJ.jointpos) == RS_SUCC)
	{
		result = true;
		std::cout << "************** ik and movej succ*****************" << std::endl;
	}
	else
	{
		std::cerr << "movej failed!" << std::endl;
	}

	return result;
}

bool to_zero(RSHD rshd)
{
	bool result = false;
	double zero_pos[6] = { 0,0,0,0,0,0 };
	if (rs_move_joint(rshd, zero_pos) == RS_SUCC)
	{
		result = true;
		std::cout << "to_zero movej succ" << std::endl;
	}
	else
	{
		result = false;
		std::cerr << "to_zero movej failed!" << std::endl;
	}

	return result;
}

bool my_ik(RSHD rshd) 
{
	//开始点设置
	double startPointJointAngle[aubo_robot_namespace::ARM_DOF] = { 0.0 / 180.0*M_PI,  0.0 / 180.0*M_PI,  0.0 / 180.0*M_PI, 0.0 / 180.0*M_PI, 0.0 / 180.0*M_PI,0.0 / 180.0*M_PI };

	//目标位置xyz设置
	aubo_robot_namespace::wayPoint_S wpNow;
	rs_get_current_waypoint(rshd,&wpNow);
	std::cout << "当前的路点信息的xyz坐标是："<<std::endl;
	std::cout<<"x:"<<wpNow.cartPos.position.x
			<<" y:"<<wpNow.cartPos.position.y
			<<" z:"<<wpNow.cartPos.position.z
			<<std::endl;
	std::cout << "请输入路点信息的xyz坐标" << std::endl;
	//targetPosition.x = 0.5;
	//targetPosition.y = 0.5;
	//targetPosition.z = 0.5;
	aubo_robot_namespace::Pos targetPosition;
		std::cin >>targetPosition.x;
		std::cin >>targetPosition.y;
		std::cin >>targetPosition.z;
	
	std::cout << "目标路点xyz信息是" << targetPosition.x<<" "<<targetPosition.y << " " << targetPosition.z << " " << std::endl;

	//目标姿态，先有欧拉角，再转换四元数
	aubo_robot_namespace::Rpy rpy;
	aubo_robot_namespace::Rpy temp_rpy;
	aubo_robot_namespace::Ori targetOri;

	std::cout << "请输入目标路点信息的姿态信息，rx，ry，rz" << std::endl;
	std::cin >> temp_rpy.rx
		>> temp_rpy.ry
		>> temp_rpy.rz;
	std::cout << "你输入目标路点信息的姿态信息(角度)：\n rx:"<<temp_rpy.rx <<" ry:"<<temp_rpy.ry<<" rz:"<<temp_rpy.rz <<std::endl;
	rpy.rx = temp_rpy.rx / 180.0*M_PI;
	rpy.ry = temp_rpy.ry / 180.0*M_PI;
	rpy.rz = temp_rpy.rz / 180.0*M_PI;
	rs_rpy_to_quaternion(rshd, &rpy, &targetOri);

	//目标路点
	aubo_robot_namespace::wayPoint_S wayPoint;

	for(int i = 0;i<ARM_DOF;i++)
	{
		startPointJointAngle[i] = wpNow.jointpos[i];
	}

	bool result = false;
	if (RS_SUCC == rs_inverse_kin(rshd, startPointJointAngle, &targetPosition, &targetOri, &wayPoint))
	{
		std::cout << "ik succ" << std::endl;
		result = true;
		//printRoadPoint(&wayPoint);
		for (int i = 0; i < ARM_DOF; i++)
		{
			std::cout <<"第"<<i+1<<"个关节角是:"<< wayPoint.jointpos[i]<< std::endl;
		}
		std::cout << "运动开始" << std::endl;
		my_moveJ2(rshd,wayPoint);
	}
	else
	{
		std::cerr << "ik failed" << std::endl;
		result = false;
	}
	return result;
}

int main()
{
	if (my_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
	{
		//my_robotStartup(g_rshd);

		//my_moveJ(g_rshd);
		
		if(my_ik(g_rshd)==RS_SUCC)
		{
			std::cout << "执行完成" << std::endl;
		}
		

		//my_moveL(g_rshd);
		//to_zero(g_rshd);
	}


    std::cout << "Hello World!\n"; 
}


