#include"myAUBO2_function.h"
#include "rsdef.h"
#define M_PI 3.14159265235

#define ROBOT_ADDR "192.168.74.129"
#define ROBOT_PORT 8899

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