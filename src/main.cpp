#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <unistd.h>

#include <signal.h>

#include "trinity_peripheral/trinity_peripheral_node.h"
#include "trinity_peripheral/type.h"

//ros
#include <ros/ros.h>


using namespace trinity_peripheral;


void onRosShutdown(int sig)
{
	//close uart first
	uart_uninit(&TrinityPeripheral::m_uartHd);

	//close ros node
	ros::shutdown();
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"trinity_peripheral_node",ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	//設置串口的波特率
	std::string trinity_uart_port = std::string("");
	int trinity_uart_baudrate = 0;
	ros::param::get("~trinity_uart_port",trinity_uart_port);
	ros::param::get("~trinity_uart_baudrate",trinity_uart_baudrate);
	ROS_INFO("trinity_uart_port: %s, trinity_uart_baudrate: %d",trinity_uart_port.c_str(),trinity_uart_baudrate);


	TrinityPeripheral trinityPeripheral(nh);
	int ret = 0;
	ret = uart_init(&TrinityPeripheral::m_uartHd, trinity_uart_port.c_str(),trinity_uart_baudrate,&TrinityPeripheral::uartRec, NULL);
	if (0 != ret)
	{
		printf("uart_init error ret = %d\n", ret);
	}

	signal(SIGINT,onRosShutdown);
	ros::spin();
	return 0;

}
