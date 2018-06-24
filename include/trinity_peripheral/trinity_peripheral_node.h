#ifndef TRINITY_PERIPHERAL_H
#define TRINITY_PERIPHERAL_H

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "trinity_peripheral/type.h"
#include "trinity_peripheral/uart.h"

#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

//ROS
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Range.h>

//ROS Custom

//公共命令相关
#include "trinity_platform_msgs/SingleParamSetting.h"



namespace trinity_peripheral{

class TrinityPeripheral
{
public:
	TrinityPeripheral(const ros::NodeHandle &nh);
	~TrinityPeripheral();

	static void onMessageProcess(const char* buf, const int len);
	static bool cksCheck(const char* inbuffer,const int length);
	static void uartRec(const void *msg, unsigned int msglen, void *user_data);
	
protected:
	void onBatteryCb(const std_msgs::UInt16::ConstPtr& msg);
	void onSingleParamSettingCb(const trinity_platform_msgs::SingleParamSetting::ConstPtr &msg);

public:
	static UART_HANDLE m_uartHd;

private:
	ros::NodeHandle m_nh;
	ros::Subscriber m_subBattery,m_subSingleParamSetting;

	static ros::Publisher m_pubUltrasonic;
	
	static std::vector<char> m_bigbuf;        //存放接收到的数据的容器
	static int m_recvIndex;                   //接收到的数据的个数
	static int m_packageLength;

	static boost::mutex m_ttsStatusMutex;
	ros::Timer m_cmdvel_timer;

};

}
#endif//TRINITY_PERIPHERAL_H
