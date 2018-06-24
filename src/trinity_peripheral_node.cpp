#include "trinity_peripheral/trinity_peripheral_node.h"
#include "trinity_peripheral/tool.h"
#include "trinity_peripheral/type.h"
#include <std_msgs/UInt16.h>
#include "trinity_platform_msgs/TrinityUltrasonics.h"

#include <boost/shared_ptr.hpp>
#include <boost/smart_ptr.hpp>
#include <iostream>

using namespace trinity_peripheral;

UART_HANDLE TrinityPeripheral::m_uartHd;

boost::mutex TrinityPeripheral::m_ttsStatusMutex;

std::vector<char> TrinityPeripheral::m_bigbuf;

int TrinityPeripheral::m_recvIndex = 0;

int TrinityPeripheral::m_packageLength = 0;

ros::Publisher TrinityPeripheral::m_pubUltrasonic;


TrinityPeripheral::TrinityPeripheral(const ros::NodeHandle &nh)
	:m_nh(nh)
{
	//Subscriber
	m_subBattery = m_nh.subscribe<std_msgs::UInt16>("battery_volume",1,&TrinityPeripheral::onBatteryCb,this);


	//Publisher
	m_pubUltrasonic = m_nh.advertise<trinity_platform_msgs::TrinityUltrasonics>("trinityUltrasonics",1);	

	m_bigbuf.clear();

}


TrinityPeripheral::~TrinityPeripheral()
{

}



void TrinityPeripheral::onMessageProcess(const char* buf, const int len)
{
	if(!cksCheck(buf,len))
		return;

	//解析数据
	unsigned char command_type = buf[3];
	//ROS_INFO("command_type: %d",command_type);
	switch(command_type)
	{
		case 0x20: //Ultrasonic data
		{
			trinity_platform_msgs::TrinityUltrasonics ultrasonics;
			sensor_msgs::Range range;
			for(int i=0;i<4;i++)
			{
				range.header.stamp = ros::Time::now();
				std::stringstream ss;
				ss<<"ultrasonic"<<i;
				range.header.frame_id = ss.str();
				range.min_range = 0.05;
				range.max_range = 4.0;

				range.range = static_cast<unsigned char>(buf[6+i*2]) | static_cast<unsigned char>(buf[7+i*2]) << 8 ;
				ultrasonics.ultrasonics.push_back(range);			
			}
			m_pubUltrasonic.publish(ultrasonics);

			break;
		}
		default:
			break;
	}
}



bool TrinityPeripheral::cksCheck(const char* buffer,const int length)
{
	uint16_t crc = bytes2int16(buffer+length-3);
	if(crc == getCks((const uint8_t*)buffer,length-3))
	{	
		return true;
	}
	else
	{	
		return false;
	}
}



void TrinityPeripheral::uartRec(const void *msg, unsigned int msglen, void *user_data)
{
	if(0 == m_bigbuf.size())
	{
		for(int i=0;i<msglen;i++)
		{
			//ROS_INFO("m_recvIndex: %d",m_recvIndex);
			if(((unsigned char*)msg)[i] == SYNC_FLAG_START)
			{
				//ROS_INFO("Recv header.");
				m_bigbuf.push_back(((unsigned char*)msg)[i]);
				m_recvIndex += 1;
			}
			else
			{
				if(m_recvIndex > 0)
				{
					//已经接收到数据头，开始将数据添加到容器中
					m_bigbuf.push_back(((unsigned char*)msg)[i]);
					m_recvIndex += 1;

					/*if(5 == m_recvIndex)
					{
						m_packageLength = ((unsigned char*)msg)[i];
					}*/
					if(m_recvIndex >= 6 && !m_packageLength)
					{
						m_packageLength = static_cast<unsigned char>(m_bigbuf[4]) | static_cast<unsigned char>(m_bigbuf[5]) << 8;
						//ROS_INFO("package length: %d",m_packageLength);

						if(m_packageLength > MAX_FRAME_LENGTH)
						{
							m_recvIndex = 0;
							m_packageLength = 0;
							m_bigbuf.clear();
							continue;
						}
					}
					if(m_packageLength+9 == m_recvIndex)
					{
						//读到一帧完整的数据
						onMessageProcess(m_bigbuf.data(),m_bigbuf.size());

						//标识符还原
						m_recvIndex = 0;
						m_packageLength = 0;
						m_bigbuf.clear();
					}
				}
				else
				{
					//没有接收到数据头
					//ROS_INFO("Drop this byte: %x",((unsigned char*)msg)[i]);
				}
			}
		}
	}
	else
	{
		for(int i=0;i<msglen;i++)
		{
			m_bigbuf.push_back(((unsigned char*)msg)[i]);
			m_recvIndex += 1;

			/*if(5 == m_recvIndex)
			{
				m_packageLength = ((unsigned char*)msg)[i];
			}*/
			if(m_recvIndex >=6 && !m_packageLength)
			{
				m_packageLength = static_cast<unsigned char>(m_bigbuf[4]) | static_cast<unsigned char>(m_bigbuf[5]) << 8;
				//ROS_INFO("package length: %d",m_packageLength);

				if(m_packageLength > MAX_FRAME_LENGTH)
				{
					m_recvIndex = 0;
					m_packageLength = 0;
					m_bigbuf.clear();
					continue;
				}
			}
			
			if(m_packageLength+9 == m_recvIndex)
			{
				//读到一帧完整的数据
				onMessageProcess(m_bigbuf.data(),m_bigbuf.size());

				//标识符还原
				m_recvIndex = 0;
				m_packageLength = 0;
				m_bigbuf.clear();
			}
		}
	}
}





/*设置单个参数设置*/
void TrinityPeripheral::onSingleParamSettingCb(const trinity_platform_msgs::SingleParamSetting::ConstPtr &msg)
{
	/*boost::shared_array<char> paramBuffer(new char[13]);

	//Header
	paramBuffer[0] = SYNC_FLAG_START;
 	paramBuffer[1] = MOTION_DEVICE_TYPE;
	paramBuffer[2] = MOTION_DEVICE_ADDRESS;

	//Command
	paramBuffer[3] = 0x1E;

	//Length
	paramBuffer[4] = 0x05;

	//Data
	paramBuffer[5] = msg->index;
	char* paramData = hangfa_platform::int322bytes(msg->data);
	memcpy(paramBuffer.get()+6,paramData,4);

	//CRC check
	uint16_t crcValue = getCks((uint8_t*)(paramBuffer.get()),10);
	char* crc = hangfa_platform::int162bytes(crcValue);
	memcpy(paramBuffer.get()+10,crc,2);

	paramBuffer[12] = SYNC_FLAG_END;

	boost::mutex::scoped_lock lock(m_ttsStatusMutex);
	uart_send(m_uartHd,paramBuffer.get(),13);*/
}



void TrinityPeripheral::onBatteryCb(const std_msgs::UInt16::ConstPtr& msg)
{
	ROS_INFO("Send battery volume to motion board: %d",msg->data);
	boost::shared_array<char> paramBuffer(new char[11]);

        //Header
        paramBuffer[0] = SYNC_FLAG_START;
        paramBuffer[1] = DEVICE_TYPE;
        paramBuffer[2] = DEVICE_ADDRESS;

        //Command
        paramBuffer[3] = 0x21;

        //Length
        paramBuffer[4] = 0x02;
	paramBuffer[5] = 0x00;

        //Data
        char* paramData = int162bytes(msg->data);
        memcpy(paramBuffer.get()+6,paramData,2);

        //CRC check
        uint16_t crcValue = getCks((uint8_t*)(paramBuffer.get()),8);
        char* crc = int162bytes(crcValue);
	memcpy(paramBuffer.get()+8,crc,2);

        paramBuffer[10] = SYNC_FLAG_END;

        boost::mutex::scoped_lock lock(m_ttsStatusMutex);
        uart_send(m_uartHd,paramBuffer.get(),11);
}
