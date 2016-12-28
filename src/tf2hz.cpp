#include <signal.h>
#include <ctime>
#include <iostream>
#include <pthread.h>

#include "tf2hz.h"
#include "monotonic_time.h"

#define BUF_SIZE	128

using namespace std;

Transform2Hz::Transform2Hz(int64_t ms) : m_i64LaserScanCount(0), m_i64ImuCount(0),	m_i64WheelFeedbackCount(0)
{
	m_i64Interval		= 	ms;
}

bool Transform2Hz::Initial()
{
	m_subLaserScan 		=	m_nhMyNodeHandle.subscribe<sensor_msgs::LaserScan, Transform2Hz>("/rplidar_scan", BUF_SIZE, &Transform2Hz::LaserScanCallback, this);
	m_subImu			= 	m_nhMyNodeHandle.subscribe<sensor_msgs::Imu, Transform2Hz>("/imu_data", BUF_SIZE, &Transform2Hz::ImuCallback, this);
	m_subWheelFeedback	=	m_nhMyNodeHandle.subscribe<andbot::WheelFb, Transform2Hz>("/feedback_wheel_angularVel", BUF_SIZE, &Transform2Hz::WheelFeedbackCallback, this);
	
	m_pubLaserScanHz	=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/laserscan_hz", BUF_SIZE);
	m_pubImuHz			=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/imu_hz", BUF_SIZE);
	m_pubWheelFeedbackHz=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/wheelfb_hz", BUF_SIZE);
	
	return true;
}

void Transform2Hz::LaserScanCallback(sensor_msgs::LaserScan laser_scan)
{
	m_i64LaserScanCount++;
}

void Transform2Hz::ImuCallback(sensor_msgs::Imu imu)
{
	m_i64ImuCount++;
}

void Transform2Hz::WheelFeedbackCallback(andbot::WheelFb wheel_fb)
{
	m_i64WheelFeedbackCount++;
}

void* Transform2Hz::PublishThread(void* dies)
{
	Transform2Hz *pPublishers= (Transform2Hz*)dies;
	pPublishers->StartPublish();
	return NULL;
}

void Transform2Hz::StartPublish()
{
	std_msgs::Float32	temp;
	MonotonicTime		Start, End;
	float				fTemp;
	Start.Now();
	while(ros::ok())
	{	
		//	Start.Now();
		::usleep(m_i64Interval*1000);
				
		//	publish LaserScan Hz
		fTemp = ((float)m_i64LaserScanCount)/((float)((End.Now() - Start).ToMilliSecond()));
		temp.data = fTemp*1000.0f;
		m_pubLaserScanHz.publish(temp);
		
		//	publish Imu Hz
		fTemp = ((float)m_i64ImuCount)/((float)((End.Now() - Start).ToMilliSecond()));
		temp.data = fTemp*1000.0f;
		m_pubImuHz.publish(temp);
		
		//	publish WheelFb Hz
		fTemp = ((float)m_i64WheelFeedbackCount)/((float)((End.Now() - Start).ToMilliSecond()));
		temp.data = fTemp*1000.0f;
		m_pubWheelFeedbackHz.publish(temp);
	};
}

void Transform2Hz::Run()
{
	pthread_create(&m_PublishThreadID, NULL, &Transform2Hz::PublishThread, (void*)this);
	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();
	pthread_join(m_PublishThreadID, NULL);
}