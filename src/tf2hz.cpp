#include <signal.h>
#include <ctime>
#include <iostream>
#include <pthread.h>

#include "tf2hz.h"
#include "monotonic_time.h"

#define BUF_SIZE	128

using namespace std;

Transform2Hz::Transform2Hz(int64_t ms) : m_i64TotalMillionSeconds(0), m_i64LaserScanCount(0), m_i64ImuCount(0),	m_i64WheelFeedbackCount(0),
	m_i64LaserScanPrevCount(0), m_i64ImuPrevCount(0),	m_i64WheelFeedbackPrevCount(0)
{
	m_i64Interval		= 	ms;
	m_bIsExit			=	false;
	m_bIsFirstTime		=	true;
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
	//cout<<"Transform2Hz::LaserScanCallback() : "<<m_i64LaserScanCount<<endl;
}

void Transform2Hz::ImuCallback(sensor_msgs::Imu imu)
{
	m_i64ImuCount++;
	//cout<<"Transform2Hz::ImuCallback() : "<<m_i64ImuCount<<endl;
}

void Transform2Hz::WheelFeedbackCallback(andbot::WheelFb wheel_fb)
{
	m_i64WheelFeedbackCount++;
	//cout<<"Transform2Hz::WheelFeedbackCallback() : "<<m_i64WheelFeedbackCount<< endl;
}

void* Transform2Hz::PublishThread(void* dies)
{
//	Transform2Hz *pPublishers= dynamic_cast<Transform2Hz*>(dies);
	Transform2Hz *pPublishers= (Transform2Hz*)dies;
	pPublishers->StartPublish();
	return NULL;
}

void Transform2Hz::StartPublish()
{
	int64_t	i64LSCnt;
	int64_t	i64IMUCnt;
	int64_t	i64WFDCnt;
	std_msgs::Float32	temp;
	MonotonicTime	Start, End;
	Start.Now();
	while(ros::ok())
	{	
		//	Start.Now();
		::usleep(m_i64Interval*1000);
		
		i64LSCnt = m_i64LaserScanCount;
		i64IMUCnt= m_i64ImuCount;
		i64WFDCnt= m_i64WheelFeedbackCount;
		
		//	publish LaserScan Hz
		//	m_fLaserScanHz = ((float)(i64LSCnt-m_i64LaserScanPrevCount))/((float)((End.Now() - Start).ToMilliSecond()));
		m_fLaserScanHz = ((float)i64LSCnt)/((float)((End.Now() - Start).ToMilliSecond()));
		//cout<<"laserscan_count = "<<i64LSCnt-m_i64LaserScanPrevCount<<endl;
		//m_i64LaserScanPrevCount=i64LSCnt;
		temp.data = m_fLaserScanHz*1000.0f;
		//cout<<"laserscan_hz = "<<m_fLaserScanHz<<endl;
		m_pubLaserScanHz.publish(temp);
		
		//	publish Imu Hz
		//	m_fImuHz = ((float)(i64IMUCnt-m_i64ImuPrevCount))/((float)((End.Now() - Start).ToMilliSecond()));
		m_fImuHz = ((float)i64IMUCnt)/((float)((End.Now() - Start).ToMilliSecond()));
		//cout<<"imu_count = "<<i64IMUCnt-m_i64ImuPrevCount<<endl;
		//m_i64ImuPrevCount = i64IMUCnt;
		temp.data = m_fImuHz*1000.0f;
		//cout<<"imu_hz = "<<m_fImuHz<<endl;
		m_pubImuHz.publish(temp);
		
		//	publish WheelFb Hz
		//	m_fWheelFeedbacknHz = ((float)(i64WFDCnt-m_i64WheelFeedbackPrevCount))/((float)((End.Now() - Start).ToMilliSecond()));
		m_fWheelFeedbacknHz = ((float)i64WFDCnt)/((float)((End.Now() - Start).ToMilliSecond()));
		//cout<<"wheelfb_count = "<<i64WFDCnt-m_i64WheelFeedbackPrevCount<<endl;
		//m_i64WheelFeedbackPrevCount = i64WFDCnt;
		temp.data = m_fWheelFeedbacknHz*1000.0f;
		//cout<<"wheelfb_hz = "<<m_fWheelFeedbacknHz<<endl;
		m_pubWheelFeedbackHz.publish(temp);
		//ros::spinOnce();
	};
}

void Transform2Hz::Run()
{
	pthread_create(&m_PublishThreadID, NULL, &Transform2Hz::PublishThread, (void*)this);
	ros::spin();
	pthread_join(m_PublishThreadID, NULL);
}