#include <signal.h>
#include <ctime>
#include <pthread.h>

#include "tf2hz.h"
#include "monotonic_time.h"

#define BUF_SIZE	128

Transform2Hz::Transform2Hz(int64_t ms) : m_i64TotalMillionSeconds(0), m_i64LaserScanCount(0), m_i64ImuCount(0),	m_i64WheelFeedbackCount(0),
	m_i64LaserScanPrevCount(0), m_i64ImuPrevCount(0),	m_i64WheelFeedbackPrevCount(0)
{
	m_i64Interval		= 	ms;
	m_bIsExit			=	false;
	m_bIsFirstTime		=	true;
}

bool Transform2Hz::Initial()
{
	m_subLaserScan 		=	m_nhMyNodeHandle.subscribe<sensor_msgs::LaserScan, Transform2Hz>("sensor_msgs/LaserScan", BUF_SIZE, &Transform2Hz::LaserScanCallback, this);
	m_subImu			= 	m_nhMyNodeHandle.subscribe<sensor_msgs::Imu, Transform2Hz>("sensor_msgs/Imu", BUF_SIZE, &Transform2Hz::ImuCallback, this);
	m_subWheelFeedback	=	m_nhMyNodeHandle.subscribe<andbot::WheelFb, Transform2Hz>("andbot/WheelFb", BUF_SIZE, &Transform2Hz::WheelFeedbackCallback, this);
	
	m_pubLaserScanHz	=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/laserscan_hz", BUF_SIZE);
	m_pubImuHz			=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/imu_hz", BUF_SIZE);
	m_pubWheelFeedbackHz=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/whellfb_hz", BUF_SIZE);
	
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
	
	while(ros::ok())
	{	
		Start.Now();
		::usleep(m_i64Interval*1000);
		
		i64LSCnt = m_i64LaserScanCount;
		i64IMUCnt= m_i64ImuCount;
		i64WFDCnt= m_i64WheelFeedbackCount;
		
		//	publish LaserScan Hz
		m_fLaserScanHz = ((float)(i64LSCnt-m_i64LaserScanPrevCount))/((float)((End.Now() - Start).ToMilliSecond()));
		m_i64LaserScanPrevCount=i64LSCnt;
		temp.data = m_fLaserScanHz;
		m_pubLaserScanHz.publish(temp);
		
		//	publish Imu Hz
		m_fImuHz = ((float)(i64IMUCnt-m_i64ImuPrevCount))/((float)((End.Now() - Start).ToMilliSecond()));
		m_i64ImuPrevCount = i64IMUCnt;
		temp.data = m_fImuHz;
		m_pubImuHz.publish(temp);
		
		//	publish WheelFb Hz
		m_fWheelFeedbacknHz = ((float)(i64WFDCnt-m_i64WheelFeedbackPrevCount))/((float)((End.Now() - Start).ToMilliSecond()));
		m_i64WheelFeedbackPrevCount = i64WFDCnt;
		temp.data = m_fWheelFeedbacknHz;
		m_pubWheelFeedbackHz.publish(temp);
		ros::spinOnce();
	};
}

void Transform2Hz::Run()
{
	pthread_create(&m_PublishThreadID, NULL, &Transform2Hz::PublishThread, (void*)this);		
	ros::spin();
	pthread_join(m_PublishThreadID, NULL);
}