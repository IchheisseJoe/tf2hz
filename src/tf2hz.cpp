#include <signal.h>
#include "tf2hz.h"

#define BUF_SIZE	1024

Transform2Hz::Transform2Hz(int64_t ms) : m_i64TotalMillionSeconds(0), m_i64LaserScanCount(0), m_i64ImuCount(0),	m_i64WheelFeedbackCount(0),
	m_i64LaserScanPrevCount(0), m_i64ImuPrevCount(0),	m_i64WheelFeedbackPrevCount(0)
{
	m_i64Interval		= 	ms;
	m_bIsExit			=	false;
	m_bIsFirstTime		=	true;
}

bool Initial()
{
	m_subLaserScan 		=	m_nhMyNodeHandle.subscribe<sensor_msgs::LaserScan>("sensor_msgs/LaserScan", BUF_SIZE, &Transform2Hz::LaserScanCallback, this);
	m_subImu			= 	m_nhMyNodeHandle.subscribe<sensor_msgs::Imu>("sensor_msgs/Imu", BUF_SIZE, &Transform2Hz::ImuCallback, this);
	m_subWheelFeedback	=	m_nhMyNodeHandle.subscribe<andbot::WheelFb>("andbot/WheelFb", BUF_SIZE, &Transform2Hz::WheelFeedbackCallback, this);
	
	m_pubLaserScanHz	=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/laserscan_hz", BUF_SIZE);
	m_pubImuHz			=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/imu_hz", BUF_SIZE);
	m_pubWheelFeedbackHz=	m_nhMyNodeHandle.advertise<std_msgs::Float32>("/whellfb_hz", BUF_SIZE);
		
	return true;
}

void Transform2Hz::LaserScanCallback(const sensor_msgs::LaserScan& laser_scan)
{
	m_i64LaserScanCount++;
}

void Transform2Hz::ImuCallback(const sensor_msgs::Imu& imu)
{
	m_i64ImuCount++;
}

void Transform2Hz::WheelFeedbackCallback(const andbot::WheelFb& wheel_fb)
{
	m_i64WheelFeedbackCount++;
}

void* Transform2Hz::TimerThread(void* dies)
{
	int64_t	i64LSCnt;
	int64_t	i64IMUCnt;
	int64_t	i64WFDCnt;
	std_msgs::Float32	temp;
	
	do
	{	
		::usleep(m_i64Interval*1000);
		i64LSCnt = m_i64LaserScanCount;
		i64IMUCnt= m_i64ImuCount;
		i64WFDCnt= m_i64WheelFeedbackCount;
		
		//	publish LaserScan Hz
		m_fLaserScanHz = ((float)(i64LSCnt-m_i64LaserScanPrevCount))/(float)m_i64Interval);
		m_i64LaserScanPrevCount=i64LSCnt;
		temp.data = m_fLaserScanHz;
		m_pubLaserScanHz.publish(temp);
		
		//	publish Imu Hz
		m_fImuHz = ((float)(i64IMUCnt-m_i64ImuPrevCount))/(float)m_i64Interval);
		m_i64ImuPrevCount = i64IMUCnt;
		temp.data = m_fImuHz;
		m_pubImuHz.publish(temp);
		
		//	publish WheelFb Hz
		m_fWheelFeedbacknHz = ((float)(i64WFDCnt-m_i64WheelFeedbackPrevCount))/(float)m_i64Interval);
		m_i64WheelFeedbackPrevCount = i64WFDCnt;
		temp.data = m_fWheelFeedbacknHz;
		m_pubWheelFeedbackHz.publish(temp);
		
	}	while(!m_bIsExit);
}



void Transform2Hz::Run()
{
	signal(SIGINT, 
	pthread_create(&m_TimerThreadID, NULL, &Transform2Hz::TimerThread, this);
	ros::spin();
}