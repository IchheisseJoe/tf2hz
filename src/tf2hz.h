#ifndef __TF2HZ_H__
#define __TF2HZ_H__

#include <stdint.h>
#include <pthread.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <andbot/WheelFb.h>

class Transform2Hz
{
public:
	Transform2Hz(int64_t ms);
	bool Initial();
	void Run();
	void StartPublish();
private:	
	void LaserScanCallback(sensor_msgs::LaserScan laser_scan);
	void ImuCallback(sensor_msgs::Imu imu);
	void WheelFeedbackCallback(andbot::WheelFb wheel_fb);
	static void* PublishThread(void* obj);
	
		
	ros::NodeHandle	m_nhMyNodeHandle;
	ros::Subscriber	m_subLaserScan;
	ros::Subscriber	m_subImu;
	ros::Subscriber	m_subWheelFeedback;
	ros::Publisher	m_pubLaserScanHz;
	ros::Publisher	m_pubImuHz;
	ros::Publisher	m_pubWheelFeedbackHz;
	pthread_t		m_PublishThreadID;
	int64_t			m_i64Interval;
	int64_t			m_i64LaserScanCount;
	int64_t			m_i64ImuCount;
	int64_t			m_i64WheelFeedbackCount;
};
	
#endif	