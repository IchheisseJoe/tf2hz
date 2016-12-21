#include <ros/ros.h>
#include "tf2hz.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tf2hz");
	Transform2Hz tf2hz(1000);
	tf2hz.Initial();
	tf2hz.Run();	
	return 0;
}
