#include <ros/ros.h>
#include <iostream>
#include "tf2hz.h"

using namespace std;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tf2hz");
	Transform2Hz tf2hz(5000);
	tf2hz.Initial();
	tf2hz.Run();
	return 0;
}
