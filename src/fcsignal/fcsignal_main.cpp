/*
 * fcsignal_main.cpp
 * 
 * created on 21 July 2019
 * author @wntun
 * 
 * */

#include <ros/ros.h>
#include "fcsignal.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "fcpub");
	FCSignal fc;
	fc.MainLoop();
	return 0;
}
