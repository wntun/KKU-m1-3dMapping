/*
 * checkpoint_main.cpp
 * 
 * created on 20 July 2019
 * author @wntun
 * 
 * */

#include <ros/ros.h>
#include "checkpoint.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "checkpoint");
	CheckPoint cp;
	cp.MainLoop();
	return 0;
}
