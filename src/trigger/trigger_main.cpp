/*
 * trigger_main.cpp
 * 
 * create on 21 July 2019
 * author @wntun
 * 
 * */
 
#include <ros/ros.h>
#include "trigger.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "trigger");
	Trigger trg;
	trg.MainLoop();
	return 0;
}

