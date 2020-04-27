/*
 * fcsignal.h
 * 
 * created on 21 July 2019
 * author @wntun
 * 
 * */

#include <ros/ros.h> 
#include <wiringPi.h>
//#include <chrono>
//#include <ctime>

#include "ku_mapping/ku_fcsignal.h"


using namespace std;

#define FCPIN 29

class FCSignal{
	protected:
		ros::NodeHandle nh;		
		ros::Publisher pub_fcSignal;
		ku_mapping::ku_fcsignal msg_fc;
		int curr_signal;
		
	
	public:
		FCSignal();
		~FCSignal();
		
		void MainLoop();
		//void publishInterrupt();
	
	
		
};
