/*
 * fcsignal.cpp
 * 
 * created on 21 July 2019
 * author @wntun
 * 
 * */
 
/*
 
 #include "fcsignal.h"
 

 FCSignal::FCSignal(){
	 ROS_INFO("FCSignal Constructor");	 
	 FCSignal::pub_fcSignal = nh.advertise<ku_mapping::ku_fcsignal>("ku_mapping/KU_FCSignal", 1, true);
	 
	 if(wiringPiSetupPhys() == -1){
		 ROS_ERROR("fcsignal.cpp : GPIO setup fail.");
		 exit(1);
	}
	
	pinMode(FCPIN, INPUT);
	/*
	if(wiringPiISR(FCPIN, INT_EDGE_BOTH, &FCSignal::publishInterrupt)<0){
		ROS_ERROR("Unable to setup ISR");
	}*//*
 }
 
 FCSignal::~FCSignal(){}
 
 void FCSignal::MainLoop(){
	 ROS_INFO("FCSignal MainLoop");
	 
	 ros::Rate r(1000);
	 //ros::Timer timer = nh.createTimer(ros::Duration(0.02), readGPIO);
	 //ros::spin();
	 
	 while(ros::ok()){
		msg_fc.value = digitalRead(FCPIN);
		msg_fc.time = ros::Time::now();
		
		/*
		std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
		auto duration = now.time_since_epoch();
		
		msg_fc.time = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
		* *//*
		pub_fcSignal.publish(msg_fc);
	 } 
 }*/
 
 /*
void FCSignal::publishInterrupt(){
	msg_fc.value = digitalRead(FCPIN);
	msg_fc.time = ros::Time::now();
	pub_fcSignal.publish(msg_fc);
}*/


#include <ros/ros.h> 
#include <wiringPi.h>

#include "ku_mapping/ku_fcsignal.h"


using namespace std;

#define FCPIN 29


ros::Publisher pub_fcSignal;
ku_mapping::ku_fcsignal msg_fc;




/*
void readGPIO(const ros::TimerEvent&){
	/*
	 for(int i=0; i<50; i++){		
		msg_fc.value[i] = digitalRead(FCPIN);
		msg_fc.time[i] = ros::Time::now();
	 }
	 * 
	 msg_fc.value = digitalRead(FCPIN);
	 msg_fc.time = ros::Time::now();
	 pub_fcSignal.publish(msg_fc);
 }
 * */
 
 void publishInterrupt(){
	
	msg_fc.value = digitalRead(FCPIN);
	msg_fc.header.stamp = ros::Time::now();
	pub_fcSignal.publish(msg_fc);
	
	/*
	int curr_signal = digitalRead(FCPIN);
	if(curr_signal==0 && prevSignal==1){
		if((ros::Time::now() - prev_time).nsec>FCSIGNALTIMENSEC){
			msg_trigger.value = 1;
			pub_trigger.publish(msg_trigger);
		}
	}
	prev_time = ros::Time::now();
	prevSignal = curr_signal;
	* */
	
}
 int main(int argc, char **argv){
	ros::init(argc, argv, "fcpub");
	ros::NodeHandle nh;	
	if(wiringPiSetupPhys() == -1){
		 ROS_ERROR("fcsignal.cpp : GPIO setup fail.");
		 exit(1);
	}
	
	pinMode(FCPIN, INPUT);
	pub_fcSignal = nh.advertise<ku_mapping::ku_fcsignal>("ku_mapping/KU_FCSignal", 30, true);
	
	
		 	
	//ros::Timer timer = nh.createTimer(ros::Duration(0.000000001), readGPIO);
	if(wiringPiISR(FCPIN, INT_EDGE_BOTH, &publishInterrupt)<0){
		ROS_ERROR("Unable to setup ISR");
	}
	ros::spin();
	
	
	/*
	
	while(ros::ok()){
		msg_fc.value = digitalRead(FCPIN);
		msg_fc.header.stamp = ros::Time::now();
		pub_fcSignal.publish(msg_fc);
	}
	* */
	
	return 0;
}

 
