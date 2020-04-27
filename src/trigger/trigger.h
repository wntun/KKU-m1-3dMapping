/*
 * trigger.h
 * 
 * create on 21 July 2019
 * author @wntun
 * 
 * */

#include <ros/ros.h>

#include "ku_mapping/ku_trigger.h"
#include "ku_mapping/ku_fcsignal.h"

using namespace std;

//#define FCMAXNSEC 1900000.0 //1500000.0// 1100- 1900.0
//#define FCMINNSEC 1810000.0


class Trigger{
	protected:
		ros::NodeHandle nh;
		ros::Publisher pub_trigger;
		ros::Subscriber sub_fcSignal;
		
		int prevSignal;
		int headerSeq;
		ros::Time prevTime;
		ku_mapping::ku_trigger msg_trigger;
		
		double trigger_min;
		double trigger_max;
		
		
	public:
		Trigger();
		~Trigger();
		
		void MainLoop();
	
	private:
		void callBackFCSignal(const ku_mapping::ku_fcsignal::ConstPtr &msg);
		
};
