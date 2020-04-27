/*
 * checkpoint.h
 * 
 * create on 20 July 2019
 * author @wntun
 * 
 * */

#include<ros/ros.h>

#include "ku_mapping/ku_gps.h"
#include "ku_mapping/ku_trigger.h"
#include "sensor_msgs/Imu.h"

#include <ctime>
#include <chrono>
#include <fstream>

using namespace std;

//#define MININTERVALNSEC 110000000.0 // 110 ms

class CheckPoint{
	protected:
		ros::NodeHandle nh;	
		//ros::NodeHandle pn("~");
		
		ros::Subscriber sub_kuGPS;
		ros::Subscriber sub_vnimu;
		ros::Subscriber sub_kuTrigger;
		
		ku_mapping::ku_gps msgkuGPS;
		sensor_msgs::Imu msgIMU;
		
		ofstream imu_writer;
		ofstream gps_writer;
		ofstream img_writer;
		
		ros::Time prevTime;
		
		double picture_interval;
		
		int fc_count;
	
	public:
		CheckPoint();
		~CheckPoint();
		
		void MainLoop();
	
	private:
		void writeCSVFiles();
		void callBackKUGPS(const ku_mapping::ku_gps::ConstPtr &msg);
		void callBackIMU(const sensor_msgs::Imu::ConstPtr &msg);
		void callBackKUTrigger(const ku_mapping::ku_trigger::ConstPtr &msg);
};
