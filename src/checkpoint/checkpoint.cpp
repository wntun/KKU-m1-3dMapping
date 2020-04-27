/*
 * checkpoint.cpp
 * 
 * created on 20 July 2019
 * author @wntun
 * 
 * */

#include "checkpoint.h"



CheckPoint::CheckPoint(){
	ROS_INFO("checkpoint constructor");
	
	prevTime = ros::Time::now();
	
	fc_count = 1;
	
	sub_kuGPS = nh.subscribe<ku_mapping::ku_gps>("ku_mapping/KU_GPS", 100, &CheckPoint::callBackKUGPS, this);
	sub_vnimu = nh.subscribe<sensor_msgs::Imu>("ku_mapping/IMU", 100, &CheckPoint::callBackIMU, this);
	sub_kuTrigger = nh.subscribe<ku_mapping::ku_trigger>("ku_mapping/KU_Trigger", 30, &CheckPoint::callBackKUTrigger, this);
	
	string file_folder;
	nh.getParam("/checkpoint/file_folder", file_folder);
	nh.getParam("/checkpoint/picture_interval_nsec", picture_interval);
	
	ROS_INFO("picture interval: %f", picture_interval);
	
	// imu & gps writer initialization
	int write_time = (int)ros::Time::now().sec;
	ROS_INFO(" GPS, IMU and Picture info are saving in %s", file_folder.c_str());
	
	string imu_file_name = file_folder + "imu_" + std::to_string(write_time) + ".csv"; 
	string gps_file_name = file_folder + "gps_" + std::to_string(write_time) + ".csv";
	string img_file_name = file_folder + "img_stamp_" + std::to_string(write_time) + ".csv";
	CheckPoint::imu_writer.open(imu_file_name);
	CheckPoint::gps_writer.open(gps_file_name);
	CheckPoint::img_writer.open(img_file_name);
	
	if(!imu_writer){
	    ROS_WARN("IMU file error");
	}
	if(!gps_writer){
	    ROS_WARN("GPS file error");
	}
	if(!img_writer){
	    ROS_WARN("Picture info. file error");
	}
	CheckPoint::imu_writer<<"time stamp, system time, acc_x, acc_y, acc_z, vel_x, vel_y, vel_z, roll, pitch, yaw"<< endl;
	//gps_writer<<"type, id, gps time, gps week, latitude, longitude, altitude, relative altitude, gps altitude, roll, pitch, yaw"<<endl;
	CheckPoint::gps_writer<<"timestamp, latitude, longitude, altitude, roll, pitch, yaw"<<endl;
	CheckPoint::img_writer<<"time stamp, system time"<<endl;
}

CheckPoint::~CheckPoint(){}

void CheckPoint::MainLoop(){
	ROS_INFO("checkpoint mainloop");
	//writeCSVFiles();
	ros::Rate rate(1);
	while(ros::ok()){
		ros::spinOnce();
		//rate.sleep();
		//delay(100);
		
		// need to call writeCSVFiles when we get fc signal 
		// done in ku_trigger callback function
		
	}
	
	ros::Duration(0.5).sleep();
	CheckPoint::imu_writer.close();
	CheckPoint::gps_writer.close();
	CheckPoint::img_writer.close();
}


void CheckPoint::callBackKUGPS(const ku_mapping::ku_gps::ConstPtr &msg){
	CheckPoint::msgkuGPS.latitude = msg->latitude;
	CheckPoint::msgkuGPS.longitude = msg->longitude;
	CheckPoint::msgkuGPS.relative_altitude = msg->relative_altitude;
	CheckPoint::msgkuGPS.roll = msg->roll;
	CheckPoint::msgkuGPS.pitch = msg->pitch;
	CheckPoint::msgkuGPS.yaw = msg->yaw;
}

void CheckPoint::callBackIMU(const sensor_msgs::Imu::ConstPtr &msg){
	CheckPoint::msgIMU.linear_acceleration.x = msg->linear_acceleration.x;
	CheckPoint::msgIMU.linear_acceleration.y = msg->linear_acceleration.y;
	CheckPoint::msgIMU.linear_acceleration.z = msg->linear_acceleration.z;
	CheckPoint::msgIMU.angular_velocity.x = msg->angular_velocity.x;
	CheckPoint::msgIMU.angular_velocity.y = msg->angular_velocity.y;
	CheckPoint::msgIMU.angular_velocity.z = msg->angular_velocity.z;
	CheckPoint::msgIMU.orientation.x = msg->orientation.x;
	CheckPoint::msgIMU.orientation.y = msg->orientation.y;
	CheckPoint::msgIMU.orientation.z = msg->orientation.z;	
}

void CheckPoint::callBackKUTrigger(const ku_mapping::ku_trigger::ConstPtr &msg){
	/*
	if(msg->value==1){
		ROS_INFO("Received FC Signal");
		writeCSVFiles();
	}
	* */
	
	if(msg->value == 1){
		double duration = (msg->header.stamp - prevTime).sec; //ader.stamp - prevTime).sec *1000000000;
		ROS_INFO("Duration : [%f]", duration);
		if(duration>=picture_interval){
			ROS_INFO("[%d] Received FC Signal!", fc_count);
			ROS_INFO("Duration : [%f]", duration);
			writeCSVFiles();
			prevTime = msg->header.stamp;
			fc_count += 1;
		}	
		//prevTime = msg->header.stamp;			
	}
	/*
	else if(msg->value==-1)
	{
		prevTime = msg->header.stamp;
	}*/
	
	
}

void CheckPoint::writeCSVFiles(){
	//ROS_INFO("checkpoint writeCSVFiles");
	
	auto sys_time = std::chrono::system_clock::to_time_t(chrono::system_clock::now()); //std::chrono::system_clock::now();
	std::string sys_time_temp = std::ctime(&sys_time);
	std::string sys_time_str = sys_time_temp.substr(0, sys_time_temp.size()-1);
	// imu 
	CheckPoint::imu_writer<< (int)ros::Time::now().sec; //(double)msgIMU.header.stamp.sec;
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< sys_time_str;
	CheckPoint::imu_writer<<",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.linear_acceleration.x; 
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.linear_acceleration.y;
	CheckPoint::imu_writer<< ","; 
	CheckPoint::imu_writer<< CheckPoint::msgIMU.linear_acceleration.z;
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.angular_velocity.x; 
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.angular_velocity.y;
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.angular_velocity.z;
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.orientation.x;
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.orientation.y; 
	CheckPoint::imu_writer<< ",";
	CheckPoint::imu_writer<< CheckPoint::msgIMU.orientation.z;
	CheckPoint::imu_writer<<endl<<std::flush;
	
	//gps
	CheckPoint::gps_writer<<(int)ros::Time::now().sec;
	CheckPoint::gps_writer<< ",";
	CheckPoint::gps_writer<< std::to_string(CheckPoint::msgkuGPS.latitude); 
	CheckPoint::gps_writer<< ",";
	CheckPoint::gps_writer<< std::to_string(CheckPoint::msgkuGPS.longitude);
	CheckPoint::gps_writer<< ",";
	CheckPoint::gps_writer<< std::to_string(CheckPoint::msgkuGPS.relative_altitude); 
	CheckPoint::gps_writer<< ",";
	CheckPoint::gps_writer<< std::to_string(CheckPoint::msgkuGPS.roll); 
	CheckPoint::gps_writer<< ",";
	CheckPoint::gps_writer<< std::to_string(CheckPoint::msgkuGPS.pitch); 
	CheckPoint::gps_writer<< ",";
	CheckPoint::gps_writer<< std::to_string(CheckPoint::msgkuGPS.yaw); 
	CheckPoint::gps_writer<< endl<<std::flush;
	
	//img
	CheckPoint::img_writer<<(int)ros::Time::now().sec;
	CheckPoint::img_writer<<",";
	CheckPoint::img_writer<<sys_time_str;
	CheckPoint::img_writer<<endl<<std::flush;
}
