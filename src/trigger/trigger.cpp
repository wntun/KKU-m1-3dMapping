/*
 * trigger.cpp
 * 
 * create on 21 July 2019
 * author @wntun
 * 
 * */
 
 #include "trigger.h"
 
 Trigger::Trigger(){
	ROS_INFO("Trigger Constructer~");
	pub_trigger = nh.advertise<ku_mapping::ku_trigger>("ku_mapping/KU_Trigger", 30, true);
	sub_fcSignal = nh.subscribe<ku_mapping::ku_fcsignal>("ku_mapping/KU_FCSignal", 30, &Trigger::callBackFCSignal, this);
	prevSignal = -1;
	prevTime = ros::Time::now();
	
	nh.getParam("/tgpub/trigger_min_nanosecond", trigger_min);
	nh.getParam("/tgpub/trigger_max_nanosecond", trigger_max);
	
	ROS_INFO("Trigger min : %f, trigger max : %f", trigger_min, trigger_max);
 }
 
 Trigger::~Trigger(){}
 
 void Trigger::MainLoop(){
	 ROS_INFO("Trigger MainLoop");
	 //ros::Timer timer = nh.createTimer(ros::Duation(0.002), checkSignal);
	 //ros::spin();
	 
	 //ros::Rate r(4000);
	 
	 
	 
	 while(ros::ok()){
		 ros::spinOnce();
		 //ros::Duration(0.000001).sleep();
		 //r.sleep();
	 }
	 
	 //ros::AsyncSpinner spinner(1);
	 //spinner.start();
	 //ros::waitForShutdown();

	 
 }

void Trigger::callBackFCSignal(const ku_mapping::ku_fcsignal::ConstPtr &msg){
	/*
	if(headerSeq!=msg->header.seq){
		if(prevSignal == 1){
			msg_trigger.header.stamp = ros::Time::now();
			//ROS_INFO("Elapsed time : %d", (msg->time - prevTime).nsec);
			if((msg->header.stamp - prevTime).nsec>FCSIGNALTIMENSEC){
				msg_trigger.value = 1;
				pub_trigger.publish(msg_trigger);
			}
			else{
				msg_trigger.value = 0;
			}
		}
		prevSignal = msg->value;
		prevTime = msg->header.stamp;
		headerSeq = msg->header.seq;
	}	
	* */
	/*
	msg_trigger.value = 2;
	if(prevSignal ==-1){
		prevTime = msg->header.stamp;
		prevSignal = msg->value;
	}
	msg_trigger.header.stamp = msg->header.stamp;//msg->header.stamp;
	* */
	if(prevSignal==1 && msg->value==0){
		double duration = (msg->header.stamp - prevTime).nsec;
		//ROS_INFO("Elapsed time : %f", (duration));
		if(duration<=trigger_max && duration>=trigger_min){
			ROS_INFO("Elapsed time : %f", (duration));
			msg_trigger.value = 1;
			msg_trigger.header.stamp = msg->header.stamp;
			pub_trigger.publish(msg_trigger);
			
		}
		prevSignal = msg->value;
	}
	else if(prevSignal==0 && msg->value==1){
		prevSignal = msg->value;
		prevTime = msg->header.stamp;
	}
	
	if(prevSignal==-1){
		prevSignal = msg->value;
		prevTime = msg->header.stamp;
	}
	
	/*
	if(prevSignal != msg->value){
		if(msg->value==0){
			double duration = (msg->header.stamp - prevTime).nsec;
			if(duration<=FCMAXNSEC && duration>=FCMINNSEC){
				msg_trigger.value = 1;
				msg_trigger.header.stamp = msg->header.stamp;
				pub_trigger.publish(msg_trigger);
				ROS_INFO("Elapsed time : %f", (duration));
			}
		}
		prevSignal = msg->value;
		prevTime = msg->header.stamp;
	}
	/*
	
	if(prevSignal!=msg->value){
		if(prevSignal == 1){						
			double duration = (msg->header.stamp - prevTime).nsec;
			//ROS_INFO("Elapsed time : %f", (duration*1000));
			//if(duration>=FCMINNSEC){
			if(duration<=FCMAXNSEC && duration>=FCMINNSEC){
				msg_trigger.value = 1;
				msg_trigger.header.stamp = msg->header.stamp;
				ROS_INFO("Elapsed time : %f", duration);
				//pub_trigger.publish(msg_trigger);
				//ROS_INFO("Elapsed time : %d", (msg->header.stamp - prevTime).nsec);
				//ros::Duration(0.00000001).sleep();
				pub_trigger.publish(msg_trigger);
			}
			prevSignal = msg->value;
			prevTime = msg->header.stamp;
		}
		else if(prevSignal==0 || prevSignal==-1){
			prevSignal = msg->value;
			prevTime = msg->header.stamp;
		}

		//prevSignal = msg->value;
		//prevTime = msg->header.stamp; //ros::Time::now();// msg->header.stamp;
	}
	*/
	
}


