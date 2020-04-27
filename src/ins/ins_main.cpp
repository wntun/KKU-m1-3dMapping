/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */
 
#include <iostream>
#include <cmath>

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "std_srvs/Empty.h"
#include "ku_mapping/ku_gps.h"


#define PI 3.14159265358979323846  /* pi */

ros::Publisher pubIMU, pubMag, pubGPS, pubKuGPS, pubOdom, pubTemp, pubPres;
ros::ServiceServer resetOdomSrv;

//Unused covariances initilized to zero's
boost::array<double, 9ul> linear_accel_covariance = { };
boost::array<double, 9ul> angular_vel_covariance = { };
boost::array<double, 9ul> orientation_covariance = { };
XmlRpc::XmlRpcValue rpc_temp;


// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;


#if !defined (O_BINARY)
    /*To have portable binary open() on *nix and on Windows */
    #define O_BINARY 0
#endif
// Method declarations for future use.
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);
//void timer_start(std::function<void(void)> func, unsigned int interval);
//void capture_to_file();
//void takePicture_gphoto();
//GPContext* sample_create_context();


std::string frame_id;
// Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
bool tf_ned_to_enu;
// Initial position after getting a GPS fix.
vec3d initial_position;
bool initial_position_set = false;

//camera gphoto
//Camera *camera;
//GPContext *context;


// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    boost::array<double, 9ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 9; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}


// Reset initial position to current position
bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    initial_position_set = false;
    return true;
}

int main(int argc, char *argv[])
{    
    // ROS node init
    ros::init(argc, argv, "ins");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pubIMU = n.advertise<sensor_msgs::Imu>("ku_mapping/IMU", 100);
    //pubMag = n.advertise<sensor_msgs::MagneticField>("ku_mapping/Mag", 1000);
    pubGPS = n.advertise<sensor_msgs::NavSatFix>("ku_mapping/GPS", 100);
    pubKuGPS = n.advertise<ku_mapping::ku_gps>("ku_mapping/KU_GPS", 100);
    //pubOdom = n.advertise<nav_msgs::Odometry>("ku_mapping/Odom", 1000);
    //pubTemp = n.advertise<sensor_msgs::Temperature>("ku_mapping/Temp", 1000);
    //pubPres = n.advertise<sensor_msgs::FluidPressure>("ku_mapping/Pres", 1000);

    resetOdomSrv = n.advertiseService("reset_odom", resetOdom);

    // Serial Port Settings
    string SensorPort;
    int SensorBaudrate;
    int async_output_rate;
    
    //int camera_interval;

    // Load all params
    pn.param<std::string>("frame_id", frame_id, "ku_mapping");
    pn.param<bool>("tf_ned_to_enu", tf_ned_to_enu, false);
    pn.param<int>("async_output_rate", async_output_rate, 40);
    pn.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
    pn.param<int>("serial_baud", SensorBaudrate, 115200);

    //Call to set covariances
    if(pn.getParam("linear_accel_covariance",rpc_temp))
    {
        linear_accel_covariance = setCov(rpc_temp);
    }
    if(pn.getParam("angular_vel_covariance",rpc_temp))
    {
        angular_vel_covariance = setCov(rpc_temp);
    }
    if(pn.getParam("orientation_covariance",rpc_temp))
    {
        orientation_covariance = setCov(rpc_temp);
    }
    
    
    ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

    // Create a VnSensor object and connect to sensor
    VnSensor vs;

    // Default baudrate variable
    int defaultBaudrate;
    // Run through all of the acceptable baud rates until we are connected
    // Looping in case someone has changed the default
    bool baudSet = false;
    while(!baudSet){
        // Make this variable only accessible in the while loop
        static int i = 0;
        defaultBaudrate = vs.supportedBaudrates()[i];
        ROS_INFO("Connecting with default at %d", defaultBaudrate);
        // Default response was too low and retransmit time was too long by default.
        // They would cause errors
        vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
        vs.setRetransmitDelayMs(50);  // Retransmit every 50 ms

        // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
        // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
        // All other values seem to work fine.
        try{
            // Connect to sensor at it's default rate
            if(defaultBaudrate != 128000 && SensorBaudrate != 128000)
            {
                vs.connect(SensorPort, defaultBaudrate);
                // Issues a change baudrate to the VectorNav sensor and then
                // reconnects the attached serial port at the new baudrate.
                vs.changeBaudRate(SensorBaudrate);
                // Only makes it here once we have the default correct
                ROS_INFO("Connected baud rate is %d",vs.baudrate());
                baudSet = true;
            }
        }
        // Catch all oddities
        catch(...){
            // Disconnect if we had the wrong default and we were connected
            vs.disconnect();
            ros::Duration(0.2).sleep();
        }
        // Increment the default iterator
        i++;
        // There are only 9 available data rates, if no connection
        // made yet possibly a hardware malfunction?
        if(i > 8)
        {
            break;
        }
    }

    // Now we verify connection (Should be good if we made it this far)
    if(vs.verifySensorConnectivity())
    {
        ROS_INFO("Device connection established");
    }else{
        ROS_ERROR("No device communication");
        ROS_WARN("Please input a valid baud rate. Valid are:");
        ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
        ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
    }
    // Query the sensor's model number.
    string mn = vs.readModelNumber();
    ROS_INFO("Model Number: %s", mn.c_str());

    // Set Data output Freq [Hz]
    vs.writeAsyncDataOutputFrequency(async_output_rate);

    // Configure binary output message
    BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            1000 / async_output_rate,  // update rate [ms]
            COMMONGROUP_QUATERNION
            | COMMONGROUP_ANGULARRATE
            | COMMONGROUP_POSITION
            | COMMONGROUP_ACCEL
            | COMMONGROUP_MAGPRES
            | COMMONGROUP_TIMEGPS, // we want GPS time and week
            TIMEGROUP_TIMEGPS,  // we want GPS time and week
            IMUGROUP_NONE,
            GPSGROUP_POSLLA,
            ATTITUDEGROUP_YPRU, //<-- returning yaw pitch roll uncertainties
            INSGROUP_INSSTATUS
            | INSGROUP_POSLLA
            | INSGROUP_POSECEF
            | INSGROUP_VELBODY
            | INSGROUP_ACCELECEF,
            GPSGROUP_NONE);

    vs.writeBinaryOutput1(bor);
    
    // Set Data output Freq [Hz]
    vs.writeAsyncDataOutputFrequency(async_output_rate);
    vs.registerAsyncPacketReceivedHandler(NULL, BinaryAsyncMessageReceived);
    
    
    while (ros::ok())
    {
        ros::spinOnce();
         // Need to make sure we disconnect properly. Check if all ok.
    }

    // Node has been terminated
    vs.unregisterAsyncPacketReceivedHandler();
    ros::Duration(0.5).sleep();
    vs.disconnect();
    
    //gp_camera_exit(camera, context);
    return 0;
}

//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
    ku_mapping::ku_gps msgkuGPS;
    // IMU
    sensor_msgs::Imu msgIMU;
    msgIMU.header.stamp = ros::Time::now();
    msgIMU.header.frame_id = frame_id;
    

    if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration())
    {

        vec4f q = cd.quaternion();
        vec3f ar = cd.angularRate();
        vec3f al = cd.acceleration();

        //Quaternion message comes in as a Yaw (z) Pitch (y) Roll (x) format
        if (tf_ned_to_enu)
        {
            // Flip x and y then invert z
            msgIMU.orientation.x = q[1];
            msgIMU.orientation.y = q[0];
            msgIMU.orientation.z = -q[2];
            msgIMU.orientation.w = q[3];

            if (cd.hasAttitudeUncertainty())
            {
                vec3f orientationStdDev = cd.attitudeUncertainty();
                msgIMU.orientation_covariance[0] = orientationStdDev[1]*orientationStdDev[1]*PI/180; // Convert to radians Pitch
                msgIMU.orientation_covariance[4] = orientationStdDev[0]*orientationStdDev[0]*PI/180; // Convert to radians Roll
                msgIMU.orientation_covariance[8] = orientationStdDev[2]*orientationStdDev[2]*PI/180; // Convert to radians Yaw
            }
            // Flip x and y then invert z
            msgIMU.angular_velocity.x = ar[1];
            msgIMU.angular_velocity.y = ar[0];
            msgIMU.angular_velocity.z = -ar[2];
            // Flip x and y then invert z
            msgIMU.linear_acceleration.x = al[1];
            msgIMU.linear_acceleration.y = al[0];
            msgIMU.linear_acceleration.z = -al[2];
        }
        else
        {
            msgIMU.orientation.x = q[0];
            msgIMU.orientation.y = q[1];
            msgIMU.orientation.z = q[2];
            msgIMU.orientation.w = q[3];

            if (cd.hasAttitudeUncertainty())
            {
                vec3f orientationStdDev = cd.attitudeUncertainty();
                msgIMU.orientation_covariance[0] = orientationStdDev[2]*orientationStdDev[2]*PI/180; // Convert to radians Roll
                msgIMU.orientation_covariance[4] = orientationStdDev[1]*orientationStdDev[1]*PI/180; // Convert to radians Pitch
                msgIMU.orientation_covariance[8] = orientationStdDev[0]*orientationStdDev[0]*PI/180; // Convert to radians Yaw
            }
            msgIMU.angular_velocity.x = ar[0];
            msgIMU.angular_velocity.y = ar[1];
            msgIMU.angular_velocity.z = ar[2];
            msgIMU.linear_acceleration.x = al[0];
            msgIMU.linear_acceleration.y = al[1];
            msgIMU.linear_acceleration.z = al[2];
        }
        // Covariances pulled from parameters
        msgIMU.angular_velocity_covariance = angular_vel_covariance;
        msgIMU.linear_acceleration_covariance = linear_accel_covariance;
        
        if(cd.hasPositionEstimatedLla())
    {
        //if(cd.hasInsStatus()){
        //    printf("\n %d vs %d \n", cd.insStatus(), INSSTATUS_GPS_FIX);
        //}
        vec3d lla = cd.positionEstimatedLla();

        sensor_msgs::NavSatFix msgGPS;
        msgGPS.header.stamp = msgIMU.header.stamp;
        msgGPS.header.frame_id = msgIMU.header.frame_id;
        msgGPS.latitude = lla[0];
        msgGPS.longitude = lla[1];
        msgGPS.altitude = lla[2];
        pubGPS.publish(msgGPS);
        
        // ku gps
        msgkuGPS.type = "CAM";     
        msgkuGPS.roll = msgIMU.orientation.x;
        msgkuGPS.pitch = msgIMU.orientation.y;
        msgkuGPS.yaw = msgIMU.orientation.z;
        msgkuGPS.latitude = lla[0];
        msgkuGPS.longitude = lla[1];
        msgkuGPS.altitude = 0;
        msgkuGPS.relative_altitude = lla[2];
        msgkuGPS.gps_altitude = 0;
        if(cd.hasTimeGps())
        {
            msgkuGPS.gps_time = cd.timeGps();
            //printf("GPS time:  %d\n", cd.timeGps());
        }
    
        if(cd.hasWeek())
        {
            msgkuGPS.gps_week = cd.week();
            //printf("GPS time week :  %d \n", cd.week());
        }
        
        
        
    }

    else
    {
        msgkuGPS.type = "CAM";
               
        msgkuGPS.roll = msgIMU.orientation.x;
        msgkuGPS.pitch = msgIMU.orientation.y;
        msgkuGPS.yaw = msgIMU.orientation.z;
        msgkuGPS.latitude = 0;
        msgkuGPS.longitude = 0;
        msgkuGPS.altitude = 0;
        msgkuGPS.relative_altitude = 0;
        msgkuGPS.gps_altitude = 0;
        //pubKuGPS.publish(msgkuGPS);
        
    }
        
        
        pubIMU.publish(msgIMU);
        pubKuGPS.publish(msgkuGPS);
        
        
    }

    // Magnetic Field
    if (cd.hasMagnetic())
    {
        vec3f mag = cd.magnetic();
        sensor_msgs::MagneticField msgMag;
        msgMag.header.stamp = msgIMU.header.stamp;
        msgMag.header.frame_id = msgIMU.header.frame_id;
        msgMag.magnetic_field.x = mag[0];
        msgMag.magnetic_field.y = mag[1];
        msgMag.magnetic_field.z = mag[2];
        //pubMag.publish(msgMag);
    }

    // GPS
    /*
    if(cd.hasTimeGps())
    {
        msgkuGPS.gps_time = cd.timeGps();
            //printf("GPS time:  %d\n", cd.timeGps());
    }

    
    if(cd.hasWeek())
    {
        msgkuGPS.gps_week = cd.week();
            //printf("GPS time week :  %d \n", cd.week());
    }
    //* */
    
    //printf("INS status %d \n", cd.hasInsStatus());
    //ROS_INFO("GPS data problem");
    
    //if (cd.insStatus() == INSSTATUS_GPS_FIX) // || cd.insStatus() ==3)
    
    
    // Temperature
    if (cd.hasTemperature())
    {
        float temp = cd.temperature();

        sensor_msgs::Temperature msgTemp;
        msgTemp.header.stamp = msgIMU.header.stamp;
        msgTemp.header.frame_id = msgIMU.header.frame_id;
        msgTemp.temperature = temp;
        //pubTemp.publish(msgTemp);
    }

    // Barometer
    if (cd.hasPressure())
    {
        float pres = cd.pressure();

        sensor_msgs::FluidPressure msgPres;
        msgPres.header.stamp = msgIMU.header.stamp;
        msgPres.header.frame_id = msgIMU.header.frame_id;
        msgPres.fluid_pressure = pres;
        //pubPres.publish(msgPres);
    }
}

/*
void takePicture(){
    auto sys_time = std::chrono::system_clock::to_time_t(chrono::system_clock::now()); //std::chrono::system_clock::now();
    std::string sys_time_temp = std::ctime(&sys_time);
    std::string sys_time_str = sys_time_temp.substr(0, sys_time_temp.size()-1);
    
	pinMode(CAMERAPIN, OUTPUT);
	//softPwmCreate(CAMERAPIN, 0, 100);
	digitalWrite(CAMERAPIN, HIGH);
	delay(200);
	digitalWrite(CAMERAPIN, LOW);
	
    
    //img_stamp_writer<<(double)sys_time;

    delay(400);
}

// timer to capture image in regular interval
void timer_start(std::function<void(void)> func, unsigned int interval){
	std::thread([func, interval](){
		while(true){
			func();
			std::this_thread::sleep_for(std::chrono::milliseconds(interval));
		}
	}).detach();
}


// capture image and save it in specified folder yaml with name as sec sequence
// use gphoto2 command
void takePicture_gphoto(){
	ROS_INFO("capturing image");
	std::string time_sec = to_string(ros::Time::now().sec);
	std::string gphoto_command = "gphoto2 --capture-image-and-download --filename " + image_folder + time_sec + ".arw"; //infinity frames 30s interval
	system(gphoto_command.c_str());
}

// ref https://github.com/gphoto/libgphoto2/blob/master/examples/sample-capture.c
// capture image and save it in specified folder yaml with name as sec sequence
// use libgphoto library
void capture_to_file(){
    ROS_INFO("capturing image");

    int fd, retval;
    CameraFile *file;
    CameraFilePath camera_file_path;
    char *data;
    unsigned long size;

    std::string time_sec = to_string(ros::Time::now().sec) + ".arw";


    strcpy(camera_file_path.folder,"/");
    strcpy(camera_file_path.name, time_sec.c_str());

    retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
    retval = gp_file_new(&file);

    retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);
    gp_file_get_data_and_size (file, (const char**)&data, &size);

    FILE *f;

    std::string fn = image_folder+time_sec; //+".arw";

    f = fopen(fn.c_str(), "wb");
    if(f){
        retval = fwrite(data, size, 1, f);
        if(retval != size){
            printf("  fwrite size %ld, written %d\n", size, retval);
        }
        fclose(f);
    }
    else
        printf("  fopen foo2.jpg failed.\n");

    gp_file_free(file);
    retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);
}

GPContext* sample_create_context() {
    GPContext *context;

    // This is the mandatory part 
    context = gp_context_new();

    // All the parts below are optional! 
        gp_context_set_error_func (context, ctx_error_func, NULL);
        gp_context_set_status_func (context, ctx_status_func, NULL);

    // also:
    //gp_context_set_cancel_func    (p->context, ctx_cancel_func,  p);
        //gp_context_set_message_func   (p->context, ctx_message_func, p);
        //if (isatty (STDOUT_FILENO))
                //gp_context_set_progress_funcs (p->context, ctx_progress_start_func, ctx_progress_update_func,ctx_progress_stop_func, p);
    return context;
}
*/
