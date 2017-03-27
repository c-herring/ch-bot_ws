#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

class MotorController
{
	public:
		MotorController();
		void PublishEncoders();
		void PublishVelocities();
		void printDebug();
	
	private:
		// I2C transfer buffers
		char TXbuf[50];
		char RXbuf[50];
		int TWIDelay;
		// File pointer for I2C motor controller
		int fp_i2cController;

		// Ros nodehandle. Main access point for all ROS comms
		ros::NodeHandle nh;
		
		// Message objects
		std_msgs::String test_msg;
		geometry_msgs::Twist vel_msg;
		geometry_msgs::Point wheel_vel_msg;
		std_msgs::Int32 left_encoder_pos_msg; 
		std_msgs::Int32 right_encoder_pos_msg;
		

		
		// Pubs
		ros::Publisher testPub;
		ros::Publisher LencoderPub;
		ros::Publisher RencoderPub;		
		ros::Publisher velPub;
		
		// Subs
		ros::Subscriber direct_cmd; // Send a null terminated ASCII command string directly to motor controller
		ros::Subscriber set_vel; // Set velocities to achieve twist
		
		// Callbacks
		void cmd_callback(const std_msgs::String::ConstPtr& msg);
		void set_vel_callback(const geometry_msgs::Twist msg);
				
		// Robot Geometry
		float wheelGeometryX[2];
		float wheelGeometryY[2];
		float wheelRad;
		int ticksPerRev;
		
};


MotorController::MotorController()
{
	TWIDelay = 500;
	
	// Initialise geometry
	// Left wheel is in +ve X, right is -ve
	wheelGeometryX[0] = 90;
	wheelGeometryX[1] = -90;
	
	wheelGeometryY[0] = 0;
	wheelGeometryY[1] = 0;
	
	wheelRad = 50;
	ticksPerRev = 768;
	
	// Subscribe to the command topics
	direct_cmd = nh.subscribe<std_msgs::String>("direct_cmd", 100, &MotorController::cmd_callback, this);
	set_vel = nh.subscribe<geometry_msgs::Twist>("set_vel", 100, &MotorController::set_vel_callback, this);
	//twist_vel = nh.subscribe<geometry_msgs::Twist>("twist_set", 100, &MotorController::set_twist_callback, this);
	
	// -------- Create the publishers --------
	testPub = nh.advertise<std_msgs::String>("test_pub", 100);
	LencoderPub = nh.advertise<std_msgs::Int32>("L_encoder_pos", 100);
	RencoderPub = nh.advertise<std_msgs::Int32>("R_encoder_pos", 100);	
	velPub = nh.advertise<geometry_msgs::Point>("wheel_velocities", 100);
	
	// -------- Set up wiringPi --------
	wiringPiSetupSys();
	if ( (fp_i2cController = wiringPiI2CSetup(0x08)) == -1)
	{
		printf("Failed to connect to device 0x08 - Exiting\n");
	}
}

void MotorController::PublishEncoders()
{
	// Set controllers to output positions
	write(fp_i2cController, "?P", 2);
	delayMicroseconds(TWIDelay);
	
	// Read from each controller
	read(fp_i2cController, RXbuf, 50);
	delayMicroseconds(TWIDelay);
	
	// Pull data out from the strings
	sscanf(RXbuf, "PA%ldB%ld", &(left_encoder_pos_msg.data), &(right_encoder_pos_msg.data));	

	// Publish
	LencoderPub.publish(left_encoder_pos_msg);
	RencoderPub.publish(right_encoder_pos_msg);
	
}

void MotorController::PublishVelocities()
{
	// Set controllers to output velocities
	write(fp_i2cController, "?V", 2);
	delayMicroseconds(TWIDelay);
	
	// Read from each controller
	read(fp_i2cController, RXbuf, 50);
	delayMicroseconds(TWIDelay);
	
	int32_t tempLeft;
	int32_t tempRight;
	
	// Pull data out from the strings
	sscanf(RXbuf, "VA%ldB%ld", &(tempRight), &(tempLeft));	
	wheel_vel_msg.x = tempLeft;
	wheel_vel_msg.y = tempRight;
	printf("Publishing y = %ld\tx = %ld\n", tempRight, tempLeft);
	
	velPub.publish(wheel_vel_msg);
}


void MotorController::cmd_callback(const std_msgs::String::ConstPtr& msg)
{
	
	write(fp_i2cController, msg->data.c_str(), msg->data.length() + 1);
	delayMicroseconds(TWIDelay);
	ROS_INFO("Command sent to Left Controller: [%s]", msg->data.c_str());
}

/*
 *	Set instantaneous tangental velocity (twist.linear.x) and angular velocity (twist.anguilar.z) in m/s
 *  Input is m/s and rad/s, must convert that to encoder ticks/sec.
 */
void MotorController::set_vel_callback(const geometry_msgs::Twist msg)
{
	// Construct and send the command strings
	int32_t L_motor = (int32_t) -((msg.linear.x*1000 - wheelGeometryX[0]*sin(msg.angular.z)) / (2*3.1415*wheelRad) * ticksPerRev);
	int32_t R_motor = (int32_t) ((msg.linear.x*1000 - wheelGeometryX[1]*sin(msg.angular.z)) / (2*3.1415*wheelRad) * ticksPerRev);
	
	sprintf(TXbuf, "VA%ldB%ld", R_motor, L_motor);
	write(fp_i2cController, TXbuf, strlen(TXbuf));
	delayMicroseconds(TWIDelay);
}

/*
 * Set velocity in X, Y and roation about Z in mm/s
 */
 /*
void MotorController::set_twist_callback(const geometry_msgs::Twist msg)
{
	int32_t RightFront;
	int32_t LeftFront;
	int32_t RightRear;
	int32_t LeftRear;
	
	RightFront = (int32_t) -((msg.linear.y - msg.linear.x + msg.angular.z*(wheelGeometryX[0] + wheelGeometryY[0])) / wheelRad * ticksPerRev);
	RightRear = (int32_t) -((msg.linear.y + msg.linear.x + msg.angular.z*(wheelGeometryX[1] - wheelGeometryY[1])) / wheelRad * ticksPerRev);
	LeftRear = (int32_t) ((msg.linear.y - msg.linear.x + msg.angular.z*(wheelGeometryX[2] + wheelGeometryY[2])) / wheelRad * ticksPerRev);	
	LeftFront = (int32_t) ((msg.linear.y + msg.linear.x + msg.angular.z*(wheelGeometryX[3] - wheelGeometryY[3])) / wheelRad * ticksPerRev);
	
//	printf("linear x: %f, y: %f, z: %f\nangular x: %f, y: %f, z: %f\n", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
//	printf("VA%ldB%ld\t", LeftRear, LeftFront);
//	printf("VA%ldB%ld\n", RightFront, RightRear);
	
	// Construct and send the command strings
	sprintf(TXbuf, "VA%ldB%ld", LeftRear, LeftFront);
	write(fdL, TXbuf, strlen(TXbuf));
	delayMicroseconds(TWIDelay);
	
	sprintf(TXbuf, "VA%ldB%ld", RightFront, RightRear);
	write(fdR, TXbuf, strlen(TXbuf));
	delayMicroseconds(TWIDelay);
}*/

void MotorController::printDebug()
{
	// -------- DEBGGING --------
		// Set controllers to output debig string
		//write(fdL, "?V", 2);
		//delayMicroseconds(TWIDelay);
		write(fp_i2cController, "?t", 2);	
		delayMicroseconds(TWIDelay);
		// Read from each controller
		//read(fdL, RXbuf1, 50);
		//delayMicroseconds(TWIDelay);
		read(fp_i2cController, RXbuf, 50);
		delayMicroseconds(TWIDelay);
		// Print the strings
		printf("%s\n", RXbuf);
}


int main(int argc, char** argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "motor_controller_node");
	
	// Create the motor controller object
	MotorController controller;
	ros::Rate loop_rate(10);
	
	
	
	while (ros::ok())
	{
		
		controller.printDebug();
		// Get raw encoder positions and publish them
		controller.PublishEncoders();
		// Get raw encoder velocities and publish them
		controller.PublishVelocities();
		
		// Spin then sleep
		ros::spinOnce();
		loop_rate.sleep();		
	}
}



















