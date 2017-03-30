#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define SERIAL_DELAY_US 3000
#define MAX_RX_BUF 50


class MotorController
{
	public:
		MotorController();
		void PublishEncoders();
		void PublishVelocities();
		void printDebug();
		void setVelocities();
		void publishOdometry();
	
	private:
		bool pollEncoders();
	
		// Serial transfer buffers
		char txBuf[50];
		char rxBufL[MAX_RX_BUF];
		char rxBufR[MAX_RX_BUF];
		int TWIDelay;
		// Motor controller Serial handle
		int hserial;

		// Ros nodehandle. Main access point for all ROS comms
		ros::NodeHandle nh;
		
		// Message objects
		std_msgs::String test_msg;
		geometry_msgs::Twist vel_msg;
		geometry_msgs::Point wheel_vel_msg;
		std_msgs::Int32 left_encoder_pos_msg; 
		std_msgs::Int32 right_encoder_pos_msg;
		nav_msgs::Odometry odom;
		geometry_msgs::TransformStamped odom_trans;
		geometry_msgs::Quaternion odom_quat;
		
		float L_motor;
		float R_motor;
		
		// Odometry tracking (maybe just directly use the odom struct in future. But cna I roate the quaternion as easy as +=?)
		int32_t rightEnc;
		int32_t rightEnc_old;
		int32_t leftEnc;
		int32_t leftEnc_old;
		bool odomReady; // Have we ready two encoder positions since the node has been running?
		double x;
		double y;
		double th;
		ros::Time current_time, last_time;
		
		// Pubs
		ros::Publisher testPub;
		ros::Publisher LencoderPub;
		ros::Publisher RencoderPub;		
		ros::Publisher velPub;
		ros::Publisher odom_pub;
		tf::TransformBroadcaster odom_broadcaster;
		
		
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


bool MotorController::pollEncoders()
{
	// Request position from 0 motor
	serialFlush(hserial);
	serialPrintf(hserial, "0p\r");
	delayMicroseconds(SERIAL_DELAY_US);
	
	int newChar;
	// Reset the RX buffer
	int rxCount = 0;
	rxBufL[0] = '\0';
	if ((newChar = serialGetchar(hserial)) != -1)
	{
		delay(3);
		rxBufL[rxCount++] = (char) newChar;
		while ( (serialDataAvail(hserial) > 0) & (rxCount < MAX_RX_BUF-2)){
			newChar = serialGetchar(hserial);	
			delay(3);
			rxBufL[rxCount++] = (char) newChar;
		}
		rxBufL[rxCount] = '\0';
	}	
	
	// Request position from 1 motor
	serialFlush(hserial);
	serialPrintf(hserial, "1p\r");
	delayMicroseconds(SERIAL_DELAY_US);
	
	// Reset the RX buffer 
	rxCount = 0;
	rxBufR[0] = '\0';
	if ((newChar = serialGetchar(hserial)) != -1)
	{
		delay(3);
		rxBufR[rxCount++] = (char) newChar;
		while ((serialDataAvail(hserial) > 0) & (rxCount < MAX_RX_BUF-2)){
			newChar = serialGetchar(hserial);	
			delay(3);
			rxBufR[rxCount++] = (char) newChar;
		}
		rxBufR[rxCount] = '\0';
	}
	
	// Try to pull data out of string. If it succeeds, we have new data to publish
	if ((sscanf(rxBufL, "0p%d", &(leftEnc)) == 1) & (sscanf(rxBufR, "1p%d", &(rightEnc)) == 1))
		// Return true only if we have already read some encoders so far this node.
		if (odomReady)
			return true;
		else
		{
			odomReady = true;
			leftEnc_old = leftEnc;
			rightEnc_old = rightEnc;
		}
	else
		return false;
}

MotorController::MotorController()
{
	// Initialise geometry
	// Left wheel is in +ve X, right is -ve
	wheelGeometryX[0] = 90; // mm
	wheelGeometryX[1] = -90; // mm
	
	wheelGeometryY[0] = 90; // mm
	wheelGeometryY[1] = -90; // mm
	
	wheelRad = 50; // mm
	ticksPerRev = 768; // encoder ticks per revolution
	
	// Motor Velocities in encoder ticks/sec
	L_motor = 0;
	R_motor = 0;
	
	// Initialise odometry
	rightEnc = 0;
	rightEnc_old = 0;
	leftEnc = 0;
	leftEnc_old = 0;
	odomReady = false;
	x = 0.0;
	y = 0.0;
	th = 0.0;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	// Subscribe to the command topics
	direct_cmd = nh.subscribe<std_msgs::String>("direct_cmd", 100, &MotorController::cmd_callback, this);
	set_vel = nh.subscribe<geometry_msgs::Twist>("set_vel", 100, &MotorController::set_vel_callback, this);
	//twist_vel = nh.subscribe<geometry_msgs::Twist>("twist_set", 100, &MotorController::set_twist_callback, this);
	
	// -------- Create the publishers --------
	testPub = nh.advertise<std_msgs::String>("test_pub", 100);
	LencoderPub = nh.advertise<std_msgs::Int32>("L_encoder_pos", 100);
	RencoderPub = nh.advertise<std_msgs::Int32>("R_encoder_pos", 100);	
	velPub = nh.advertise<geometry_msgs::Point>("wheel_velocities", 100);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	
	// -------- Set up wiringPi --------
	wiringPiSetupSys();
	if ( (hserial = serialOpen(SERIAL_PORT, 115200)) == -1)
	{
		printf("Failed to connect to device - Exiting\n");
	}
}

void MotorController::PublishEncoders()
{
	
	
	//printf("raw Lpos(%d) = %s\tRpos(%d) = %s\n", tempCount, rxBufL, rxCount, rxBufR);
	// Pull data from the motor zero string and publish
	//if (sscanf(rxBufL, "0p%d", &(left_encoder_pos_msg.data)) == 1) LencoderPub.publish(left_encoder_pos_msg);
	// Pull data from the motor one string  and publish
	//if (sscanf(rxBufR, "1p%d", &(right_encoder_pos_msg.data)) == 1) RencoderPub.publish(right_encoder_pos_msg);
	
	// Calculate dx, dy, dth.
	
	// Check if the wheels traveled 
	

	
}

void MotorController::PublishVelocities()
{
	
	// Request velocity from 0 motor
	serialFlush(hserial);
	serialPrintf(hserial, "0v\r");
	delayMicroseconds(SERIAL_DELAY_US);
	
	int newChar;
	int rxCount = 0;
	rxBufL[0] = '\0';
	if ((newChar = serialGetchar(hserial)) != -1)
	{
		delay(3);
		rxBufL[rxCount++] = (char) newChar;
		while ((serialDataAvail(hserial) > 0) & (rxCount < MAX_RX_BUF-2)){
			newChar = serialGetchar(hserial);	
			delay(3);
			rxBufL[rxCount++] = (char) newChar;
		}
		rxBufL[rxCount] = '\0';
	}	
	int tempCount = rxCount;

	// Request velocity from 1 motor
	serialFlush(hserial);
	serialPrintf(hserial, "1v\r");
	delayMicroseconds(SERIAL_DELAY_US);
	
	rxCount = 0;
	rxBufR[0] = '\0';
	if ((newChar = serialGetchar(hserial)) != -1)
	{
		delay(3);
		rxBufR[rxCount++] = (char) newChar;
		while ((serialDataAvail(hserial) > 0) & (rxCount < MAX_RX_BUF-2)){
			newChar = serialGetchar(hserial);	
			delay(3);
			rxBufR[rxCount++] = (char) newChar;
		}
		rxBufR[rxCount] = '\0';
	}	
	
	//printf("raw Lvel(%d) = %s\tRvel(%d) = %s\n", tempCount, rxBufL, rxCount, rxBufR);
	// Pull data from the motor strings. If successful, publish.
	if ((sscanf(rxBufL, "0v%lf", &(wheel_vel_msg.x)) == 1) & (sscanf(rxBufR, "1v%lf", &(wheel_vel_msg.y)) == 1))
	{
		velPub.publish(wheel_vel_msg);
	}
	
}


void MotorController::cmd_callback(const std_msgs::String::ConstPtr& msg)
{
	serialFlush(hserial);
	serialPuts(hserial, msg->data.c_str());
	delayMicroseconds(SERIAL_DELAY_US);
	ROS_INFO("Command sent to Controller: [%s]", msg->data.c_str());
}

/*
 *	Set instantaneous tangental velocity (twist.linear.x) and angular velocity (twist.anguilar.z) in m/s
 *  Input is m/s and rad/s, must convert that to encoder ticks/sec.
 */
void MotorController::set_vel_callback(const geometry_msgs::Twist msg)
{
	// Construct and send the command strings
	
	L_motor = (float) ((msg.linear.x*1000 - wheelGeometryX[0]*sin(msg.angular.z)) / (2*3.1415*wheelRad) * ticksPerRev);
	R_motor = (float) ((msg.linear.x*1000 - wheelGeometryX[1]*sin(msg.angular.z)) / (2*3.1415*wheelRad) * ticksPerRev);
	//printf("saw calc = %lf\twz = %lf\tLmot = %f\tRmot = %f\n", wheelGeometryX[0]*sin(msg.angular.z), msg.angular.z, L_motor, R_motor);
	
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
		//write(fp_i2cController, "?t", 2);	
		//delayMicroseconds(TWIDelay);
		// Read from each controller
		//read(fdL, RXbuf1, 50);
		//delayMicroseconds(TWIDelay);
		//read(fp_i2cController, RXbuf, 50);
		//delayMicroseconds(TWIDelay);
		// Print the strings
		//printf("%s\n", RXbuf);
}

void MotorController::setVelocities()
{
	//printf("Setting L_motor = %f\tR_motor = %f\n", L_motor, R_motor);
	serialFlush(hserial);
	serialPrintf(hserial, "0V%f\r", L_motor);
	delayMicroseconds(SERIAL_DELAY_US);
	serialFlush(hserial);
	serialPrintf(hserial, "1V%f\r", R_motor);
	delayMicroseconds(SERIAL_DELAY_US);
}


// --------
// Just using the example from http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
		
// --------
void MotorController::publishOdometry()
{
	// Poll the encoders, if we get data then publish it
	if (pollEncoders() | 1)
	{
		// Publish raw encoder data (for debugging)
		left_encoder_pos_msg.data = leftEnc;
		right_encoder_pos_msg.data = rightEnc;
		LencoderPub.publish(left_encoder_pos_msg);
		RencoderPub.publish(right_encoder_pos_msg);
		
		// Calculate total distance and angle travelled. 
		// NOTE THIS IS ONLY VALID FOR VERY SMALL MOVEMENTS!!!!!
		// RELIES ON sin(th) == th for very small th
		// ((2*pi*r*d_enc1/tick per rev) + (2*pi*r*d_enc2/tick per rev)) /2 
		double d_L = 2*3.1415*wheelRad/1000*(leftEnc-leftEnc_old)/ticksPerRev;
		double d_R = 2*3.1415*wheelRad/1000*(rightEnc-rightEnc_old)/ticksPerRev;
		double d = (d_L + d_R)/2; //3.1415*wheelRad/1000*((leftEnc-leftEnc_old) + (rightEnc-rightEnc_old))/ticksPerRev;
		double dth = (d_R - d_L)/(fabs(wheelGeometryX[0] - wheelGeometryX[1])/1000);
		
		// Update x y and th positions
		th += dth;
		double dx = -d*sin(dth); // These are dx and dy in the robot local frame
		double dy =  d*cos(dth);	// Need to convert them to odom
		
		// x and y in odom
		x += dx*cos(th) - dy*sin(th);
		y += dx*sin(th) + dy*cos(th);
		
		// Get the current time and calculate dt
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		last_time = current_time;
		
		// Generate the transform to publish over tf
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		// Populate the transform using the calculated odometry data
		odom_quat = tf::createQuaternionMsgFromYaw(th);
		odom_trans.transform.rotation = odom_quat;
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		// And send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		// Now we can send the odometry information to ros
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		// Fill in pose data - it is the same as the transform in this case.
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		// Calculate and fill in velocity data
		odom.twist.twist.linear.x = dx / dt;
		odom.twist.twist.linear.y = dy / dt;
		odom.twist.twist.angular.z = th / dt;
		// And publish
		odom_pub.publish(odom);
		
		leftEnc_old = leftEnc;
		rightEnc_old = rightEnc;
		
		
	}
/*
	  double vx = 0.1;
	  double vy = -0.1;
	  double vth = 0.1;
  */
	//current_time = ros::Time::now();
	//double dt = (current_time - last_time).toSec();
	
	/*
    x += (0.1 * cos(th) - 0.1 * sin(th)) * dt;
    y += (0.1 * sin(th) + 0.1 * cos(th)) * dt;
    th += 0.1 * dt;*/
	
	    /*x += delta_x;
    y += delta_y;
    th += delta_th;*/
	
	//x += (0.1*cos(th) + 0.1*sin(th))*dt;
	//y += (0.1*sin(th) - 0.1*cos(th))*dt;
	//th += 0.1*dt;
	//odom_quat = tf::createQuaternionMsgFromYaw(th);
	//odom_trans.header.stamp = current_time;
	//odom_trans.header.frame_id = "odom";
	//odom_trans.child_frame_id = "base_link";
		
	//odom_trans.transform.translation.x = x;
	//odom_trans.transform.translation.y = y;
	//odom_trans.transform.translation.z = 0.0;
	//odom_trans.transform.rotation = odom_quat;
	//send the transform
	//odom_broadcaster.sendTransform(odom_trans);
	
	//odom.header.stamp = current_time;
	//odom.header.frame_id = "odom";
	//set the position
	//odom.pose.pose.position.x = x;
	//odom.pose.pose.position.y = y;
	//odom.pose.pose.position.z = 0.0;
	//odom.pose.pose.orientation = odom_quat;
	//set the velocity
	//odom.child_frame_id = "base_link";
	//odom.twist.twist.linear.x = 0.1;
	//odom.twist.twist.linear.y = 0.1;
	//odom.twist.twist.angular.z = 0.1;
	
	//odom_pub.publish(odom);
	//last_time = current_time;
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
		// Set the velocities 
		controller.setVelocities();
		// publish odometry
		controller.publishOdometry();
		
	
		
		// Spin then sleep
		ros::spinOnce();
		loop_rate.sleep();		
	}
	printf ("Exiting\n");
}



















