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
	
	private:
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
		
		float L_motor;
		float R_motor;
		
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
	// Initialise geometry
	
	wheelGeometryX[0] = 0;
	wheelGeometryX[1] = 0;
	// Left wheel is in +ve Y, right is -ve
	wheelGeometryY[0] = 90;
	wheelGeometryY[1] = -90;
	
	wheelRad = 50;
	ticksPerRev = 768;
	
	L_motor = 0;
	R_motor = 0;
	
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
	if ( (hserial = serialOpen(SERIAL_PORT, 9600)) == -1)
	{
		printf("Failed to connect to device - Exiting\n");
	}
}

void MotorController::PublishEncoders()
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
	
	//printf("raw Lpos(%d) = %s\tRpos(%d) = %s\n", tempCount, rxBufL, rxCount, rxBufR);
	// Pull data from the motor zero string and publish
	if (sscanf(rxBufL, "0p%d", &(left_encoder_pos_msg.data)) == 1) LencoderPub.publish(left_encoder_pos_msg);
	// Pull data from the motor one string  and publish
	if (sscanf(rxBufR, "1p%d", &(right_encoder_pos_msg.data)) == 1) RencoderPub.publish(right_encoder_pos_msg);

	
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
	L_motor = (float) -((msg.linear.x*1000 - wheelGeometryX[0]*sin(msg.angular.z)) / (2*3.1415*wheelRad) * ticksPerRev);
	R_motor = (float) ((msg.linear.x*1000 - wheelGeometryX[1]*sin(msg.angular.z)) / (2*3.1415*wheelRad) * ticksPerRev);
	
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
		
		// Spin then sleep
		ros::spinOnce();
		loop_rate.sleep();		
	}
	printf ("Exiting\n");
}



















