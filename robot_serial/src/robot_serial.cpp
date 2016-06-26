/*
 * robot_serial.cpp
 *
 *  Created on: Aug 8, 2015
 *      Author: ratmcu
 */
//https://github.com/wjwwood/serial

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <math.h>
#include "serial/serial.h"

#define distancePerCount 0.00664
#define wheelGap 0.385
#define PI 3.14159
using namespace std;
void enumerate_ports();
void velmsgCB(const geometry_msgs::Twist::ConstPtr& msg);
serial::Serial robotserial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_serial");
  	ros::NodeHandle nt_serial;
  	ros::Time current_time, last_time;
  	ros::Rate lrate(100); //100 times a second
  	string dataString;
  	ros::Subscriber subteleop = nt_serial.subscribe<geometry_msgs::Twist>("cmd_vel",1,velmsgCB);
  	//ros::Subscriber subteleop = nt_serial.subscribe<geometry_msgs::Twist>("navigation_velocity_smoother/raw_cmd_vel",1,velmsgCB);
  	//^use velmsgCB to calculate L n R encoder counts and send them to propeller board.
  	
  	ros::Publisher odom_pub = nt_serial.advertise<nav_msgs::Odometry>("odom", 100);
  	tf::TransformBroadcaster odom_broadcaster;
  	
  	double leftEncoderCount = 0.0;
  	double rightEncoderCount = 0.0;
  	double leftEncoderCountOld = 0.0;
  	double rightEncoderCountOld = 0.0;
  	double dl = 0.0;
	double dx = 0.0;
	double dy = 0.0;
	double dth = 0.0;
	double dt = 0.0;
  	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	
	double odomValues[7];
	bool odomReady = false;

	//enumerate_ports();
	
	robotserial.flush();
	while(robotserial.available())dataString = robotserial.readline(100,"\n");
	//robotserial.write("d,0.387,0.00664,0.0,0.0,0.0\r"); // initialize robot  parameters trackWidth and distancepertick
	robotserial.write("d,0.403000,0.006760,0,0,0,0.0,0.0,0.0\r");
	
	while(nt_serial.ok())
	{	
		ros::spinOnce();  // check for incoming messages
		current_time = ros::Time::now();
	  //taking encoder counts from robot
		//while(!robotserial.available());
		if(robotserial.available())
		{
			dataString = robotserial.readline(100,"\n");
			cout << "odom received: "<< dataString << endl;
			char * dataStringArray = new char [dataString.length()+1];
			std::strcpy (dataStringArray, dataString.c_str());
		        //printf("%s\n",dataStringArray);
			// dataStringArray now contains a c-string copy of str
			char * token = std::strtok (dataStringArray,",");
			char *unconverted;
			int i = 0;
			
			if(*token=='o') 
			{
				while (token!=0)
				{
					//std::cout << token << "-" << atoi(token) <<'\n';
					odomValues[i] = atof(token);
					token = std::strtok(NULL,",");
		
					i++;
				}
				odomReady = true;
			}
			else if(*token=='d')
			{
				//cout <<  dataString << endl;	
			}
		}
		
		
		
	/*---------------------------------------------------------------------*/
	//calculate the odometry
	/*
		leftEncoderCount = odomValues[1];
		rightEncoderCount = odomValues[2];
		//cout << leftEncoderCount << ',' << rightEncoderCount << '\n';
		//cout << leftEncoderCountOld << ',' << rightEncoderCountOld << '\n';
		if(leftEncoderCount!=leftEncoderCountOld||rightEncoderCount!=rightEncoderCountOld)
		{
			dl = (double)((leftEncoderCount-leftEncoderCountOld)+(rightEncoderCount-rightEncoderCountOld))*distancePerCount/2;
			dt = (current_time.toSec()-last_time.toSec());
			dth = (double)((rightEncoderCount-rightEncoderCountOld)-(leftEncoderCount-leftEncoderCountOld))*distancePerCount/(0.39);
			leftEncoderCountOld = leftEncoderCount;
			rightEncoderCountOld = rightEncoderCount;
			last_time = current_time;
			
			th += dth;
			
			if (th > PI) 
			{
				th -= 2.0 * PI;
			} 
			else 
			{
				if (th <= -PI) 
				{
				    th += 2.0 * PI;
				}
			}
		
			dx = dl*cos(th);
			dy = dl*sin(th);
			x += dx;
			y += dy;
			vx = dx/dt;
			vy = dy/dt;
			vth = dth/dt;
			
		}
		else
		if((current_time.toSec()-last_time.toSec())>.5)	
		{
			dl = 0.0;
			dth = 0.0;
			vx = 0.0;
			vy = 0.0;
			vth = 0.0;
		}
		*/
		if(odomReady == true)
		{
			x = odomValues[1];
			y = odomValues[2];
			th = odomValues[3];
			vx = odomValues[4]*cos(odomValues[3]);
			vy = odomValues[4]*sin(odomValues[3]);
			vth = odomValues[5];
		
			//print(term, "o,%.3f,%.3f,%.3f,%.3f,%.3f\n", X, Y, Heading, V, Omega);
			//cout << dl/dt << ',' << vx<< ',' << vy<< ',' << vth << ','<< x << ','<< y << ',' << th <<'\n';
			//cout << dl/dt <<  ',' << vth << '\n';

			//since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			//set the velocity
			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = vth;

			//publish the message
			odom_pub.publish(odom);
			odomReady = false;
		}
		
		
		lrate.sleep();
	}
	return 0;
}


void enumerate_ports()
{
	std::vector<serial::PortInfo> devices_found = serial::list_ports();

	std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(Port: %s, Description: %s, Hardware ID: %s)\n", device.port.c_str(), device.description.c_str(),
				device.hardware_id.c_str() );
	}
}

void velmsgCB(const geometry_msgs::Twist::ConstPtr& msg)
{
	std::stringstream buffer;
	buffer << "s" << "," << msg->linear.x << "," << msg->angular.z << "\r" ;
	robotserial.write(buffer.str());
}
	
