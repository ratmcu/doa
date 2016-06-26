#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "doa_service/DoaRssi.h"
#include "math.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
/* A simple server in the internet domain using TCP
   The port number is passed as an argument
   This version runs forever, forking off a separate
   process for each connection
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "serial/serial.h"


//#define SDR


float phi = 1.5634;
float rssi = 0.3215;

#ifdef SDR
serial::Serial robotserial("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000));
#endif

bool add(doa_service::DoaRssi::Request  &req,
		 doa_service::DoaRssi::Response &res)
{
  //get_sock_data();
  res.doa = phi;
  res.rssi = rssi;
  ROS_INFO("request: %ld", req.req);
  ROS_INFO("sending back response: DOA: [%f], RSSI: [%f]", res.doa,res.rssi);;
  return true;
}

using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "doa_rssi_server");
  ros::NodeHandle n2;
  tf::TransformListener listener;
  ros::Rate r(10);
  ros::ServiceServer service = n2.advertiseService("get_doa_rssi", add);
  ros::Publisher marker_pub_path = n2.advertise<visualization_msgs::Marker>("beacon_point", 10);
  tf::StampedTransform transformMtoR;
  tf::StampedTransform transform;
  string dataString;
  float doarssi[3];
  bool doarssiready = false;
  //ros::spin();//a blocking call duhh!
  ROS_INFO("ready to give DOA and RSSI!!");

   visualization_msgs::Marker markerDOA,markerIntersections,markerBeacon1,markerBeacon2,markerPath;
   markerDOA.header.frame_id  = "/map";
   markerDOA.header.stamp = ros::Time::now();
   markerDOA.ns = "markers";
   markerDOA.action   = visualization_msgs::Marker::ADD;
   markerDOA.pose.orientation.w = 1.0;
   markerDOA.id = 0;
   markerDOA.type = visualization_msgs::Marker::POINTS;
 //########POINTS markers use x and y scale for width/height respectively
   markerDOA.scale.x = 1.2;
   markerDOA.scale.y = 1.2;
   markerDOA.scale.z = 0.5;
   markerDOA.color.r = 1.0f;
   markerDOA.color.b = 1.0f;
   markerDOA.color.a = 1.0;
   geometry_msgs::Point marker_point;
   marker_point.x = 19.0;
   marker_point.y = 19.0;
   marker_point.z = 0.0;
   markerDOA.points.push_back(marker_point);
   marker_pub_path.publish(markerDOA);
  while(n2.ok())
  {
#ifndef SDR
	  try{
		 listener.lookupTransform("base_link", "beacon",
								 ros::Time(0), transform);
	  }
	  catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		continue;
	  }
	  phi = atan2f(transform.getOrigin().y(),transform.getOrigin().x());
	  rssi = 1/(transform.getOrigin().x()*transform.getOrigin().x()+transform.getOrigin().y()*transform.getOrigin().y());
#else
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
		if(*token=='t')
		{
			while (token!=0)
			{
				//std::cout << token << "-" << atoi(token) <<'\n';
				doarssi[i] = atof(token);
				token = std::strtok(NULL,",");
				i++;
			}
			doarssiready = true;
		}
	}
#endif
	if(doarssiready==true)
	{
		phi = doarssi[1];
		rssi = doarssi[2];
		doarssiready = false;
	}
	ros::spinOnce();
	marker_pub_path.publish(markerDOA);
  }
  return 0;
}
