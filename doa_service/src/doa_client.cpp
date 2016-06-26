#include "ros/ros.h"
#include "doa_service/DoaRssi.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "doa_rssi_client");
//  if (argc != 2)
//  {
//    ROS_INFO("usage: just an integer");
//    return 1;
//  }

  ros::NodeHandle n;
  ros::Rate r(10);
  ros::ServiceClient client;
  while(!ros::service::waitForService("get_doa_rssi", ros::Duration(3.0))) {
         ROS_ERROR("Service doa_rssi not available - waiting.");
  }
  client = n.serviceClient<doa_service::DoaRssi>("get_doa_rssi");
  doa_service::DoaRssi srv;
  srv.request.req = 55 ;
  while(n.ok())
  {
	  ros::spinOnce();
	  if (client.call(srv))
	  {
		ROS_INFO("DOA: %f  RSSI: %f", srv.response.doa,srv.response.rssi);
	  }
	  else
	  {
		ROS_ERROR("Failed to call service get_doa_n_rssi");
		return 1;
	  }
	  r.sleep();
  }

  return 0;
}
