#include <ros/ros.h>
// always remember to add these dependencies in the file CMakeLists.txt #
//find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs move_base_msgs tf costmap_2d)#
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Pose2D.h>
#define forEach BOOST_FOREACH
#include <math.h>
#include <doa_service/DoaRssi.h>

const float d = 0.8;
const int   DOA_N = 6;
const int   MAX_NDOA = 50;
const float MAX_RSSI = 1.0;
const float alpha = 10*M_PI/180;
float get_heading(float,float,float phi,float);
int8_t get_map_cost(float x, float y);
void get_DOA_intersections(const std::deque<geometry_msgs::Pose2D> &localizationData,std::deque<geometry_msgs::Pose2D> &intersectionPoints);
bool sufficient_DOAs(const std::deque<geometry_msgs::Pose2D> &localizationData);
void get_beacon_estimate(const std::deque<geometry_msgs::Pose2D> &intersectionPoints, geometry_msgs::Pose2D &coordinate);
void get_step_on_plan(const nav_msgs::GetPlan &Plan,const tf::StampedTransform &transformMapToRobot,float &ggx,float &ggy);
bool get_safe_near_point(float &x,float &y);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
nav_msgs::OccupancyGrid globalGrid;
map_msgs::OccupancyGridUpdate globalGridUpdate;
int mapMetrix[5000][5000];

class robot_controller
{
	public:
 	 MoveBaseClient MoveBaseController;
 	 move_base_msgs::MoveBaseGoal goal;
	 robot_controller();
	 ~robot_controller();
	 void move_step(float *heading);
	 void go_to_goal(const geometry_msgs::PoseStamped &poseStamped);
 	private:
	 geometry_msgs::Quaternion goalQuat;
	 tf::Quaternion goalQuatTf;
};
robot_controller::robot_controller():  MoveBaseController("move_base",true)
{
	while(!MoveBaseController.waitForServer(ros::Duration(5.0)))
	{
	      ROS_INFO("Waiting for the move_base action server to come up");
	}
	ROS_INFO("move_base action server is running");
}
robot_controller::~robot_controller(){};
void robot_controller::move_step(float *heading)
{
		  goal.target_pose.header.frame_id = "base_link";
		  goal.target_pose.header.stamp   = ros::Time::now();
		  goal.target_pose.pose.position.x = d*cos(*heading);
		  goal.target_pose.pose.position.y = d*sin(*heading);
		  goalQuatTf.setRPY(0,0,*heading);
		  tf::quaternionTFToMsg(goalQuatTf,goalQuat);
		  goal.target_pose.pose.orientation = goalQuat;
		  ROS_INFO("Sending a step to move");
		  MoveBaseController.sendGoal(goal);
}

void robot_controller::go_to_goal(const geometry_msgs::PoseStamped &poseStamped)
{
		 goal.target_pose.header.frame_id = "map";
		 goal.target_pose.header.stamp   = ros::Time::now();
		 goal.target_pose.pose = poseStamped.pose;
		 ROS_INFO("Sending a goal to move");
		 MoveBaseController.sendGoal(goal);
}

void globalCostmapmsgCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	globalGrid = *msg;
//	ROS_INFO("got map %d x %d @ (%d,%d) with %d",globalGrid.info.height,globalGrid.info.width,
//			(int)globalGrid.info.origin.position.x,(int)globalGrid.info.origin.position.y,(int)globalGrid.info.resolution);
	for(int i=0;i<globalGrid.info.height;i++)
		for(int j=0;j<globalGrid.info.width;j++)
		{
			mapMetrix[j][i] = globalGrid.data[j+i*400];
		}
}
void globalCostmapUpdatemsgCB(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
	globalGridUpdate = *msg;
	ROS_INFO("got map update %d x %d @ (%d,%d)",globalGridUpdate.height,globalGridUpdate.width,
			(int)globalGridUpdate.x,(int)globalGridUpdate.y);
	for(int i=0;i<globalGridUpdate.height;i++)
			for(int j=0;j<globalGridUpdate.width;j++)
			{
				mapMetrix[j+(int)globalGridUpdate.x][i+(int)globalGridUpdate.y] = globalGridUpdate.data[j+i*globalGridUpdate.width];
			}
}
enum state_t
{
	START=0,
	SEEK,
	FOLLOW_PLAN,
	DOA_LINE,
	SUCCESS,
	ERROR
};
int main(int argc, char** argv){

  ros::init(argc, argv, "robot_brain");
  ros::NodeHandle node;
  ros::Rate r(10);

///################publishers and subscribers###############################///////////////////////
  ros::Publisher marker_pub_path = node.advertise<visualization_msgs::Marker>("visualization_marker_path", 10);
  ros::Publisher marker_pub_intersections = node.advertise<visualization_msgs::Marker>("visualization_marker_intersections", 10);
  ros::Publisher marker_pub_doa = node.advertise<visualization_msgs::Marker>("visualization_marker_doa", 10);
  ros::Subscriber subOccupancyGridGlobal = node.subscribe<nav_msgs::OccupancyGrid>("move_base/global_costmap/costmap",1,globalCostmapmsgCB);
  ros::Subscriber subOccupancyGridGlobalUpdate = node.subscribe<map_msgs::OccupancyGridUpdate>("move_base/global_costmap/costmap_updates",1,globalCostmapUpdatemsgCB);
///endof publishers and subscribers/////////////////////

//########visualization stuff####################?/////////////////
  visualization_msgs::Marker markerDOA,markerIntersections,markerBeacon1,markerBeacon2,markerPath;
  markerDOA.header.frame_id  = "/map";
  markerDOA.header.stamp = ros::Time::now();
  markerDOA.ns = "markers";
  markerDOA.action   = visualization_msgs::Marker::ADD;
  markerDOA.pose.orientation.w = 1.0;
  markerDOA.id = 0;
  markerDOA.type = visualization_msgs::Marker::POINTS;
//########POINTS markers use x and y scale for width/height respectively
  markerDOA.scale.x = 0.08;
  markerDOA.scale.y = 0.08;
  markerDOA.scale.z = 0.08;
  markerDOA.color.r = 1.0f;
  markerDOA.color.a = 1.0;
  geometry_msgs::Point marker_point;
  markerIntersections = markerBeacon1 = markerPath = markerDOA;
  markerIntersections.color.r = 0.0f;
  markerIntersections.color.g = 1.0f;
  markerBeacon1.scale.x = 0.12;
  markerBeacon1.scale.y = 0.12;
  markerBeacon1.scale.z = 0.12;
  markerBeacon2 = markerBeacon1;
  markerBeacon1.color.r = 0.0f;
  markerBeacon1.color.b = 1.0f;
  markerPath.color.b = 0.25f;
  markerPath.color.g = 0.5f;
 //#########End of visualization stuff######################/////////////////////

 //#########planner service members#########################/////////////////////
  ros::ServiceClient moveBaseClient;
  while(!ros::service::waitForService("move_base/make_plan", ros::Duration(3.0))) {
       ROS_ERROR("Service move_base/make_plan not available - waiting.");
  }
  moveBaseClient = node.serviceClient<nav_msgs::GetPlan>("move_base/make_plan", true);
  nav_msgs::GetPlan srvPlan;
 //#########end of planner service members###############////////////////////////

 //#########get_doa_rssi service members#################////////////////////////
  ros::ServiceClient doaClient;
    while(!ros::service::waitForService("doa_service/get_doa_rssi", ros::Duration(3.0))) {
           ROS_ERROR("Service doa_rssi not available - waiting.");
    }
    doaClient = node.serviceClient<doa_service::DoaRssi>("doa_service/get_doa_rssi");
    doa_service::DoaRssi srvDoaRssi;
  //#########get_doa_rssi service members###############////////////////////////

  //marker_pub.publish(point);

  robot_controller controller;

  //variables for taking DOA and map positions
  float phi,heading,oldHeading,rssi;
  int beaconPointCount = 0;
  std::deque<geometry_msgs::Pose2D> localizationData;
  std::deque<geometry_msgs::Pose2D> intersectionPoints;
  geometry_msgs::Pose2D localizaionPoint;
  geometry_msgs::Pose2D oldBeaconPoint;
  geometry_msgs::Pose2D newBeaconPoint;
  //##########state machine variables////////////

  state_t state = START;

  bool doaValid = false;
  bool tfValid = false;
  bool goodEstimation = false;
  bool takeDoaRssi =  false;
  //######### transform objects########################//////////////
  tf::TransformListener tfListener;
  tf::StampedTransform transformMapToRobot;

  geometry_msgs::PoseStamped initialGoal;
  initialGoal.pose.position.x = 20.0;
  initialGoal.pose.position.y = 11.0;
  initialGoal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  controller.go_to_goal(initialGoal);
  controller.MoveBaseController.waitForResult();
  if(controller.MoveBaseController.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
		  ROS_INFO("Proceeding to the state machine");
  }

  while(node.ok())
  {
	  ros::spinOnce();

	  ROS_INFO("state: %d",state);
	  /*float xxx,yyy;
	  xxx = 12.0;
	  yyy = 12.0;
	  get_safe_near_point(xxx,yyy);
	  ROS_INFO("safe near point @(%f,%f)",xxx,yyy);*/
	  switch(state)
	  {
	  	  case START:
	  	  {
			  localizationData.clear();
			  intersectionPoints.clear();
			  markerDOA.points.clear();
			  markerBeacon1.points.clear();
			  markerBeacon2.points.clear();
			  markerPath.points.clear();

			  takeDoaRssi = true;
			  if(rssi<MAX_RSSI)
				  state = SEEK;
	  	  }break;
		  case SEEK:
		  {
			  if(doaValid==true && tfValid==true)
			  {
				  heading = get_heading(transformMapToRobot.getOrigin().x(),transformMapToRobot.getOrigin().y(),phi,tf::getYaw(transformMapToRobot.getRotation()));
				  get_DOA_intersections(localizationData,intersectionPoints);
				  ROS_INFO("delta headings:%f",fabs(heading-oldHeading));
				  if(intersectionPoints.size()>0)
				  {
					  state = FOLLOW_PLAN;
					  takeDoaRssi = false;
				  }
				  else if((fabs(heading-oldHeading)>6.0))
				  {
					  ROS_INFO("robot went to and fro!!");
					  state = DOA_LINE;
					  takeDoaRssi = false;
				  }
				  else
				  {
					  heading = get_heading(transformMapToRobot.getOrigin().x(),transformMapToRobot.getOrigin().y(),phi,tf::getYaw(transformMapToRobot.getRotation()));
					  controller.move_step(&heading);
					  oldHeading = heading;
					  controller.MoveBaseController.waitForResult();
					  if(controller.MoveBaseController.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
						 ROS_INFO("on course!!");
					  else
						 ROS_INFO("changing course");
					  takeDoaRssi = true;
				  }
			  }
			  if(rssi>MAX_RSSI)
			  	 state = SUCCESS;
		  }break;
		  case FOLLOW_PLAN:
		  {
			  //enough DOA points?? <--decided by checking the angle between 1st and last DOA
				  //if estimations are different then check for the uncertainty flag if so take new estimation
			  //use points on plan to move "50cm"???????**
			  if(rssi<MAX_RSSI)
			  {

				  //estimate region

				  get_DOA_intersections(localizationData,intersectionPoints);
				  get_beacon_estimate(intersectionPoints,newBeaconPoint);
				  ROS_INFO("estimated beacon location @(%f,%f)",newBeaconPoint.x,newBeaconPoint.y);
				  //try to get a plan >> may fail but try somehow to reach near
				  srvPlan.request.start.header.frame_id = "/map";
				  srvPlan.request.start.pose.position.x = transformMapToRobot.getOrigin().x();
				  srvPlan.request.start.pose.position.y = transformMapToRobot.getOrigin().y();
				  srvPlan.request.goal.header.frame_id = "/map";
				  srvPlan.request.goal.pose.position.x = newBeaconPoint.x;
				  srvPlan.request.goal.pose.position.y = newBeaconPoint.y;
				  srvPlan.request.tolerance  = 0.5;
				  if (moveBaseClient.call(srvPlan))
				  {
					  markerPath.points.clear();
					  if(!srvPlan.response.plan.poses.empty())
					  {
						   geometry_msgs::PoseStamped lastPose = srvPlan.response.plan.poses[0];
						   forEach(const geometry_msgs::PoseStamped & ptr, srvPlan.response.plan.poses)
						   {
									 //ROS_INFO("x=%f y=%f ",ptr.pose.position.x,ptr.pose.position.y);
							   marker_point.x = ptr.pose.position.x;
							   marker_point.y = ptr.pose.position.y;
							   marker_point.z = 0.0;
							   markerPath.points.push_back(marker_point);
						   }
						   //get 50cm apart point on the path plan
						   //give the goal to reach
						   //get_step_on_plan(srvPlan,transformMapToRobot,gx,gy);
						   if(srvPlan.response.plan.poses.size()>100)
						   {
							   controller.go_to_goal(srvPlan.response.plan.poses[100]);
						   }
						   else
						   {
							   controller.go_to_goal(srvPlan.response.plan.poses[srvPlan.response.plan.poses.size()-1]);
						   }
						   controller.MoveBaseController.waitForResult();
						   if(controller.MoveBaseController.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
						   {
								  ROS_INFO("moving on the planned path");
								  takeDoaRssi =  true;
						   }
					  }
					  else
					  {
						  state =  ERROR;
						  ROS_INFO("beacon way is obstructed");
					  }
				  }
				  else
				  {
					 ROS_ERROR("Failed to call planner service");
				  }
			  }
			  else
			  {
				  ROS_INFO("reached the beacon");
				  state = SUCCESS;
			  }
		  }break;
		  case DOA_LINE:
		  {
			  //try moving in the same heading till you have enough intersections
			  state = ERROR;

		  }break;
		  case SUCCESS:
		  {
			  if(rssi<MAX_RSSI)
				  state = START;
			  else
				  ROS_INFO("waiting for the beacon to appear far");
			  oldHeading = heading;
			  takeDoaRssi =  true;
		  }break;
		  case ERROR:
		  {
			  ROS_INFO("life is unfair!! and your code SUCKS!!");
			  state = START;
		  }break;
		  default:
		  {
			  state = START;
		  }
	  }///end of switch

	  if(takeDoaRssi == true)
	  {
			  srvDoaRssi.request.req = 77;				  //update the phi
			  if (doaClient.call(srvDoaRssi))
			  {
				  ROS_INFO("DOA: %f  RSSI: %f", srvDoaRssi.response.doa,srvDoaRssi.response.rssi);
				  phi = srvDoaRssi.response.doa;
				  rssi = srvDoaRssi.response.rssi;
				  doaValid = true;
				  oldHeading = 0.0;
			  }
			  else
			  {
				  ROS_ERROR("Failed to call service get_doa_n_rssi");
				  doaValid = false;
			  }
			  //get robot coordinates
			  try{
				tfListener.lookupTransform("map", "base_link",
										 ros::Time(0), transformMapToRobot);
				tfValid = true;
			  }
			  catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				tfValid = false;
				continue;
			  }
			  //update DOA dequeue
			  if(doaValid==true && tfValid==true)
			  {
				  localizaionPoint.x = transformMapToRobot.getOrigin().x();
				  localizaionPoint.y = transformMapToRobot.getOrigin().y();
				  localizaionPoint.theta = phi + tf::getYaw(transformMapToRobot.getRotation());
				 // ROS_INFO("localization point @(%f,%f) bearing = %f",localizaionPoint.x,localizaionPoint.y,localizaionPoint.theta);

				  if(beaconPointCount==MAX_NDOA)
				  {
					  localizationData.pop_front();
					  localizationData.push_back(localizaionPoint);
				  }
				  else
				  {
					  localizationData.push_back(localizaionPoint);
					  beaconPointCount++;
				  }
				  marker_point.x = localizaionPoint.x;
				  marker_point.y = localizaionPoint.y;
				  marker_point.z = 0.0;
				  markerDOA.points.push_back(marker_point);
				  takeDoaRssi =  false;
			  }
			  for (unsigned int i=0; i<intersectionPoints.size(); i++)
			  {
				  if(intersectionPoints.at(i).theta==0.0)
				  {
					  marker_point.x = intersectionPoints.at(i).x;
					  marker_point.y = intersectionPoints.at(i).y;
					  marker_point.z = 1.0;
					  //ROS_INFO("DOA intersection points received @(%f,%f)",p.x,p.y);
					  markerIntersections.points.push_back(marker_point);
				  }
			  }
  	  }
	  marker_pub_intersections.publish(markerIntersections);
	  marker_pub_doa.publish(markerDOA);
	  marker_pub_path.publish(markerPath);

	  r.sleep();
  }
  return 0;
}



float get_heading(float robot_x,float robot_y,float phi,float alpha)
{
	//half beam width i  radians
	//start from the middle
	//if middle fails
	//minimum step to cover beam
	//start expanding to sides from middle
	float half_beam_width = acos(-1);
	float beam_width=0;
	float positive_heading,negative_heading;
	float delta_heading = 0.09;
	float i =20.0;
	positive_heading = negative_heading = phi;
	/*ROS_INFO("phi: %f robot_x: %f robot_y: %f",phi,robot_x,robot_y);
	robot_x =1.0;
	robot_y =1.0;*/
	float heading = 999.0;
	while(beam_width<2*half_beam_width)
	{
		for(i=1.0;i<20.0;i+=1.0)
			if(get_map_cost(robot_x+(i*((d)/20.0)*cos(positive_heading+alpha)),robot_y+(i*((d)/20.0)*sin(positive_heading+alpha)))>10)
			{
				//ROS_INFO("heading:%f",heading);
				break;
			}
		if(i==20.0)
				return positive_heading;
		for(i=1.0;i<20.0;i+=1.0)
			if(get_map_cost(robot_x+(i*((d)/20.0)*cos(negative_heading+alpha)),robot_y+(i*((d)/20.0)*sin(negative_heading+alpha)))>10)
			{
				//ROS_INFO("heading:%f",negative_heading);
				break;
			}
		if(i==20.0)
				return negative_heading;
		positive_heading = positive_heading + delta_heading;
		negative_heading = negative_heading - delta_heading;
		beam_width = positive_heading - negative_heading;
	}
	ROS_ERROR("failed to get a heading");
	return heading;
}
int8_t get_map_cost(float x, float y)
{
		if(x<(float)globalGrid.info.width*globalGrid.info.resolution&&y<(float)globalGrid.info.height*globalGrid.info.resolution)
			return mapMetrix[(int)(x/globalGrid.info.resolution)][(int)(y/globalGrid.info.resolution)];
		//ROS_ERROR("out of map area");
		return 199;
}
void get_DOA_intersections(const std::deque<geometry_msgs::Pose2D> &localizationData,std::deque<geometry_msgs::Pose2D> &intersectionPoints)
{
	if(localizationData.size()<2)return;
	//float m1,m2,c1,c2;
	float x_step = 0.001;
	float y_thresh = 0.2;
	int numberOfIntersections=0;

	if(intersectionPoints.size()>0)intersectionPoints.clear();

	//ROS_INFO("DOA points: %d",localizationData.size());

	for(unsigned int i=0;i<localizationData.size();i++)
	{
		numberOfIntersections += i;
	}
//	ROS_INFO("intersection points: %d",numberOfIntersections);
	//intersectionPoints.resize(numberOfIntersections);

	for(unsigned int i =0 ; i<localizationData.size(); i++)
	{
		//make line 1
		//start from the next line and loop through all of them
			//make line 2
				//start from x=0 and increment by x_step s
					//if the angle between lines are greater than a given alpha
						//if abs(*y1-*y2)<y_thresh add the intersection point<= (*y1+*y2)/2
		float m1 = tan(localizationData[i].theta);
		for(unsigned int j =i+1 ; j<localizationData.size(); j++)
		{
			float m2 = tan(localizationData[j].theta);
			float dm = m1-m2;
			float dy = localizationData[i].y-localizationData[j].y;
			float dmx = m2*localizationData[j].x-m1*localizationData[i].x;
			float c1 = localizationData[i].y-localizationData[i].x*m1;

			if(fabs((m2-m1)/(1+m1*m2))>tan(alpha))
			{
				for(float x=0.5;x<(float)globalGrid.info.width*globalGrid.info.resolution;x+=x_step)
				{
					if(fabs(dy+dmx+x*dm)<=y_thresh)
					{
						float y = m1*x+c1;
						if(y>0.0&&y<(float)globalGrid.info.height*globalGrid.info.resolution)
						{
							//ROS_INFO("DOA intersection at (%f,%f)",x,y);
							geometry_msgs::Pose2D intPoint;
							intPoint.x = x;
							intPoint.y = y;
							intPoint.theta = 0.0;
							intersectionPoints.push_back(intPoint);
						}
						/*else
						{
							//ROS_INFO("DOA intersection at (%f,%f)",x,y);
							geometry_msgs::Pose2D intPoint;
							intPoint.x = -9.0;
							intPoint.y = -9.0;
							intPoint.theta = -9.0;
							intersectionPoints.push_back(intPoint);
						}*/
						break;
					}
				}
			}
		}

	}
	ROS_INFO("intersection points: %ld",intersectionPoints.size());
}

bool sufficient_DOAs(const std::deque<geometry_msgs::Pose2D> &localizationData)
{

	if(localizationData.size()==0)
		return false;
	float m1 = tan(localizationData[0].theta);
	float m2 = tan(localizationData.back().theta);
	ROS_INFO("tan(alpha) : %f %f",fabs((m2-m1)/(1+m1*m2)),tan(alpha));
	if(fabs((m2-m1)/(1+m1*m2))>tan(alpha))
		return true;
	return false;


}
void get_beacon_estimate(const std::deque<geometry_msgs::Pose2D> &intersectionPoints, geometry_msgs::Pose2D &coordinate)
{
	  float x_0 = 0.0;
	  float y_0 = 0.0;
	  float count = 0.0;
	  for (unsigned int i=0; i<intersectionPoints.size(); i++)
	  {
		  	  //ROS_INFO("intersection point @(%f,%f)",intersectionPoints.at(i).x,intersectionPoints.at(i).y);
			  x_0 += intersectionPoints.at(i).x;
			  y_0 += intersectionPoints.at(i).y;
			  count += 1.0;
	  }
	  x_0 = x_0/count;
	  y_0 = y_0/count;
	  ROS_INFO("estimated beacon location @(%f,%f)",x_0,y_0);
	  //get_safe_near_point(x_0,y_0);
	  float xx = x_0;
	  float yy = y_0;

		for(float rr=0.0;rr<1.0;rr+=0.05)
		{
			for(float th=0.0;th<2*acos(-1);th+=0.09)
			{
				xx = x_0 + rr*cos(th);
				yy = y_0 + rr*sin(th);
				if(get_map_cost(xx,yy)==0)
				{
					y_0 = yy;
					x_0 = xx;
					break;
				}
			}
			if(get_map_cost(xx,yy)==0)break;
		}

		coordinate.x = x_0;
		coordinate.y = y_0;
}

void get_step_on_plan(const nav_msgs::GetPlan &Plan,const tf::StampedTransform &transformMapToRobot,float &ggx,float &ggy)
{
	float dx = Plan.response.plan.poses[0].pose.position.x-transformMapToRobot.getOrigin().x();
	float dy = Plan.response.plan.poses[0].pose.position.y-transformMapToRobot.getOrigin().y();

	float distance = sqrt(dx*dx+dy*dy);
	int pos = 0;
	while(distance<1.0)
	{
		dx = Plan.response.plan.poses[pos].pose.position.x-Plan.response.plan.poses[pos+1].pose.position.x;
		dy = Plan.response.plan.poses[pos].pose.position.y-Plan.response.plan.poses[pos+1].pose.position.y;
		distance += sqrt(dx*dx+dy*dy);
	}
	ggx = Plan.response.plan.poses[20].pose.position.x;
	ggy = Plan.response.plan.poses[20].pose.position.y;
}
bool get_safe_near_point(float &x_,float &y_)
{
	double xx = (double)x_;
	double yy = (double)y_;

	for(double rr=0.0;rr<1.0;rr+=0.05)
	{
		for(double th=0.0;th<2*acos(-1);th+=0.09)
		{
			xx = (double)x_ + rr*cos(th);
			yy = (double)y_ + rr*sin(th);
			if(get_map_cost(xx,yy)==0)
			{
				y_ = (float)yy;
				x_ = (float)xx;
				return true;
			}
		}
	}
	return false;
}


