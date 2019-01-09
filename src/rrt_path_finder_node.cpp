/*
 * Standard libs include
 */
#include <iostream>
#include <float.h>
#include <time.h>
#include <cmath>
#include <string>

/*
 * OpenCV includes
 */
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
/*
 * Ros includes
 */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

/*
 * node's libraries
 */
#include "rrt_path_finder.hpp"



#define ROBOT_RADIUS 0.35

double targetX=0;
double targetY=0;

typedef struct _Robot
{
	double robot_pos[2];
	double robot_pos_in_image[2];
}Robot;

typedef struct _Map
{
	int8_t *data;
	uint32_t height;
	uint32_t width;
	float res;
	double origin[2];
}Map;

static void onMouse( int event, int x, int y, int, void* );



Robot r;

void odom_to_map(Robot& r, const Map& m)
{
	r.robot_pos_in_image[0] = r.robot_pos[0]/m.res - m.origin[0]/m.res;
	r.robot_pos_in_image[1] = m.height - (r.robot_pos[1]/m.res - m.origin[1]/m.res);
}




static void onMouse( int event, int x, int y, int, void* )
{
  if( event != cv::EVENT_LBUTTONDOWN )
    return;
  targetX= x;
  targetY= y;

  std::cout << " x:" << x << "  y:" << y << std::endl;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	r.robot_pos[0] = msg->pose.pose.position.x;
	r.robot_pos[1] = msg->pose.pose.position.y;
}



bool get_slam_map(Map& map)
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
	nav_msgs::GetMap srv;
	if (client.call(srv))
  {
    ROS_INFO("Service GetMap succeeded.");
    std_msgs::Header header = srv.response.map.header;
	  nav_msgs::MapMetaData info = srv.response.map.info;
	  map.width = info.width;
	  map.height = info.height;
	  map.res = info.resolution;
	  map.origin[0] = info.origin.position.x;
	  map.origin[1] = info.origin.position.y;
	  map.data = new int8_t(map.width*map.height);
	  for(int i = 0 ; i < map.height*map.width ; i++)
	  	map.data[i] = srv.response.map.data[i];
	  ROS_INFO("Map width, height: %u, %u", map.width, map.height);
  }
  else
  {
    ROS_ERROR("Service GetMap failed.");
    return false;
  }
  ROS_INFO("Map loading succeeded.");
  return true;
}



std::vector<Vertex*>& rrt(Vertex* v, double tX, double tY,cv::Mat &emptyMap)
{
	/* RRT Part */
	cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.
	//cv::namedWindow( "Display window2", cv::WINDOW_NORMAL );// Create a window for display.
	cv::setMouseCallback( "Display window", onMouse, 0 );
	cv::Mat image,endIm;
	emptyMap.copyTo(image);
	emptyMap.copyTo(endIm);
	
	cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10),cv::Point(-1,-1));
	cv::erode(emptyMap, emptyMap, se, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	
	srand(time(NULL));
	double dq = MAX_INC;
	std::vector<Vertex*> vertices;
	vertices.push_back(v);
	
	//vertices.push_back(new Vertex{{60.,60.},NULL,0,0});
	Vertex qrand, *qnear, *qnew=NULL;
	int h = emptyMap.rows;
	int w = emptyMap.cols;
	std::cout << "size map"  << " ( " << h << " , " << w  << " )" << std::endl;
	
	while(ros::ok() && (qnew == NULL || !(abs(qnew->data[0] - tX)<10 && abs(qnew->data[1] - tY)<10)))
		{
			tX=targetX;
			tY=targetY;
			cv::circle(image, cv::Point(tX,tY), 10, cv::Scalar(255,0,0), -1, 8, 0);
			qnew = NULL;
			Vertex target{{tX, tY}, NULL, 0.0, 0};
			int ind = is_goal_reachable(target, vertices, emptyMap);
			if( ind != -1)
				{
					qnear = vertices[ind];
					qnew = new_conf(target, *qnear, &dq, emptyMap);
				}
			while(qnew == NULL)
				{
					
					rand_free_conf(qrand, h, w);
					cv::circle(image, cv::Point(qrand.data[0],qrand.data[1]), 2, cv::Scalar(0,0,0), -1, 8, 0);
					qnear = nearest_vertex(qrand, vertices);
					qnew = new_conf(qrand, *qnear, &dq, emptyMap);
					dq = MAX_INC;
				}
			vertices.push_back(qnew);
			cv::line(image, vertex_to_point2f(*qnear), vertex_to_point2f(*qnew), cv::Scalar(0,0,255), 1, CV_AA);
			cv::imshow( "Display window", image );                   // Show our image inside it.
			//cv::imshow( "Display window2", emptyMap );  
			cv::waitKey(10);
		}

	std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;
	Vertex *parent=qnew->parent;

	std::vector<Vertex*> path(qnew->index+1);

	/* Finding the path part */
	find_path(image, parent, qnew, path);

	/* Shortening the path with straight lines part */
	straighten_path(endIm, emptyMap, path);
	 
	/* Keep showing the map with the robot on it */
	// while(ros::ok())
	// 	{
	// 		cv::imshow( "Display window", image );
	// 		cv::waitKey(10);
	// 	}
	// /* Keep showing the map with the robot on it */
	// while(ros::ok())
	// 	{
	// 		// flip(emptyMap, emptyMap, 0);
	// 		//cv::circle(emptyMap, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(255,255,255), 1, 8, 0);
	// 		ros::spinOnce();
	// 		//odom_to_map(r, map);
	// 		// std::cout << r.robot_pos[0] << " " << r.robot_pos[1] << std::endl;
	// 		// std::cout << r.robot_pos_in_image[0] << " " << r.robot_pos_in_image[1] << std::endl;
	// 		//cv::circle(emptyMap, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(0,0,0), 1, 8, 0);
	// 		// flip(emptyMap, emptyMap, 0);
	// 		cv::imshow( "Display window", emptyMap );                   // Show our image inside it. 
	// 		//std::cout << cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]) << std::endl;
			       
	// 		cv::waitKey(10);
	  	
	// 	}
	
	std::vector<Vertex*>* pathCopy = new std::vector<Vertex*>(path); 
	return *pathCopy;
}

int simpleRRT(char * map_file)
{
	/* RRT example: not with a robot in parallel for pathfinding */
	cv::Mat image,emptyMap;
	image = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);   // Read the file
	emptyMap = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);
	
	std::vector<Vertex*> path = rrt(new Vertex{{60.,60.},NULL,0,0}, targetX, targetY, emptyMap);

	for(int i = 0 ; i<path.size(); i++)
		std::cout << "pt n°" << i << " ( " << path[i]->data[0] << " , " << path[i]->data[1] << " )" << std::endl; 
	 
	/* Keep showing the map with the robot on it */
	while(ros::ok()){}
}

/**
 * Construct_Path_Msg function
 * Used to populate a nav_msgs::Path given a list of x and y coordinates
 * @param path the path to convvert
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path construct_path_msg(std::vector<Vertex*> &path)
{
	nav_msgs::Path msg;
	std::vector<geometry_msgs::PoseStamped> poses(path.size());
	for (int i = 0; i < path.size(); i++)
	{
		poses.at(i).pose.position.x = path[i]->data[0];
		poses.at(i).pose.position.y = path[i]->data[1];
	}
	msg.poses = poses;
	return msg;
}

/* Main fonction */
int main(int argc, char* argv[])
{
	bool without_mapping = false;
	for(int i = 0 ; i < argc ; i++)
		{
			std::string arg(argv[i]);
			if(arg == "--without-mapping")
				without_mapping = true;
		}

	if(argc < 3 && without_mapping)
		{
			std::cout << "Usage: ./test [image_name] [flags: --without-mapping]" << std::endl;
			return 1;
		}
	
	ros::init(argc, argv, "rrt_path_finder_node");
	if(without_mapping)
		{
			std::cout << "simple RRT exemple" << std::endl;
			simpleRRT(argv[1]);
			return 0;
		}

	ros::NodeHandle n;
	ros::Rate loop_rate(100); //10 Hz

	// Subscribe to Odometry to get the position of the robot in the map
	ros::Subscriber subOdom = n.subscribe("/odom", 10, odomCallback);
	ros::spinOnce();
	std::vector<Vertex*> vertices;
	cv::Mat image,emptyMap, inMap;



	

 
	/* Real part, not to show off ! */
	Map map;
	map.width = 0;
	map.height = 0;
	// if(!get_slam_map(map))	{ return 1;}

	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
	nav_msgs::GetMap srv;
	if (client.call(srv))
		{
			ROS_INFO("Service GetMap succeeded.");
			std_msgs::Header header = srv.response.map.header;
			nav_msgs::MapMetaData info = srv.response.map.info;
			map.width = info.width;
			map.height = info.height;
			map.res = info.resolution;
			map.origin[0] = info.origin.position.x;
			map.origin[1] = info.origin.position.y;
			map.data = &(srv.response.map.data[0]);

			ROS_INFO("Map width, height: %u, %u", map.width, map.height);
		}
	else
		{
			ROS_ERROR("Service GetMap failed.");
			return 1;
		}
	ROS_INFO("Map loading succeeded.");

	if(map.height > 0 && map.width > 0)
		{
			emptyMap = cv::Mat(map.height, map.width, CV_8UC1, map.data);  
	  
			cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ROBOT_RADIUS/map.res,ROBOT_RADIUS/map.res),cv::Point(-1,-1));
			cv::dilate(emptyMap, emptyMap, se, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

			cv::bitwise_not(emptyMap, emptyMap, cv::noArray());

			cv::threshold(emptyMap, emptyMap, 254, 255, cv::THRESH_BINARY);

			flip(emptyMap, emptyMap, 0);

			ros::spinOnce();
			odom_to_map(r, map);
		}

	targetX = r.robot_pos_in_image[0] + 1;
	targetY = r.robot_pos_in_image[1];

	std::vector<Vertex*>& path = rrt(new Vertex{{24,254},NULL,0,0},targetX,targetY,emptyMap);
	//rrt(new Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0},targetX,targetY,emptyMap);

	ros::Publisher pubPath = n.advertise<nav_msgs::Path>("path", 10);
	nav_msgs::Path path_msg = construct_path_msg(path);
	pubPath.publish(path_msg);
	while(ros::ok()){cv::waitKey(10);}
	
	return 0;
}
