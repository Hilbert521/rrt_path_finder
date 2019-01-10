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
#include "tf/transform_listener.h"

/*
 * node's libraries
 */
#include "rrt_path_finder.hpp"



#define ROBOT_RADIUS 0.35

double targetX=0;
double targetY=0;

typedef struct _Robot
{
	// x, y, theta
	double robot_pos[3];
	double robot_pos_in_image[2];
}Robot;

typedef struct _Map
{
	// int8_t *data;
	cv::Mat map;
	uint32_t height;
	uint32_t width;
	float res;
	double origin[2];
	double tX;
	double tY;
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

// bool get_slam_map(Map& map)
// {
// 	ros::NodeHandle n;
// 	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
// 	nav_msgs::GetMap srv;
// 	if (client.call(srv))
//   {
//     ROS_INFO("Service GetMap succeeded.");
//     std_msgs::Header header = srv.response.map.header;
// 	  nav_msgs::MapMetaData info = srv.response.map.info;
// 	  map.width = info.width;
// 	  map.height = info.height;
// 	  map.res = info.resolution;
// 	  map.origin[0] = info.origin.position.x;
// 	  map.origin[1] = info.origin.position.y;
// 	  map.data = new int8_t(map.width*map.height);
// 	  for(int i = 0 ; i < map.height*map.width ; i++)
// 	  	map.data[i] = srv.response.map.data[i];
// 	  ROS_INFO("Map width, height: %u, %u", map.width, map.height);
//   }
//   else
//   {
//     ROS_ERROR("Service GetMap failed.");
//     return false;
//   }
//   ROS_INFO("Map loading succeeded.");
//   return true;
// }



std::vector<Vertex*>& rrt(Vertex* v, Map& m)
{
	/* RRT Part */
	cv::Mat image,endIm, emptyMap;

	m.map.copyTo(emptyMap);
	m.map.copyTo(image);
	cv::circle(image, cv::Point(v->data[0], v->data[1]), 10, cv::Scalar(0,255,0), -1, 8, 0);
	image.copyTo(endIm);
	

	cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10),cv::Point(-1,-1));
	cv::erode(emptyMap, emptyMap, se, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	
	srand(time(NULL));
	double dq = MAX_INC;
	std::vector<Vertex*> vertices;
	vertices.push_back(v);
	
	// vertices.push_back(new Vertex{{60.,60.},NULL,0,0});
	Vertex qrand, *qnear, *qnew=NULL;
	int h = emptyMap.rows;
	int w = emptyMap.cols;
	std::cout << "size map"  << " ( " << h << " , " << w  << " )" << std::endl;
	
	while(ros::ok() && (qnew == NULL || !(abs(qnew->data[0] - m.tX)<10 && abs(qnew->data[1] - m.tY)<10)))
		{
			m.tX=targetX;
			m.tY=targetY;
			cv::circle(image, cv::Point(m.tX,m.tY), 10, cv::Scalar(0,255,0), -1, 8, 0);

			qnew = NULL;
			Vertex target{{m.tX, m.tY}, NULL, 0.0, 0};
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
			cv::imshow( "Display window2", image );                   // Show our image inside it.
			cv::waitKey(10);
		}

	std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;
	Vertex *parent=qnew->parent;

	std::vector<Vertex*> path(qnew->index+1);

	/* Finding the path part */
	find_path(image, parent, qnew, path);

	/* Shortening the path with straight lines part */
	straighten_path(endIm, emptyMap, path);
	 	
	std::vector<Vertex*>* pathCopy = new std::vector<Vertex*>(path); 
	return *pathCopy;
}

int simpleRRT(char *map_file)
{
	/* RRT example: not with a robot in parallel for pathfinding */
	cv::Mat image,emptyMap;
	Map map;
	image = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);   // Read the file
	emptyMap = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);
	
	map.width = image.cols;
	map.height = image.rows;
	map.res = 1;
	map.tX = targetX;
	map.tY = targetY;
	map.map = image;
	
	std::vector<Vertex*> path = rrt(new Vertex{{60.,60.},NULL,0,0}, map);

	for(int i = 0 ; i<path.size(); i++)
		std::cout << "pt n°" << i << " ( " << path[i]->data[0] << " , " << path[i]->data[1] << " )" << std::endl; 
	 
	/* Keep showing the map with the robot on it */
	while(ros::ok()){cv::waitKey(10);}
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
		std::cout << "pt n°" << i << " ( " << path[i]->data[0] << " , " << path[i]->data[1] << " )" << std::endl; 
		poses.at(i).pose.position.x =0.01* path[i]->data[0];
		poses.at(i).pose.position.y = 0.01*path[i]->data[1];
		poses.at(i).header.frame_id = "map";
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

	cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.
	cv::namedWindow( "Display window2", cv::WINDOW_NORMAL );// Create a window for display.
	cv::setMouseCallback( "Display window", onMouse, 0 );

	if(without_mapping)
	{
		std::cout << "simple RRT exemple" << std::endl;
		simpleRRT(argv[1]);
		return 0;
	}

	ros::NodeHandle n;
	ros::Rate loop_rate(100); //10 Hz
	ros::Publisher pubPath = n.advertise<nav_msgs::Path>("path", 10);

	std::vector<Vertex*> vertices;
	cv::Mat image,emptyMap, inMap;

	tf::TransformListener t;

	/* Real part, not to show off ! */
	Map map;
	map.width = 0;
	map.height = 0;
	targetX = -1;
	targetY = -1;
	double targetPastX = -1;
	double targetPastY = -1;
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
		// map.data = &(srv.response.map.data[0]);
		map.tX = targetX;
		map.tY = targetY;
		if(map.height > 0 && map.width > 0)
		{
			map.map = cv::Mat(map.height, map.width, CV_8UC1, &(srv.response.map.data[0]));  

			cv::bitwise_not(map.map, map.map, cv::noArray());

			cv::threshold(map.map, map.map, 254, 255, cv::THRESH_BINARY);

			flip(map.map, map.map, 0);

			odom_to_map(r, map);
		}

		ROS_INFO("Map width, height: %u, %u", map.width, map.height);
	}
	else
	{
		ROS_ERROR("Service GetMap failed.");
		return 1;
	}
	ROS_INFO("Map loading succeeded.");

	// if(map.height > 0 && map.width > 0)
	// {
	// 	emptyMap = cv::Mat(map.height, map.width, CV_8UC1, &(srv.response.map.data[0]));  

	// 	cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ROBOT_RADIUS/map.res,ROBOT_RADIUS/map.res),cv::Point(-1,-1));
	// 	cv::dilate(emptyMap, emptyMap, se, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

	// 	cv::bitwise_not(emptyMap, emptyMap, cv::noArray());

	// 	cv::threshold(emptyMap, emptyMap, 254, 255, cv::THRESH_BINARY);

	// 	flip(emptyMap, emptyMap, 0);

	// 	odom_to_map(r, map);
	// }

    
    r.robot_pos[0] = -1;
	r.robot_pos[1] = -1;

    while(ros::ok() && r.robot_pos[0] == -1 && r.robot_pos[1] == -1)
	{
		// Get frame change between slam_karto map frame and the frame of the odom of the robot
		tf::StampedTransform transform_slam;
		try
		{
			t.lookupTransform("map", "base_footprint", ros::Time(0), transform_slam);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		r.robot_pos[0] = transform_slam.getOrigin().x();
		r.robot_pos[1] = transform_slam.getOrigin().y();

		odom_to_map(r, map);
	}

	while(ros::ok())
	{
		// Get frame change between slam_karto map frame and the frame of the odom of the robot
		tf::StampedTransform transform_slam;
		try
		{
			t.lookupTransform("map", "base_footprint", ros::Time(0), transform_slam);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		r.robot_pos[0] = transform_slam.getOrigin().x();
		r.robot_pos[1] = transform_slam.getOrigin().y();

		odom_to_map(r, map);
		cv::circle(map.map, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(0,0,0), -1, 8, 0);
		// std::vector<Vertex*>& path = rrt(new Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0},targetX,targetY,emptyMap);
		// nav_msgs::Path path_msg = construct_path_msg(path);
		// path_msg.header.frame_id = "map";
		// pubPath.publish(path_msg);
		// std::cout << std::endl;
		// std::cout << r.robot_pos_in_image[0] << "|" << r.robot_pos_in_image[1] << std::endl;
		// std::cout << transform_slam.getOrigin().x() << std::endl;
		//    std::cout << transform_slam.getOrigin().y() << std::endl;
		//    std::cout << transform_slam.getOrigin().z() << std::endl;
		//    std::cout << transform_slam.getRotation().x() << std::endl;
		//    std::cout << transform_slam.getRotation().y() << std::endl;
		//    std::cout << transform_slam.getRotation().z() << std::endl;
		//    std::cout << transform_slam.getRotation().w() << std::endl;  

		if(targetY != targetPastY && targetX != targetPastX)
		{
			rrt(new Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0}, map);
			targetPastX = targetX;
			targetPastY = targetY;
		}

		cv::imshow( "Display window", map.map );                   // Show our image inside it.
		cv::circle(map.map, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(255,0,0), -1, 8, 0);
		//pubPath.publish(path_msg);
		cv::waitKey(10);
	}	
	return 0;
}
