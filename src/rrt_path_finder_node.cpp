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

int simpleRRT(char * map_file)
{
	/* RRT example: not with a robot in parallel for pathfinding */
	std::vector<Vertex*> vertices;
	Vertex qrand, *qnear, *qnew=NULL;
	cv::Mat image,emptyMap, inMap;

	cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.
	cv::setMouseCallback( "Display window", onMouse, 0 );

	srand(time(NULL));

	double dq = MAX_INC;

	image = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);   // Read the file
	emptyMap = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);
	int h = image.rows;
	int w = image.cols;

	vertices.push_back(new Vertex{{60.,60.},NULL,0,0});

	
	  
	/* RRT Part */
	while(ros::ok() && (qnew == NULL || !(abs(qnew->data[0] - targetX)<10 && abs(qnew->data[1] - targetY)<10)))
		{
			qnew = NULL;
			Vertex target{{targetX, targetY}, NULL, 0.0, 0};
			int ind = is_goal_reachable(target, vertices, emptyMap);
			if( ind != -1)
				{
					qnear = vertices[ind];
					qnew = new_conf(target, *qnear, &dq, emptyMap);
				}
			while(qnew == NULL)
				{
					rand_free_conf(qrand, h, w);
					qnear = nearest_vertex(qrand, vertices);
					qnew = new_conf(qrand, *qnear, &dq, emptyMap);
					dq = MAX_INC;
				}
			vertices.push_back(qnew);
			cv::line(image, vertex_to_point2f(*qnear), vertex_to_point2f(*qnew), cv::Scalar(0,0,255), 1, CV_AA);
			cv::imshow( "Display window", image );                   // Show our image inside it.  
			cv::waitKey(10);
			std::cout << "test" << std::endl;
		}
	std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;
	Vertex *parent=qnew->parent;

	std::vector<Vertex*> path(qnew->index+1);

	/* Finding the path part */
	find_path(image, parent, qnew, path);

	/* Shortening the path with straight lines part */
	straighten_path(image, emptyMap, path);
	 
	/* Keep showing the map with the robot on it */
	while(ros::ok())
		{
			cv::imshow( "Display window", image );
			cv::waitKey(10);
		}
	
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
	Vertex qrand, *qnear, *qnew=NULL;
	cv::Mat image,emptyMap, inMap;

	cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.
	cv::setMouseCallback( "Display window", onMouse, 0 );

	srand(time(NULL));

	double dq = MAX_INC;

 
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
	vertices.push_back(new Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0});

	targetX = r.robot_pos_in_image[0] + 1;
	targetY = r.robot_pos_in_image[1];

	/* RRT Part */
	while(ros::ok() && (qnew == NULL || !(abs(qnew->data[0] - targetX)<10 && abs(qnew->data[1] - targetY)<10)))
		{
			qnew = NULL;
			Vertex target{{targetX, targetY}, NULL, 0.0, 0};
			int ind = is_goal_reachable(target, vertices, emptyMap);
			if( ind != -1)
				{
					qnear = vertices[ind];
					qnew = new_conf(target, *qnear, &dq, emptyMap);
				}
			while(qnew == NULL)
				{
					rand_free_conf(qrand, map.height, map.width);
					qnear = nearest_vertex(qrand, vertices);
					qnew = new_conf(qrand, *qnear, &dq, emptyMap);
					dq = MAX_INC;
				}
			vertices.push_back(qnew);
			ros::spinOnce();
		}
	std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;
	Vertex *parent=qnew->parent;
	std::vector<Vertex*> path(qnew->index+1);

	/* Finding the path part */
	while(parent!=NULL && ros::ok())
		{
			path[qnew->index]=qnew;
			qnew=parent;
			parent=qnew->parent;
		}
	path[qnew->index]=qnew;

	/* Shortening the path with straight lines part */
	unsigned int i=0;
	while(i < path.size()-2 && ros::ok())
		{
			for(unsigned int j=0; j < path.size()-i-3; j++)
				{
					int size = path.size();
					double dist = sqrt(dist2(*path[j],*path[size-i-1]));
					if(no_wall_between(*path[j],*path[size-i-1], emptyMap)>=dist)
						{
							path.erase(path.begin()+j+1, path.begin() + size-i-1);
							break;
						}
				}
			i++;
		}

	/* Keep showing the map with the robot on it */
	while(ros::ok())
		{
			if(map.height > 0 && map.width > 0)
				{
					// flip(emptyMap, emptyMap, 0);
					cv::circle(emptyMap, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(255,255,255), 1, 8, 0);
					ros::spinOnce();
					odom_to_map(r, map);
					// std::cout << r.robot_pos[0] << " " << r.robot_pos[1] << std::endl;
					// std::cout << r.robot_pos_in_image[0] << " " << r.robot_pos_in_image[1] << std::endl;
					cv::circle(emptyMap, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(0,0,0), 1, 8, 0);
					// flip(emptyMap, emptyMap, 0);
					cv::imshow( "Display window", emptyMap );                   // Show our image inside it. 
					std::cout << cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]) << std::endl;
				}
			cv::waitKey(10);
	  	
		}
	
	return 0;
}
