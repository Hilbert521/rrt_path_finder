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
#define OP_FACTOR 3

double targetX=0;
double targetY=0;
bool rviz_goal_flag = false;

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

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	targetX = msg->pose.position.x;
	targetY = msg->pose.position.y;
	rviz_goal_flag = true;
	
	std::cout << "New GOAL:"<< std::endl;
	std::cout << targetX << " " << targetY << std::endl;
}

void odom_to_map(Robot& r, const Map& m)
{
	r.robot_pos_in_image[0] = r.robot_pos[0]/m.res - m.origin[0]/m.res;
	r.robot_pos_in_image[1] = m.height - (r.robot_pos[1]/m.res - m.origin[1]/m.res);
}

Vertex *map_to_odom(const Vertex *v, const Map& m)
{
	Vertex *ret = new Vertex();
	ret->data[0] = v->data[0]*m.res + m.origin[0];
	ret->data[1] = -(v->data[1] - m.height)*m.res + m.origin[1];
	return ret;
}

static void onMouse( int event, int x, int y, int, void* )
{
	if( event != cv::EVENT_LBUTTONDOWN )
		return;
	targetX= x;
	targetY= y;

	std::cout << " x:" << x << "  y:" << y << std::endl;
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
			map.tX = targetX;
			map.tY = targetY;
			if(map.height > 0 && map.width > 0)
				{
					cv::Mat tmp(map.height, map.width, CV_8UC1, &(srv.response.map.data[0])); 
					tmp.copyTo(map.map);// = cv::Mat(map.height, map.width, CV_8UC1, &(srv.response.map.data[0]));  

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
			return false;
		}
	ROS_INFO("Map loading succeeded.");
	return true;
}



std::vector<Vertex*>& rrt(Vertex* v, Map& m, bool with_gui,Vertex* tar=NULL)
{
	/* RRT Part */
	srand(time(NULL));
	double dq = MAX_INC;
	std::vector<Vertex*> vertices;
	Vertex qrand, *qnear, *qnew=NULL;

	cv::Mat image,endIm, emptyMap;

	int h = m.map.rows;
	int w = m.map.cols;
	//std::cout << "size map"  << " ( " << h << " , " << w  << " )" << std::endl;

	// Copy the map as it really is to emptyMap (the map we are using to check if a path is ok or not)
	m.map.copyTo(emptyMap);
	

	// Dilate all the obstacles: we add to their real border the diameter of the robot
	cv::Mat se_ouverture = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ROBOT_RADIUS/m.res/OP_FACTOR<1?1:ROBOT_RADIUS/m.res/OP_FACTOR, ROBOT_RADIUS/m.res/OP_FACTOR<1?1:ROBOT_RADIUS/m.res/OP_FACTOR),cv::Point(-1,-1));
	cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,
					       cv::Size(3*ROBOT_RADIUS/m.res<1?1:ROBOT_RADIUS/m.res, 3*ROBOT_RADIUS/m.res<1?1:ROBOT_RADIUS/m.res),
					       cv::Point(-1,-1));
	// Ouverture pour supprimer les petits elements et grossir les gros
	cv::dilate(emptyMap, emptyMap, se_ouverture, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	cv::erode(emptyMap, emptyMap, se_ouverture, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
        

  /// Apply the erosion operation
	//	erode( src, erosion_dst, element );
	cv::erode(emptyMap, emptyMap, se, cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

	m.map.copyTo(image);
	// cv::circle(image, cv::Point(v->data[0], v->data[1]), 10, cv::Scalar(0,255,0), -1, 8, 0);
	image.copyTo(endIm);
	vertices.push_back(v);
	Vertex target{{m.tX, m.tY}, NULL, 0.0, 0};
	if(tar!=NULL)
		target = *tar;
	while(ros::ok() && (qnew == NULL || !(abs(qnew->data[0] - target.data[0])<10 && abs(qnew->data[1] - target.data[1])<10)))
		{
			m.tX=targetX;
			m.tY=targetY;
			cv::circle(image, cv::Point(m.tX,m.tY), 10, cv::Scalar(0,255,0), -1, 8, 0);

			qnew = NULL;
			if(tar==NULL)
				{
					target.data[0]=m.tX;
					target.data[1]=m.tY;
				}
			else
				{
					target = *tar;
					std::cout << "target" << std::endl;
				}
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
			if(with_gui)
			{
				cv::imshow( "Display window2", image );                   // Show our image inside it.
				cv::imshow( "Display window3", emptyMap );                   // Show our image inside it.
			}
			cv::waitKey(10);
		}

	std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;
	Vertex *parent=qnew->parent;

	std::vector<Vertex*> path(qnew->index+1);

	/* Finding the path part */
	find_path(image, parent, qnew, path, with_gui);
	//std::cout << "find path OK" << std::endl;

	/* Shortening the path with straight lines part */
	straighten_path(endIm, emptyMap, path, with_gui);
	//std::cout << "straighten_path OK" << std::endl;

	//if(tar==NULL)
	linear_interpol_path(endIm, emptyMap, path, with_gui);
	//std::cout << "interpolation OK" << std::endl;
	//if(tar==NULL)
	smoothen_path(endIm, emptyMap, path, BEZIER, with_gui);
	//std::cout << "smoothen path OK" << std::endl;

	// for(int i = 0 ; i<path.size(); i++)
	// 	std::cout << "pt n°" << i << " ( " << path[i]->data[0] << " , " << path[i]->data[1] << " )" << std::endl; 
	
	std::vector<Vertex*>* pathCopy = new std::vector<Vertex*>(path); 

	std::reverse(pathCopy->begin(),pathCopy->end());
	//std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;

	return *pathCopy;
}

void avoid_obstacle(std::vector<Vertex*> &path, Map& m);
void draw_path(std::vector<Vertex*> &path, Map& m)
{
	for(int i = 0 ; i<path.size(); i++)
		cv::circle(m.map, cv::Point2f(path[i]->data[0], path[i]->data[1]), 3, cv::Scalar(0,0,0), -1, 8, 0);
	cv::imshow( "Display window2", m.map );
}
int simpleRRT(char *map_file, bool with_gui)
{
	/* RRT example: not with a robot in parallel for pathfinding */
	cv::Mat image,emptyMap;
	Map map;
	image = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);   // Read the file
	emptyMap = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);
	
	map.width = image.cols;
	map.height = image.rows;
	map.res = ROBOT_RADIUS;
	map.tX = targetX;
	map.tY = targetY;
	map.map = image;
	
	std::vector<Vertex*> path = rrt(new Vertex{{60.,60.},NULL,0,0}, map, with_gui);
	avoid_obstacle(path,map);
	draw_path(path,map);
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
nav_msgs::Path construct_path_msg(std::vector<Vertex*> &path, const Map& m)
{
	nav_msgs::Path msg;
	std::vector<Vertex*> real_path(path.size());
	std::vector<geometry_msgs::PoseStamped> poses(path.size());
	for (int i = 0; i < path.size(); i++)
		{
			real_path[i] = map_to_odom(path[i], m);
			poses.at(i).pose.position.x = real_path[i]->data[0];
			poses.at(i).pose.position.y = real_path[i]->data[1];
			//std::cout << "REAL pt n°" << i << " ( " << real_path[i]->data[0] << " , " << real_path[i]->data[1] << " )" << std::endl; 
			poses.at(i).header.frame_id = "map";
		}
	msg.poses = poses;
	msg.header.frame_id = "map";
	return msg;
}

double is_in_wall(const Vertex& v1, const cv::Mat& im)
{
  uint8_t* pixelPtr = (uint8_t*)im.data;
  int cn = im.channels();

  if(pixelPtr[(int)(ceil((v1.data[1]))*im.cols*cn + ceil((v1.data[0]))*cn)] == 0)
	  return true;
  else
	  return false;
}

/**
 * Construct_Path_Msg function
 * Used to populate a nav_msgs::Path given a list of x and y coordinates
 * @param path the path to convvert
 * @return msg the constructed nav_msgs::Path message
 */
void avoid_obstacle(std::vector<Vertex*> &path, Map& m)
{
	nav_msgs::Path msg;
	std::vector<int>::iterator it;
	std::vector<Vertex*> n_path;
	bool is_ok = true;
	cv::Mat emptyMap;
	m.map.copyTo(emptyMap);

	// Dilate all the obstacles: we add to their real border the diameter of the robot
	cv::Mat se_ouverture = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ROBOT_RADIUS/m.res/OP_FACTOR<1?1:ROBOT_RADIUS/m.res/OP_FACTOR, ROBOT_RADIUS/m.res/OP_FACTOR<1?1:ROBOT_RADIUS/m.res/OP_FACTOR),cv::Point(-1,-1));
	cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3*ROBOT_RADIUS/m.res<1?1:ROBOT_RADIUS/m.res, 3*ROBOT_RADIUS/m.res<1?1:ROBOT_RADIUS/m.res),cv::Point(-1,-1));
	// Ouverture pour supprimer les petits elements et grossir les gros
	cv::dilate(emptyMap, emptyMap, se_ouverture, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	cv::erode(emptyMap, emptyMap, se_ouverture, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	cv::erode(emptyMap, emptyMap, se, cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	
	for (int i = 0; i < path.size()-1; i++)
		{
			double dist = sqrt(dist2( *path[i],  *path[i+1]));
			n_path.push_back(path[i]);
			//std::cout << "pt : "  << i << "   /nb:" << path.size() << std::endl;
			//std::cout << "test : "  << no_wall_between( *path[i], *path[i+1], m.map)<< std::endl;
			if(dist > no_wall_between( *path[i], *path[i+1], emptyMap))
				{
					is_ok = false;
					int j = i+1;
					//std::cout << "Wall found ... "  << std::endl;
					if(is_in_wall( *path[j], emptyMap))
						{
							//std::cout << "on wall ... "  << std::endl;
							for(j=j+1; j < path.size(); j++)
								if(!is_in_wall( *path[j], emptyMap))
										break;
						}
					std::vector<Vertex*> tmp_path = rrt(path[i], m, false,path[j] );
					for(int k = 0 ; k < tmp_path.size(); k++)
						n_path.push_back(tmp_path[k]);
					i=j;
					
				}
		}
	
	if(!is_ok)
		{
			//std::cout << "New trajectory ... "  << std::endl;
			path.erase(path.begin(),path.end());	
			for(int i = 0; i<n_path.size();i++)
				path.push_back(n_path[i]);
			//smoothen_path(emptyMap, emptyMap, path, BEZIER, true);
			
		}
	else
		{
			//std::cout << "No trajectory ... "  << std::endl;
		}
					
}

/* Main fonction */
int main(int argc, char* argv[])
{
	bool without_mapping = false;
	bool with_gui = false;
	for(int i = 0 ; i < argc ; i++)
		{
			std::string arg(argv[i]);
			if(arg == "--without-mapping")
				without_mapping = true;
			if(arg == "--with-gui")
				with_gui = true;
		}

	if(argc < 3 && without_mapping)
		{
			std::cout << "Usage: ./test [image_name] [flags: --without-mapping]" << std::endl;
			return 1;
		}
	
	ros::init(argc, argv, "rrt_path_finder_node");

	if(with_gui)
	{
		cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window to display the robot in its environment.
		cv::namedWindow( "Display window2", cv::WINDOW_NORMAL );// Create a window to display the rrt algorithm working.
		cv::namedWindow( "Display window3", cv::WINDOW_NORMAL );// Create a window to display the rrt algorithm working.
		cv::setMouseCallback( "Display window", onMouse, 0 );
		cv::setMouseCallback( "Display window2", onMouse, 0 );
		cv::setMouseCallback( "Display window3", onMouse, 0 );
	}
	
	if(without_mapping)
		{
			std::cout << "simple RRT exemple" << std::endl;
			simpleRRT(argv[1], with_gui);
			return 0;
		}

	ros::NodeHandle n;
	ros::Rate loop_rate(100); //10 Hz
	ros::Publisher pubPath = n.advertise<nav_msgs::Path>("/rrt/path", 10);
	ros::Subscriber subPath = n.subscribe("/move_base_simple/goal", 10, goalCallback);

	std::vector<Vertex*> path;
	nav_msgs::Path path_to_pub;

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

	if(!get_slam_map(map))	{ return 1;}
    
	r.robot_pos[0] = -1;
	r.robot_pos[1] = -1;

	while(ros::ok() && r.robot_pos[0] == -1 && r.robot_pos[1] == -1)
		{
			// Get frame change between slam_karto map frame and the frame of the odom of the robot
			tf::StampedTransform transform_slam;
			try
				{
					t.lookupTransform("map", "base_link", ros::Time(0), transform_slam);
				}
			catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
					std::cout << "test" << std::endl;
				}
			r.robot_pos[0] = transform_slam.getOrigin().x();
			r.robot_pos[1] = transform_slam.getOrigin().y();

			odom_to_map(r, map);
		}

	while(ros::ok())
		{
			ros::spinOnce();
			// Get frame change between slam_karto map frame and the frame of the odom of the robot
			tf::StampedTransform transform_slam;
			try
				{
					t.lookupTransform("map", "base_link", ros::Time(0), transform_slam);
				}
			catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
			
			r.robot_pos[0] = transform_slam.getOrigin().x();
			r.robot_pos[1] = transform_slam.getOrigin().y();

			odom_to_map(r, map);
		
			if(targetY != targetPastY && targetX != targetPastX)
				{
					get_slam_map(map);
					if(rviz_goal_flag)
					{
						targetX = targetX/map.res - map.origin[0]/map.res;
						targetY = map.height - (targetY/map.res - map.origin[1]/map.res);
						std::cout << targetX << " " << targetY << std::endl;
						rviz_goal_flag = false;
					}
					std::cout << "Robot REAL pos: " << r.robot_pos[0] << " | " << r.robot_pos[1] << std::endl;
					std::cout << "Robot IM pos: " << r.robot_pos_in_image[0] << " | " << r.robot_pos_in_image[1] << std::endl;
					path = rrt(new Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0}, map, with_gui);
					path_to_pub = construct_path_msg(path, map);
					targetPastX = targetX;
					targetPastY = targetY;
				}
			//std::cout << "running..." << std::endl;
			if(path.size()>0)
				{
					avoid_obstacle(path, map);
					//std::cout << "publishing..." << std::endl;
					path_to_pub = construct_path_msg(path, map);
					pubPath.publish(path_to_pub);
				}

			if(with_gui)
			{
				cv::circle(map.map, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(0,0,0), -1, 8, 0);
				cv::imshow( "Display window", map.map );                   // Show our image inside it.
				cv::circle(map.map, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(255,0,0), -1, 8, 0);
			}

			
			cv::waitKey(10);
		}	
	return 0;
}
