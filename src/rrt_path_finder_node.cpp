/*
 * Standard libs include
 */
#include <iostream>
#include <vector>
#include <float.h>
#include <time.h>
#include <cmath>
#include <string>
/*
 * OpenCV includes
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
/*
 * Ros includes
 */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"

#define MAX_INC 10
#define PRECISION 0.1

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

typedef struct _Vertex
{
  double data[2];
  struct _Vertex * parent;
  double dist;
  unsigned int index;
}Vertex;

static void onMouse( int event, int x, int y, int, void* );
int is_goal_reachable(const std::vector<Vertex*>& lv, const cv::Mat& im);
double dist2(const Vertex& v1, const Vertex& v2);
double no_wall_between(const Vertex& v1, const Vertex& v2, const cv::Mat& im);
Vertex* nearest_vertex(const Vertex& qrand, const std::vector<Vertex*>& lv);
void rand_free_conf(Vertex& qrand, int height, int width);
Vertex* new_conf(const Vertex& qrand, Vertex& qnear, double *dq, const cv::Mat& im);
cv::Point2f vertex_to_point2f(const Vertex& v);
std::string type2str(int type);

Robot r;

void odom_to_map(Robot& r, const Map& m)
{
	r.robot_pos_in_image[0] = r.robot_pos[0]/m.res - m.origin[0]/m.res;
	r.robot_pos_in_image[1] = r.robot_pos[1]/m.res - m.origin[1]/m.res;
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

int is_goal_reachable(const Vertex& target, const std::vector<Vertex*>& lv, const cv::Mat& im)
{
  double min_dist = DBL_MAX;
  double dist;
  int ind = -1;
  for(unsigned int i = 0 ; i < lv.size() ; i++)
  {
    dist = sqrt(dist2(target,*lv[i]));
    if(no_wall_between(target,*lv[i], im)>=dist && min_dist > dist)
    {
      min_dist = dist;
      ind = i;
    }
  }
  return ind;
}

double dist2(const Vertex& v1, const Vertex& v2)
{
    return ((v1.data[0]-v2.data[0])*(v1.data[0]-v2.data[0]) + (v1.data[1]-v2.data[1])*(v1.data[1]-v2.data[1])); 
}

double no_wall_between(const Vertex& v1, const Vertex& v2, const cv::Mat& im)
{
  uint8_t* pixelPtr = (uint8_t*)im.data;
  int cn = im.channels();

  double dist = dist2(v1, v2);
//  std::cout << "dist:"<<sqrt(dist) << std::endl;
  double vx = -(v1.data[0]-v2.data[0])/sqrt(dist); //vecteur unitaire selon x
  double vy = -(v1.data[1]-v2.data[1])/sqrt(dist); //vecteur unitaire selon y
  double i = PRECISION;
  while((vx*i*vx*i) + (vy*i*vy*i) < dist)
  {
//  	std::cout << "dist:"<<sqrt((vx*i*vx*i) + (vy*i*vy*i)) << std::endl;
//  	std::cout << "x:"<<ceil((v1.data[0] + i*vx)) << " y:" << ceil((v1.data[1] + i*vy))<< std::endl;
 	// std::cout << "pix:"<<(int)pixelPtr[(int)(ceil((v1.data[1] + i*vy))*im.cols*cn + ceil((v1.data[0] + i*vx))*cn)] << std::endl;
    if(pixelPtr[(int)(ceil((v1.data[1] + i*vy))*im.cols*cn + ceil((v1.data[0] + i*vx))*cn)] == 0)
    {
      //cv::circle(im, cv::Point(ceil((v1.data[0] + i*vx)),ceil((v1.data[1] + i*vy))), 3, cv::Scalar(0,255,0), -1, 8, 0 );
      return i - PRECISION;
    }
    // std::cout << ceil((v1.data[1] + i*vy))*im.cols*cn + ceil((v1.data[0] + i*vx))*cn << std::endl;
    i += PRECISION;
  }
  return i;
}

Vertex* nearest_vertex(const Vertex& qrand, const std::vector<Vertex*>& lv)
{
  int min_index = 0;
  double min = DBL_MAX;
  double dist;

  for(unsigned int i = 0 ; i < lv.size() ; i++)
    if(min > (dist = dist2(qrand, *lv[i])))
    {
        min = dist;
        min_index = i;
    }
  return lv[min_index];
}

void rand_free_conf(Vertex& qrand, int height, int width)
{

  qrand.data[0] = rand()%height;
  qrand.data[1] = rand()%width;
  // std::cout << "randx: " << qrand.data[0] << " randy: " << qrand.data[1] << std::endl;
  qrand.parent = NULL;
}

Vertex* new_conf(const Vertex& qrand, Vertex& qnear, double *dq, const cv::Mat& im)
{
  double dist = dist2(qrand, qnear);
  double vx = qnear.data[0] + (qrand.data[0]-qnear.data[0])/sqrt(dist)*(*dq);
  double vy = qnear.data[1] + (qrand.data[1]-qnear.data[1])/sqrt(dist)*(*dq);
  vx = (vx < 0) ? 0 : vx;
  vy = (vy < 0) ? 0 : vy;
  vx = (vx > im.cols) ? im.cols : vx;
  vy = (vy > im.rows) ? im.rows : vy;
  Vertex* v = new Vertex{{vx, vy},&qnear, qnear.dist + *dq, qnear.index+1};
  if((*dq = no_wall_between(*v, qnear, im)) == MAX_INC)
    return v;
  else if(*dq > 0)
  {
    v->data[0] = qnear.data[0] + (qrand.data[0]-qnear.data[0])/sqrt(dist)*(*dq);
    v->data[1] = qnear.data[1] + (qrand.data[1]-qnear.data[1])/sqrt(dist)*(*dq);
    v->dist = qnear.dist + *dq;
    return v;
  }
  return NULL;
}

cv::Point2f vertex_to_point2f(const Vertex& v)
{
  return cv::Point2f(v.data[0], v.data[1]);
}

std::string type2str(int type) 
{
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) 
  {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void find_path(cv::Mat image, Vertex* parent, Vertex* qnew, std::vector<Vertex*>& path)
{
  while(parent!=NULL && ros::ok())
  {
    path[qnew->index]=qnew;
    cv::line(image, vertex_to_point2f(*parent), vertex_to_point2f(*qnew), cv::Scalar(0,255,0), 2, CV_AA);
    qnew=parent;
    parent=qnew->parent;
    cv::imshow( "Display window", image );
    cv::waitKey(1);
  }
  path[qnew->index]=qnew;
}

void straighten_path(cv::Mat image, const cv::Mat emptyMap, std::vector<Vertex*>& path)
{
 	unsigned int i=0;
  while(i < path.size()-2 && ros::ok())
  {
		for(unsigned int j=0; j < path.size()-i-3; j++)
		{
			int size = path.size();
			double dist = sqrt(dist2(*path[j],*path[size-i-1]));
			if(no_wall_between(*path[j],*path[size-i-1], emptyMap)>=dist)
			{
				cv::line(image, vertex_to_point2f(*path[j]), vertex_to_point2f(*path[size-i-1]), cv::Scalar(255,0,0), 2, CV_AA);
				path.erase(path.begin()+j+1, path.begin() + size-i-1);
				break;
			}
			
		}
		i++;
		cv::imshow( "Display window", image );
		cv::waitKey(10);
	}
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
	  for(int i = 0 ; i < srv.response.map.data.size() ; i++)
	  {
	  	std::cout << srv.response.map.data[i] << std::endl;
	  	map.data[i] = srv.response.map.data[i];
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

/* Main fonction */
int main(int argc, char* argv[])
{
  bool without_mapping = false;
  for(int i = 0 ; i < argc ; i++)
  {
  	std::string arg(argv[i]);
  	if(arg == "--without-mapping")
  		without_mapping = true;
  	std::cout << without_mapping << std::endl;
  }

  if(argc < 3 && without_mapping)
  {
    std::cout << "Usage: ./test [image_name] [flags: --without-mapping]" << std::endl;
    return 1;
  }

  ros::init(argc, argv, "rrt_path_finder_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10); //10 Hz

  // Subscribe to Odometry to get the position of the robot in the map
  ros::Subscriber subOdom = n.subscribe("/odom", 10, odomCallback);
	
	std::vector<Vertex*> vertices;
  Vertex qrand, *qnear, *qnew=NULL;
  cv::Mat image,emptyMap, inMap;

  cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  cv::setMouseCallback( "Display window", onMouse, 0 );

  srand(time(NULL));

  double dq = MAX_INC;

  /* RRT example: not with a robot in parallel for pathfinding */
	if(without_mapping)
	{
	  image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
	  emptyMap = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
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

	else
	{
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
	  	inMap = cv::Mat(map.height, map.width, CV_8UC1, map.data);
	  	emptyMap = cv::Mat(map.height, map.width, CV_8UC1, map.data);  
	  	cv::cvtColor(emptyMap, emptyMap, cv::COLOR_GRAY2BGR);
	  	
		  cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ROBOT_RADIUS/map.res,ROBOT_RADIUS/map.res),cv::Point(-1,-1));
		  cv::dilate(inMap, inMap, se, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	  	cv::dilate(emptyMap, emptyMap, se, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

	  	cv::bitwise_not(inMap, inMap, cv::noArray());
	  	cv::threshold(inMap, inMap, 254, 255, cv::THRESH_BINARY);
	  	flip(inMap, inMap, 0);

	  	ros::spinOnce();
	  	odom_to_map(r, map);
	  }
	  vertices.push_back(new Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0});

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
	    cv::line(inMap, vertex_to_point2f(*qnear), vertex_to_point2f(*qnew), cv::Scalar(0,0,255), 1, CV_AA);
	    cv::imshow( "Display window", inMap );                   // Show our image inside it.  
	    ros::spinOnce();
	    cv::waitKey(10);
	  }
	  std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;
	  Vertex *parent=qnew->parent;
	  std::vector<Vertex*> path(qnew->index+1);


	  /* Finding the path part */
	  while(parent!=NULL && ros::ok())
	  {
	    path[qnew->index]=qnew;
	//    std::cout << qnew->index << std::endl;
	    cv::line(inMap, vertex_to_point2f(*parent), vertex_to_point2f(*qnew), cv::Scalar(0,255,0), 2, CV_AA);
	    qnew=parent;
	    parent=qnew->parent;
	    cv::imshow( "Display window", inMap );
	    //cv::waitKey(10);
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
					cv::line(inMap, vertex_to_point2f(*path[j]), vertex_to_point2f(*path[size-i-1]), cv::Scalar(255,0,0), 2, CV_AA);
					cv::imshow( "Display window", inMap );
					path.erase(path.begin()+j+1, path.begin() + size-i-1);
					break;
				}
				cv::imshow( "Display window", inMap );
				cv::waitKey(10);
			}
			i++;
			cv::waitKey(10);
		}

		/* Keep showing the map with the robot on it */
		while(ros::ok())
		{
			if(map.height > 0 && map.width > 0)
			{
				flip(inMap, inMap, 0);
				cv::circle(inMap, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(255,255,255), 1, 8, 0);
				ros::spinOnce();
				odom_to_map(r, map);
				// std::cout << r.robot_pos[0] << " " << r.robot_pos[1] << std::endl;
	  	// 	std::cout << r.robot_pos_in_image[0] << " " << r.robot_pos_in_image[1] << std::endl;
	  		cv::circle(inMap, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(0,0,0), 1, 8, 0);
	  		flip(inMap, inMap, 0);
				cv::imshow( "Display window", inMap );                   // Show our image inside it.   
			}
	  	cv::waitKey(10);
	  	
		}
	}

  return 0;
}