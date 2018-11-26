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

#define MAX_INC 10
#define PRECISION 0.1

#define ROBOT_RADIUS 30

double targetX=0;
double targetY=0;

typedef struct _Map
{
	uint8_t *data;
	unsigned int height;
	unsigned int width;
}Map;

typedef struct _Vertex
{
  double data[2];
  struct _Vertex * parent;
  double dist;
  unsigned int index;
}Vertex;

Map map;

static void onMouse( int event, int x, int y, int, void* );
int is_goal_reachable(const std::vector<Vertex*>& lv, const cv::Mat& im);
double dist2(const Vertex& v1, const Vertex& v2);
double no_wall_between(const Vertex& v1, const Vertex& v2, const cv::Mat& im);
Vertex* nearest_vertex(const Vertex& qrand, const std::vector<Vertex*>& lv);
void rand_free_conf(Vertex& qrand, int height, int width);
Vertex* new_conf(const Vertex& qrand, Vertex& qnear, double *dq, const cv::Mat& im);
cv::Point2f vertex_to_point2f(const Vertex& v);
std::string type2str(int type);

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  Map map{NULL, info.width, info.height};
  map.data = new uint8_t(map.width*map.height);
  
  for (unsigned int i = 0; i < map.width*map.height; i++)
      map.data[i] = msg->data[i];
  // nav_msgs::OccupancyGrid* newGrid = map.Grid();
  // newGrid->header = header;
  // newGrid->info = info;
  // map_pub.publish(*newGrid);
}

static void onMouse( int event, int x, int y, int, void* )
{
  if( event != cv::EVENT_LBUTTONDOWN )
    return;
  targetX= x;
  targetY= y;

  std::cout << " x:" << x << "  y:" << y << std::endl;
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

int main(int argc, char* argv[])
{
  if(argc < 2)
  {
    std::cout << "Usage: ./test [image_name]" << std::endl;
    return 1;
  }
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;

  std::vector<Vertex*> vertices;
  Vertex qrand, *qnear, *qnew=NULL;
  cv::Mat image,emptyMap;

  ros::Subscriber subMap = n.subscribe("/map", 10, mapCallback);

  srand(time(NULL));

  image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
  emptyMap = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  int h = image.rows;
  int w = image.cols;

  cv::namedWindow("test map", cv::WINDOW_AUTOSIZE);
  cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  cv::setMouseCallback( "Display window", onMouse, 0 );
  
  // for(int i = 0 ; i < h ; i++)
  // {
  //   for(int j = 0 ; j < w ; j++)
  //   	std::cout << image.at<cv::Vec3b>(j, i) << std::endl;
  //   	// if(image.at<cv::Vec3b>(j, i) != cv::Vec3b(0,0,0))
  //   	// 	image.at<cv::Vec3b>(j, i) = cv::Vec3b(255,255,255);
  // }
//  Vertex v1={{30,30},NULL,0,0};
//  Vertex v2={{200,30},NULL,0,0};
//  double dist = sqrt(dist2(v1,v2));
//	std::cout << "dist:"<<dist << std::endl;
//	std::cout << "dist wall:"<<(double)no_wall_between(v1,v2, image) << std::endl;
//	cv::line(image, vertex_to_point2f(v1), vertex_to_point2f(v2), cv::Scalar(0,0,255), 1, CV_AA);
//	cv::imshow( "Display window", image );
//	cv::waitKey(0);
  vertices.push_back(new Vertex{{60.,60.},NULL,0,0});
  double dq = MAX_INC;

  cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ROBOT_RADIUS,ROBOT_RADIUS),cv::Point(-1,-1));
  cv::erode(emptyMap, emptyMap, se, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

  while(qnew == NULL || !(abs(qnew->data[0] - targetX)<10 && abs(qnew->data[1] - targetY)<10))
  {
    // std::cout << image.at<cv::Vec3b>(60,60) << std::endl;
    // std::cout << type2str(image.type()) << std::endl;
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
      //std::cout << dq << std::endl;
      dq = MAX_INC;
    }
    //std::cout << qnear << " " << (*qnew).parent<< " " << qnew << std::endl;
    //std::cout << "near x: " << qnear->data[0] << " near y: " << qnear->data[1] << std::endl;
    // std::cout << "new x: " << qnew->data[0] << " new y: " << qnew->data[1] << std::endl;
    vertices.push_back(qnew);
    cv::line(image, vertex_to_point2f(*qnear), vertex_to_point2f(*qnew), cv::Scalar(0,0,255), 1, CV_AA);
    cv::imshow( "Display window", image );                   // Show our image inside it.    
    cv::waitKey(10);
  }
  std::cout << "Path found. Lenght: "<< qnew->dist << std::endl;
  Vertex *parent=qnew->parent;
  std::vector<Vertex*> path(qnew->index+1);
  while(parent!=NULL)
  {
    path[qnew->index]=qnew;
//    std::cout << qnew->index << std::endl;
    cv::line(image, vertex_to_point2f(*parent), vertex_to_point2f(*qnew), cv::Scalar(0,255,0), 2, CV_AA);
    qnew=parent;
    parent=qnew->parent;
    cv::imshow( "Display window", image );
    //cv::waitKey(10);
  }
  path[qnew->index]=qnew;
//  
//  
//  
  //opti
  unsigned int i=0;
  while(i < path.size()-2)
  {
		for(unsigned int j=0; j < path.size()-i-3; j++)
		{
			int size = path.size();
			double dist = sqrt(dist2(*path[j],*path[size-i-1]));
			if(no_wall_between(*path[j],*path[size-i-1], emptyMap)>=dist)
			{
				cv::line(image, vertex_to_point2f(*path[j]), vertex_to_point2f(*path[size-i-1]), cv::Scalar(255,0,0), 2, CV_AA);
				cv::imshow( "Display window", image );
				path.erase(path.begin()+j+1, path.begin() + size-i-1);
				break;
			}
			cv::imshow( "Display window", image );
			cv::waitKey(10);
		}
		i++;
		cv::waitKey(10);
	}
	cv::imshow( "Display window", image );
  cv::waitKey(0);

//  if(! image.data )                              // Check for invalid input
//  {
//    std::cout <<  "Could not open or find the image" << std::endl ;
//    return -1;
//  }

  return 0;
}
