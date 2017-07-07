
#ifndef ARUCO_CUBE_H
#define ARUCO_CUBE_H

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <mutex>

// ARUCO
#include "aruco.h"

// CV
#include "cvdrawingutils.h"
#include "opencv2/opencv.hpp"

// ROS
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <signal.h>

using namespace aruco;
using namespace cv;

template <typename T> inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Mat Rotation33(double alpha,double beta,double gamma);

vector<Point2f>Points3DtoCamPoints(vector<cv::Point3f> objectPoints,
								   Mat rot,Mat trans,
								   Mat CameraMatrix,Mat *distCoeffs=NULL);

vector<Point2i>PTS2FtoPTS2I(vector<Point2f> vect);

vector<Point3f>Cadre3D(float size);

vector<Point3f>Axes3D(float size);

void EasyPolyLine(Mat* im,vector<Point2f>ptsCam,bool closed=false,const Scalar color=Scalar::all(255),
                  int thickness=1, int lineType=8, int shift=0);

#define MAX_SLIDING_ARUCO 6

//classe des markers qui va nous permettre de faire le calcul de la variance
//et des offset pour chaque marker qui compose un cube
class stable_marker{
public:
	//attruibuts de fonctionnement
	list<Marker> sliding_markers;
	list<ros::Time> sliding_timestamp;
	int id =-1;
	Mat trans_offset=Mat::zeros(3,1,CV_32F);
	Mat rot_offset= Mat::zeros(3,3,CV_32F);

	//attruibuts de "sorties"
	Mat M_trans=Mat::zeros(3,1,CV_32F);
	Mat M_rot  =Mat::zeros(3,3,CV_32F);

	//constructeurs
	stable_marker(){};
	stable_marker(int id_m);
	stable_marker(int id_m,Mat t_off,Mat quat_off);


	//ajout et récupération de markers
	void add_marker(Marker new_m);
	Marker last();
	void clean_old(ros::Duration delta_max);

	// calculs internes
	float m_size();
	float max_peri();
	double variance_pos();
	void compute_Trans_rot();
	void compute_all();

	//affichage
	void aff_slid(Mat * current_image,Mat CameraMatrix);

};

enum{
	FACE_CUBE_FRONT,
	FACE_CUBE_LEFT,
	FACE_CUBE_BACK,
	FACE_CUBE_RIGHT,
	FACE_CUBE_UP,
	FACE_CUBE_TOT
}FACE_CUBE;
Mat Rot_Face(int FACE_CUBE );

#define DELTA_FACE 10
#define DEFAULT_MARKER_SIZE 0.08
#define DEFAULT_USELESS_TIME .10 //en s

class aruco_cube{
public:
	//attribut de fonctionement
	int id_front=-1;
	stable_marker cube[FACE_CUBE_TOT];

	//attruibuts de "sorties"
	Mat cube_trans=Mat::zeros(3,1,CV_32F);
	Mat cube_rot  =Mat::zeros(3,3,CV_32F);

	//constructeur
	aruco_cube();
	aruco_cube(int id_f);

	//ajout marker
	void add_marker(Marker new_m);
	void update_marker(vector<Marker> vect_m);
	void clean_time_old(ros::Duration delta_max);

	//calculs
	double m_size();
	float max_peri();
	void compute_T_R();
	void compute_all();
	void publish_marcker_pose(ros::Publisher pose_pub_markers,ros::Time stamp);

	//affichage
	void aff_cube(Mat * current_image,Mat CameraMatrix,bool unique=false );
};

// classe qui gère l'arriver des images
class ImageConverter
{
  Mat src_img;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  std::recursive_mutex * r_mutex;


  ros::Time last_frame;
public:
  ros::Time timestamp;

  ImageConverter() : it_(nh_)
  {
    // subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cam1", 1, &ImageConverter::imageCb, this);
    r_mutex=new std::recursive_mutex ();
  }

  ImageConverter(string *topic) : it_(nh_)
  {
    // subscribe to input video feed and publish output video feed
	  string default_top="/cam1";
	if(topic==NULL){
		topic=&default_top;
	}
	image_sub_ = it_.subscribe(*topic, 1, &ImageConverter::imageCb, this);
	r_mutex=new std::recursive_mutex ();
  }

  ~ImageConverter()
  {
    image_sub_.shutdown();
    printf(">> ROS Stopped Image Import \n");
  }

  void getCurrentImage(cv::Mat *input_image) {
    while((timestamp.toSec() - last_frame.toSec()) <= 0) {
        usleep(500);
        ros::spinOnce();
    }
    (*r_mutex).lock();
    *input_image = src_img;
    last_frame = timestamp;
    (*r_mutex).unlock();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Time frame_time = ros::Time::now();
    timestamp = frame_time;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      (*r_mutex).unlock();
      return;
    }
    (*r_mutex).lock();
    src_img = cv_ptr->image;
    (*r_mutex).unlock();
  }

};

#endif
