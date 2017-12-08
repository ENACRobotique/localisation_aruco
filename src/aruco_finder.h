
#ifndef ARUCO_CUBE_H
#define ARUCO_CUBE_H

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <thread>
#include <mutex>

#ifdef RASPI
//Raspi gpio lib
#include <wiringPi.h>
#endif

// UTILS
#include <Utils.h>

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


// classe qui gère l'arrivée des images
class ImageConverter{
  Mat src_img;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  std::recursive_mutex * r_mutex;

  ros::Time last_frame;
public:
  ros::Time timestamp;

  ImageConverter(string *topic=NULL);

  ~ImageConverter();

  void getCurrentImage(cv::Mat *input_image);
  bool newImage(cv::Mat *input_image);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

};

#define MAX_OPTI_WIN_HISTORY .10
//classe qui gère la fenêtre d'optimisation (fenêtre ou d'autres arucos on été observé récement!)
class OptiMask{
public:
	//attruibuts de fonctionnement
	std::recursive_mutex * r_mutex;
	list<Mat> sliding_mask;
	list<ros::Time> sliding_timestamp;

	void cleanOldMask();
public:
	Mat getOptiMask();
	void updateOptiMask(vector<Marker>markers){};

	OptiMask(){
		r_mutex=new std::recursive_mutex ();
	};
};

#define PLOT_AXIS_LENGHT 0.25
//classe qui vient superviser tout le traitement
class MarkerProcesser{
public:
	//THE tools to detect marker
	MarkerDetector MDetector;

	//Minimum parameters
	int Cam_id=0;
	CameraParameters TheCameraParameters;
	float TheMarkerSize=-1;

	//Gestionaire de l'image
	ImageConverter ImConv;

	//Masque d'optimisation
	OptiMask OptimisationMask;

	//Output Publisher
	ros::Publisher pose_pub_markers;

	//Constructeurs
	MarkerProcesser(CameraParameters cam_params,float mark_size,int id_cam,string in_topic,string out_topic);
	
	//Main Function!
	void DetectUpdateMaskPublish(bool Opti=false,Mat* plot=NULL);

	void getMaskedImage();
	void publishMarckersPose(vector<Marker>markers);
	void aff_markers(vector<Marker>markers,Mat *plot);

	void RunOpti();


};

void threadUseMaskOptimisation(MarkerProcesser *mark_process);

#endif
