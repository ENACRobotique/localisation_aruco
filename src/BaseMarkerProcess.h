
#ifndef ARUCO_CUBE_H
#define ARUCO_CUBE_H

#ifdef RASPI
//Raspi gpio lib
#include <wiringPi.h>
#endif

// UTILS
#include <Utils.h>

//ROS
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//ROS IMAGE INTERFACE
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


// classe qui gère l'arrivée des images
class ImageHandler{
public:
	ros::Time timestamp;

	//Constructor, Destructor
	ImageHandler(string topic=""){}
	virtual ~ImageHandler(){}

	virtual void getCurrentImage(cv::Mat *input_image){return interfaceError();}
	virtual void getLastImage(cv::Mat *input_image){return interfaceError();}
};

#define OLDEST_OPTI_MASK 0.25
#define WATCHING_BOX_DIVIDER 2.25
//classe qui gère la fenêtre d'optimisation (fenêtre ou d'autres arucos on été observé récement!)
class OptiMask{
private:
	//attruibuts de fonctionnement
	std::recursive_mutex * r_mutex;
	list<Mat> sliding_mask;
	list<ros::Time> sliding_timestamp;
	Mat staticMask;

	//internal methods
	void cleanOldMask();
	Rect2d watchingBindingBox(Marker marker,Size im_size);

public:
	//Constructers
	OptiMask(){
		r_mutex=new std::recursive_mutex ();
	}
	OptiMask(Size im_size);

	void updateOptiMask(vector<Marker>markers);
	Mat getOptiMask();
};

//classe qui gère les sorties
class PublisherHandler{
public:
	PublisherHandler(){};
	virtual ~PublisherHandler(){};
	virtual void publishMarckersPose(vector<Marker>markers){return interfaceError();}
};

#define PLOT_AXIS_LENGHT 0.25
//classe qui vient superviser tout le traitement
class MarkerProcesser{
public:
	//THE tools to detect marker
	MarkerDetector MDetector;

	//Minimum parameters
	CameraParameters TheCameraParameters;
	float TheMarkerSize=-1;

	//Gestionaire de l'image
	ImageHandler* ImConv;

	//Masque d'optimisation
	OptiMask OptimisationMask;

	//Output Publisher
	PublisherHandler* publisher;
	std::recursive_mutex * r_save;

	//Constructeurs / Destructeurs
	MarkerProcesser(string yaml,ImageHandler* i,PublisherHandler* p);
	MarkerProcesser(CameraParameters cam_params,float mark_size,ImageHandler* i,PublisherHandler* p);
	~MarkerProcesser(){
		free(ImConv);
		free(publisher);
	}
	//Main Function!
	void DetectUpdateMaskPublish(bool Opti=false,Mat* plot=NULL);

	void getMaskedImage();
	void aff_markers(vector<Marker>markers,Mat *plot);

	void RunOpti();

};

void threadUseMaskOptimisation(MarkerProcesser *mark_process);

#endif