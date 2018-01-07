
#ifndef ARUCO_CUBE_H
#define ARUCO_CUBE_H

#ifdef RASPI
//Raspi gpio lib
#include <wiringPi.h>
#endif

// UTILS
#include <Utils.h>



// classe qui gère l'arrivée des images
class ImageHandler{
public:
	ros::Time timestamp;

	//Constructor, Destructor
	ImageHandler(string topic=""){};
	virtual ~ImageHandler(){};

	virtual void getCurrentImage(cv::Mat *input_image){
		throw std::invalid_argument(
		"Il faut redéfinir les méthodes getCurrentImage et getLastImage!");
	};
	virtual void getLastImage(cv::Mat *input_image){};
};

// classe qui gère les images arrivant de ROS
class RosImageConverter:public ImageHandler{
  Mat src_img;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  std::recursive_mutex * r_mutex;

  ros::Time last_frame;

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
public:
  ros::Time timestamp;

  //Constructor, Destructor
  RosImageConverter(string topic="");
  ~RosImageConverter();

  void getCurrentImage(cv::Mat *input_image);
  void getLastImage(cv::Mat *input_image);


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
	ImageHandler* ImConv;

	//Masque d'optimisation
	OptiMask OptimisationMask;

	//Output Publisher
	ros::Publisher pose_pub_markers;
	std::recursive_mutex * r_save;

	//Constructeurs
	MarkerProcesser(string yaml);
	MarkerProcesser(CameraParameters cam_params,float mark_size,int id_cam,string in_topic,string out_topic);
	
	//Main Function!
	void DetectUpdateMaskPublish(bool Opti=false,Mat* plot=NULL);

	void getMaskedImage();
	void publishMarckersPose(vector<Marker>markers);
	void aff_markers(vector<Marker>markers,Mat *plot);

	void RunOpti();

private:
	geometry_msgs::PoseStamped publishOneMarckerPose(Marker m);
};

void threadUseMaskOptimisation(MarkerProcesser *mark_process);

#endif
