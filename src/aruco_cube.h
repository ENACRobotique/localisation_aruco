
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

//#define PRINT_POSE

template <typename T> inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Mat Rotation33(double alpha,double beta,double gamma);

vector<Point2f>Points3DtoCamPoints(vector<cv::Point3f> objectPoints,
								   Mat rot,Mat trans,
								   CameraParameters CameraMatrix);

vector<Point2i>PTS2FtoPTS2I(vector<Point2f> vect);

vector<Point3f>Cadre3D(float size);

vector<Point3f>Axes3D(float size);

void EasyPolyLine(Mat* im,vector<Point2f>ptsCam,bool closed=false,const Scalar color=Scalar::all(255),
                  int thickness=1, int lineType=8, int shift=0);

// classe qui gère l'arrivée des images
class ImageConverter{
  Mat src_img;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  std::recursive_mutex * r_mutex;

  void (*FunctionOnReception)(void)=NULL;

  ros::Time last_frame;
public:
  ros::Time timestamp;

  ImageConverter(string *topic);
  ImageConverter(string *topic, void (*FuncOnRecp)(void));

  ~ImageConverter();

  void getCurrentImage(cv::Mat *input_image);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

};

#define MAX_SLIDING_ARUCO 6

//classe des markers qui va nous permettre de faire le calcul de la variance
//et des offset pour chaque marker qui compose un cube
class stable_marker{
public:
	//attruibuts de fonctionnement
	list<Marker> sliding_markers;
	list<ros::Time> sliding_timestamp;
	int id =-1;
	//matrices de changement de repère pour tout traiter dans le référentiel cube
	//initialiser par le cube contenant. marker->cube
	Mat trans_offset=Mat::zeros(3,1,CV_32F);
	Mat rot_offset= Mat::zeros(3,3,CV_32F);

	//attruibuts de "sorties"
	Mat M_trans=Mat::zeros(3,1,CV_32F);
	Mat M_rot  =Mat::zeros(3,3,CV_32F);

	//constructeurs
	stable_marker(){};
	stable_marker(int id_m);
	stable_marker(int id_m,Mat t_off,Mat quat_off);//utiliser par la classe aruco_cube


	//ajout et récupération de markers
	void add_marker(Marker new_m,ros::Time time_marker=ros::Time::now());
	Marker last();
	void clean_old(ros::Duration delta_max);

	// calculs internes
	float m_size();
	float max_peri();
	Point2f CentreInIm();
	double variance_pos();//TODO
	void compute_Trans_rot();
	void compute_all();

	//affichage
	void aff_slid(Mat * current_image,CameraParameters CameraMatrix,double size_obj=-1);

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
#define DEFAULT_CUBE_SIZE 0.073 //en m
#define DEFAULT_USELESS_TIME .10 //en s => 10Hz

class aruco_cube{
public:
	//attribut de fonctionement
	int id_front=-1;
	stable_marker cube[FACE_CUBE_TOT];
	float cube_size;

	//Placement de la cam % au ref monde
	Mat rot_W2C=Mat::zeros(3,3,CV_32F);
	Mat tra_W2C=Mat::zeros(3,1,CV_32F);

	//attruibuts de "sorties"
	Mat cube_rotCam  =Mat::zeros(3,3,CV_32F);
	Mat cube_transCam=Mat::zeros(3,1,CV_32F);

	Mat cube_rotWorld  =Mat::zeros(3,3,CV_32F);
	Mat cube_transWorld=Mat::zeros(3,1,CV_32F);

	ros::Time current_time=ros::TIME_MIN;

	//constructeur
	aruco_cube();
	aruco_cube(int id_f,float c_size=-1);
	aruco_cube(int id_f,float c_size,Mat RotWorld2Cam, Mat TransWorld2Cam);

	//ajout marker
	void add_marker(Marker new_m,ros::Time time_marker=ros::Time::now());
	void update_marker(vector<Marker> vect_m,ros::Time time_marker=ros::Time::now());
	void clean_time_old(ros::Duration delta_max);

	//calculs
	double m_size();
	float max_peri();
	Point2f CentreInIm();
	Rect2d WatchingBindingBox(MatSize im_size);
	ros::Time newest_time();
	void compute_T_R();
	void reproject2world();
	void compute_all();
	geometry_msgs::PoseStamped  marcker_pose();

	//affichage
	void aff_cube(Mat * current_image,CameraParameters CameraMatrix,bool unique=false );
	void aff_world(Mat * current_image,CameraParameters CameraMatrix);//affiche la table de jeu
};

class cube_manager{
public:
	ImageConverter ImConv;
	Mat current_image;
	CameraParameters TheCameraParameters;
	float TheMarkerSize=-1;
	MarkerDetector MDetector;
	vector<aruco_cube>cubes;

	//Masque d'optimisation
	Mat OptimisationMask;

	void push_back(aruco_cube aru_cub);

	void update_current_image();
	void update_marker(vector<Marker> vect_m,ros::Time time_marker=ros::Time::now());
	void DetectUpdate(bool Opti=false);
	void compute_all();
	void publish_marcker_pose(ros::Publisher pose_pub_markers);

	void UpdateOptiMask();

	void aff_cube(bool unique=false );
	void aff_world();

	//Constructeur
	cube_manager(float MarkSize,CameraParameters CamPara,string *topic);
	cube_manager(float MarkSize,float cube_size,CameraParameters CamPara,string *topic,
			Mat rot_table,Mat tra_table,vector<int> cube_ids);
};

#endif
