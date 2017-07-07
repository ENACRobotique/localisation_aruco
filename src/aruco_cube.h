
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

Mat Rotation33(double alpha,double beta,double gamma);

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
	void aff_cadre(Mat * current_image,Mat CameraMatrix);

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

	//affichage
	void aff_cube(Mat * current_image,Mat CameraMatrix);
};








#endif
