
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

#define MAX_SLIDING_ARUCO 5

//classe des markers qui va nous permettre de faire le calcul de la variance
//et des offset pour chaque marker qui compose un cube
class stable_marker{
public:
	//attruibuts
	list<Marker> sliding_markers;
	int id =-1;
	Mat trans_offset=Mat::zeros(3,1,CV_32F);
	Mat rot_offset= Mat::zeros(3,3,CV_32F);

	//constructeurs
	stable_marker(){};
	stable_marker(int id_m);
	stable_marker(int id_m,Mat t_off,Mat quat_off);


	//ajout et récupération de markers
	void add_marker(Marker new_m);
	Marker last();

	// calculs internes
	double variance_pos();

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

class aruco_cube{
public:
	//attribut
	int id_front=-1;
	stable_marker cube[FACE_CUBE_TOT];

	//constructeur
	aruco_cube();
	aruco_cube(int id_f);

	//ajout marker
	void add_marker(Marker new_m);
	void update_marker(vector<Marker> vect_m);

	//affichage

};








#endif
