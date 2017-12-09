

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <thread>
#include <mutex>

// CV
#include "opencv2/opencv.hpp"
using namespace cv;

// ARUCO
#include "aruco.h"
using namespace aruco;


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

