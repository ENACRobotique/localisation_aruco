
// CV
#include "cvdrawingutils.h"
#include "opencv2/opencv.hpp"
using namespace cv;

// ARUCO
#include "aruco.h"
using namespace aruco;

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

