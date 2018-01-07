#include <Utils.h>


tf::Quaternion Mat2Quaternion(Mat r){

	if(r.size()!=Size(3,3))
		Rodrigues(r,r);//transform vect to matrix
	double roll,yaw,pitch;
	yaw   =  atan2(r.at<float>(1,0), r.at<float>(0,0));
	if(abs(r.at<float>(2,2))>10e-3)
		roll  =  atan2(r.at<float>(2,1), r.at<float>(2,2));
	else
		roll  = M_PI*sgn( r.at<float>(2,1) );
	double square=pow( pow(r.at<float>(2,1),2)+
					   pow(r.at<float>(2,2),2) ,.5);
	pitch =  atan2(-r.at<float>(2,0), square);
	return tf::createQuaternionFromRPY(roll, pitch, yaw);
}


Mat Rotation33(double alpha,double beta,double gamma){
    // Rotation matrices around the X, Y, and Z axis
    Mat RX = (Mat_<float>(3, 3 ) <<
              1,          0,           0,
              0, cos(alpha), -sin(alpha),
              0, sin(alpha),  cos(alpha),
              0,          0,           0);

    Mat RY = (Mat_<float>(3, 3 ) <<
              cos(beta),  0,  -sin(beta),
              0, 		  1,           0,
              sin(beta),  0,   cos(beta),
              0, 		  0,           0);

    Mat RZ = (Mat_<float>(3, 3 ) <<
              cos(gamma), -sin(gamma), 0,
              sin(gamma),  cos(gamma), 0,
              0,          0,           1,
              0,          0,           0);

    // Composed rotation matrix with (RX, RY, RZ)
    return RX * RY * RZ;
}

vector<Point2f>Points3DtoCamPoints(vector<cv::Point3f> objectPoints,
								   Mat rot,Mat trans,
								   CameraParameters CameraMatrix){
	Mat vect_rot;
	rot.copyTo(vect_rot);
	if(vect_rot.size()==Size(3,3))
		Rodrigues(vect_rot,vect_rot);

	vector<Point2f> projectedPoints;
	projectPoints(objectPoints, vect_rot, trans ,CameraMatrix.CameraMatrix,CameraMatrix.Distorsion,projectedPoints);
	return projectedPoints;

}


vector<Point2i>PTS2FtoPTS2I(vector<Point2f> vect){
	vector<Point2i>res;
	for(int i =0;i<vect.size();i++)res.push_back(Point2i(vect[i]));
	return res;
}

vector<Point3f>Cadre3D(float size){
	vector<cv::Point3f> objectPoints;
	//oeil cube
	objectPoints.push_back(Point3f(0   , 0    ,0   ));
	objectPoints.push_back(Point3f(size, size, size));
	objectPoints.push_back(Point3f(size,-size, size));
	objectPoints.push_back(Point3f(0   , 0    ,0   ));
	objectPoints.push_back(Point3f(size, size,-size));
	objectPoints.push_back(Point3f(size,-size,-size));
	return objectPoints;
}

vector<Point3f>Axes3D(float size){
	vector<cv::Point3f> objectPoints;
	//axes
	objectPoints.push_back(Point3f(0   ,0   ,0   ));
	objectPoints.push_back(Point3f(size,0   ,0   ));
	objectPoints.push_back(Point3f(0   ,0   ,0   ));
	objectPoints.push_back(Point3f(0   ,size,0   ));
	objectPoints.push_back(Point3f(0   ,0   ,0   ));
	objectPoints.push_back(Point3f(0   ,0   ,size));
	return objectPoints;
}

void EasyPolyLine(Mat* im,vector<Point2f>ptsCam,bool closed,const Scalar color,
                  int thickness, int lineType, int shift){
	vector<Point2i> pts=PTS2FtoPTS2I(ptsCam);
	const Point* p = &(pts[0]);
	int n = (int)(pts.size());
	polylines(*im, &p, &n, 1, closed, color, thickness,lineType, shift);

}

void interfaceError(){
	throw std::invalid_argument(
			"Il faut redéfinir les méthodes getCurrentImage et getLastImage!");
}
