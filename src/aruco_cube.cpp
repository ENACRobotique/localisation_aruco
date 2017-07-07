#include <aruco_cube.h>

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


stable_marker::stable_marker(int id_m){
	id=id_m;
}

stable_marker::stable_marker(int id_m,Mat t_off,Mat quat_off):stable_marker(id_m){
	if(t_off.size==trans_offset.size)
		trans_offset=t_off;
	if(quat_off.size==rot_offset.size)
		rot_offset=quat_off;
}

void stable_marker::add_marker(Marker new_m){
	if(new_m.id!=id)
		return;
	sliding_markers.push_front(new_m);
	sliding_timestamp.push_front(ros::Time::now());
	if(sliding_markers.size()>MAX_SLIDING_ARUCO){
		sliding_markers.pop_back();
		sliding_timestamp.pop_back();
	}

	//changement de Tvec pour prendre en compte l'offset
	Mat rot;
	Rodrigues(sliding_markers.front().Rvec,rot);
	sliding_markers.front().Tvec+=rot*trans_offset;

	//changement de Rvec pour prendre en compte l'offset
	rot*=rot_offset;
	Rodrigues(rot,sliding_markers.front().Rvec);


}

Marker stable_marker::last(){
	if(sliding_markers.size()>0)
		return sliding_markers.front();
	return Marker();
}

void stable_marker::clean_old(ros::Duration delta_max){
	list<ros::Time>::iterator time = sliding_timestamp.end();
	while(sliding_timestamp.size()!=0 && time!=sliding_timestamp.begin()){
		time--;//pour choisir le temps suivant
		ros::Duration delta=ros::Time::now()-(*time);
		if(delta >delta_max ){
			sliding_markers.pop_back();
			sliding_timestamp.pop_back();
		}
	}
}

double stable_marker::variance_pos(){
	double tot=0;
	for(auto m = sliding_markers.begin();m!=sliding_markers.end();m++){
		tot+=1;
	}
	return tot;
}


void stable_marker::aff_cadre(Mat * current_image,Mat CameraMatrix){
	if(sliding_markers.size()==0)
		return;


	vector<cv::Point3f> objectPoints;
	float size_2=last().ssize/2;
	float len_axe=size_2*4;
	objectPoints.push_back(Point3f(0     , 0      ,0     ));
	objectPoints.push_back(Point3f(size_2, size_2, size_2));
	objectPoints.push_back(Point3f(size_2,-size_2, size_2));
	objectPoints.push_back(Point3f(0     , 0      ,0     ));
	objectPoints.push_back(Point3f(size_2, size_2,-size_2));
	objectPoints.push_back(Point3f(size_2,-size_2,-size_2));
	int nb_pt_cam=objectPoints.size();
	objectPoints.push_back(Point3f(0      ,0      ,0      ));
	objectPoints.push_back(Point3f(len_axe,0      ,0      ));
	objectPoints.push_back(Point3f(0      ,0      ,0      ));
	objectPoints.push_back(Point3f(0      ,len_axe,0      ));
	objectPoints.push_back(Point3f(0      ,0      ,0      ));
	objectPoints.push_back(Point3f(0      ,0      ,len_axe));
	Mat distCoeffs(4,1,cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;
	vector<Point2f> projectedPoints;
	projectPoints(objectPoints, last().Rvec, last().Tvec ,
			CameraMatrix,distCoeffs,
			projectedPoints);

	vector<Point2i> printed_points,axes_points;
	for(size_t i=0;i<projectedPoints.size();i++)
	{
		if(i<nb_pt_cam)
			printed_points.push_back(Point2d(projectedPoints[i]));
		else
			axes_points.push_back(Point2d(projectedPoints[i]));
		//circle(*current_image, projectedPoints[i], 2,Scalar(255,0,255);, 4);
	}

	const Point* p = &(printed_points[0]);
	int n = (int)(printed_points.size());
	//dessin de la camera
	polylines(*current_image, &p, &n, 1, true, Scalar(0,255,255), 2);
	p = &(axes_points[0]);
	n = (int)(axes_points.size());
	polylines(*current_image, &p, &n, 1, false, Scalar(255,0,0), 2);



}

Mat Rot_Face(int FACE_CUBE ){
	switch(FACE_CUBE){
	case FACE_CUBE_FRONT:return Rotation33(-M_PI_2,0,-M_PI_2);
	case FACE_CUBE_LEFT :return Rotation33(-M_PI_2,0, M_PI  );
	case FACE_CUBE_BACK :return Rotation33(-M_PI_2,0, M_PI_2);
	case FACE_CUBE_RIGHT:return Rotation33(-M_PI_2,0,      0);
	case FACE_CUBE_UP   :return Rotation33(      0,0,-M_PI_2);
	default: 			 return Rotation33(      0,0,      0);
	}
}


aruco_cube::aruco_cube():aruco_cube(-1){}

aruco_cube::aruco_cube(int id_f){
	id_front=id_f;
	int id=id_front;

	Mat t_off = (Mat_<float>(3,1) << 0,0,-DEFAULT_MARKER_SIZE);
	Mat r_off;

	for(int i=0;i<FACE_CUBE_TOT;i++){
		if(id_front>0)
			id=id_front+i*DELTA_FACE;
		r_off = Rot_Face(i);
		cube[i]=stable_marker(id,t_off,r_off);
	}
}

void aruco_cube::add_marker(Marker new_m){
	if((new_m.id-id_front)%DELTA_FACE!=0 )
		return ;
	int id=(new_m.id-id_front)/DELTA_FACE;
	if(id>=FACE_CUBE_TOT )
		return ;
	cube[id].add_marker(new_m);
}

void aruco_cube::update_marker(vector<Marker> vect_m){
	for(int i=0; i<vect_m.size();i++){
		add_marker(vect_m[i]);
	}
	clean_time_old(ros::Duration(DEFAULT_USELESS_TIME));

}

void aruco_cube::clean_time_old(ros::Duration delta_max){
	for(int i=0; i<FACE_CUBE_TOT;i++){
		cube[i].clean_old(delta_max);
	}
}


