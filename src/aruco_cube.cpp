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

float stable_marker::m_size(){
	if(sliding_markers.size()>0)
		return sliding_markers.front().ssize;
	return -1;
}

float stable_marker::max_peri(){
	float max=0;
	for(list<Marker>::iterator i=sliding_markers.begin();i!=sliding_markers.end();i++){
		float peri=(*i).getPerimeter();
		if(peri>max)
			max=peri;
	}
	return max;
}

double stable_marker::variance_pos(){
	double tot=0;
	for(list<Marker>::iterator m = sliding_markers.begin();m!=sliding_markers.end();m++){
		tot+=1;
	}
	return tot;
}

void stable_marker::compute_Trans_rot(){
	if(sliding_markers.size()>0){
		Rodrigues(sliding_markers.front().Rvec,M_rot);
		M_trans=sliding_markers.front().Tvec;
	}
	else{
		M_trans=Mat::zeros(3,1,CV_32F);
		M_rot  =Mat::zeros(3,3,CV_32F);
	}
}

void stable_marker::compute_all(){
	compute_Trans_rot();
	double var_pos=variance_pos();
}


void stable_marker::aff_slid(Mat * current_image,CameraParameters CameraMatrix,double size_obj){
	if(sliding_markers.size()==0)
		return;

	//params
	vector<cv::Point3f> objectPoints;
	float size_2;
	if(size_obj<=0)
		size_2=last().ssize/2;
	else
		size_2=size_obj/2;
	float len_axe=size_2*4;
	//oeil cub
	objectPoints=Cadre3D(size_2);
	EasyPolyLine(current_image,Points3DtoCamPoints(objectPoints,M_rot,M_trans,CameraMatrix),
				 true,Scalar(0,255,255),2);
	//axes
	objectPoints=Axes3D(len_axe);
	EasyPolyLine(current_image,Points3DtoCamPoints(objectPoints,M_rot,M_trans,CameraMatrix),
			     true,Scalar(255,0,0),2);
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

aruco_cube::aruco_cube(int id_f,float c_size){
	id_front=id_f;
	int id=id_front;

	if(c_size<=0)
		c_size=DEFAULT_CUBE_SIZE;
	cube_size=c_size;

	Mat t_off = (Mat_<float>(3,1) << 0,0,-cube_size/2);
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

void aruco_cube::compute_T_R(){
	for(int i=0;i<FACE_CUBE_TOT;i++)cube[i].compute_Trans_rot();
	float tot=0;
	cube_trans=Mat::zeros(3,1,CV_32F);
	cube_rot  =Mat::zeros(3,3,CV_32F);
	for(int i=0;i<FACE_CUBE_TOT;i++){
		if( countNonZero( cube[i].M_rot!=Mat::zeros(3,3,CV_32F) ) > 0 ){
			cube_trans+=cube[i].M_trans;
			cube_rot  +=cube[i].M_rot;
			tot++;
		}
	}
	if(tot>0){
		cube_trans/=tot;
		cube_rot  /=tot;
	}
}

double aruco_cube::m_size(){
	for(int i=0; i<FACE_CUBE_TOT;i++){
		double size=cube[i].m_size();
		if(size>0)
			return size;
	}
	return -1;
}

float aruco_cube::max_peri(){
	float max=0;
	for(int i=0;i<FACE_CUBE_TOT;i++){
		float peri=cube[i].max_peri();
		if(peri>max)
			max=peri;
	}
	return max;
}

void aruco_cube::compute_all(){
	compute_T_R();
}


geometry_msgs::PoseStamped  aruco_cube::publish_marcker_pose(ros::Time stamp){
	if(countNonZero( cube_rot!=Mat::zeros(3,3,CV_32F) ) == 0 || !ros::ok()){
		geometry_msgs::PoseStamped nul;
		nul.header.frame_id="-1";
		return nul;
	}
	float x_t, y_t, z_t,roll,yaw,pitch;
	x_t =  cube_trans.at<Vec3f>(0,0)[0];
	y_t =  cube_trans.at<Vec3f>(0,0)[1];
	z_t =  cube_trans.at<Vec3f>(0,0)[2];

	yaw   =  atan2(cube_rot.at<float>(1,0), cube_rot.at<float>(0,0));
	if(abs(cube_rot.at<float>(2,2))>10e-3)
		roll  =  atan2(cube_rot.at<float>(2,1), cube_rot.at<float>(2,2));
	else
		roll  = M_PI*sgn( cube_rot.at<float>(2,1) );
	double square=pow( pow(cube_rot.at<float>(2,1),2)+
					   pow(cube_rot.at<float>(2,2),2) ,.5);
	pitch =  atan2(-cube_rot.at<float>(2,0), square);

	geometry_msgs::Quaternion p_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw	);

	// See: http://en.wikipedia.org/wiki/Flight_dynamics
#ifdef PRINT_POSE
	printf( "Angle >> roll: %5.3f pitch: %5.3f yaw: %5.3f \n", (roll)*(180.0/CV_PI), (pitch)*(180.0/CV_PI), (yaw)*(180.0/CV_PI));
	printf( "Dist. >>  x_d: %5.3f   y_d: %5.3f z_d: %5.3f \n", x_t, y_t, z_t);
#endif
	// Now publish the pose message, remember the offsets
	geometry_msgs::PoseStamped msg_ps;
	geometry_msgs::Pose pose;

	//m.id = id_front;
	msg_ps.header.frame_id =to_string(id_front);
	msg_ps.header.stamp = stamp;
	pose.position.x = x_t;
	pose.position.y = y_t;
	pose.position.z = z_t;
	pose.orientation = p_quat;
	msg_ps.pose = pose;
	return msg_ps;
}

void aruco_cube::aff_cube(Mat * current_image,CameraParameters CameraMatrix,bool unique){
	if(countNonZero( cube_rot!=Mat::zeros(3,3,CV_32F) ) == 0 )
		return;
	if(!unique){
		for(int i=0;i<FACE_CUBE_TOT;i++){
			cube[i].aff_slid(current_image,CameraMatrix,cube_size);
		}
	}

	vector<cv::Point3f>pts_aff=Cadre3D(cube_size/2);
	EasyPolyLine(current_image,Points3DtoCamPoints(pts_aff,cube_rot,cube_trans,CameraMatrix),true,Scalar(0,255,0),2);
	//axes
	pts_aff=Axes3D(cube_size*2);
	EasyPolyLine(current_image,Points3DtoCamPoints(pts_aff,cube_rot,cube_trans,CameraMatrix),true,Scalar(0,255,0),2);


	char id_str[3];
	sprintf(id_str,"%d",id_front);

	//procection 3D => 2D cam
	vector<cv::Point3f> objectPoints;
	objectPoints.push_back(Point3f(0     , 0      ,0     ));
	vector<Point2f> projectedPoints=Points3DtoCamPoints(objectPoints,cube_rot,cube_trans,CameraMatrix);
	Point2d coin_bas_gauche_text=projectedPoints[0];

	float peri=max_peri();
	coin_bas_gauche_text.x+=peri/8;
    putText(*current_image,id_str,coin_bas_gauche_text,
    		FONT_HERSHEY_SCRIPT_SIMPLEX,peri/4./50.,
			Scalar(0,0,255),3);

}


void cube_manager::push_back(aruco_cube aru_cub){
	cubes.push_back(aru_cub);
}

void cube_manager::update_marker(vector<Marker> vect_m){
	for(int i=0; i<cubes.size();i++)cubes[i].update_marker(vect_m);
}

void cube_manager::compute_all(){
	for(int i=0; i<cubes.size();i++)cubes[i].compute_all();
}

void  cube_manager::aff_cube(Mat * current_image,CameraParameters CameraMatrix,bool unique ){
	for(int i=0; i<cubes.size();i++)cubes[i].aff_cube(current_image,CameraMatrix,unique );
}

void  cube_manager::publish_marcker_pose(ros::Publisher pose_pub_markers,ros::Time stamp){
	for(int i=0; i<cubes.size();i++){
		geometry_msgs::PoseStamped msg=cubes[i].publish_marcker_pose(stamp);
		if(msg.header.frame_id!="-1")
			pose_pub_markers.publish(msg);
	}
}

