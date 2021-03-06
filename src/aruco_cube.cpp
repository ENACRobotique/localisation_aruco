#include "../includes/aruco_cube.h"

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

void stable_marker::add_marker(Marker new_m,ros::Time time_marker){
	if(new_m.id!=id)
		return;

	//Verification que l'axe Z est visible (filtrage des arucos déformés)
	Mat rot;
	Rodrigues(new_m.Rvec,rot);
	Mat Mtest=rot*(Mat_<float>(3, 1 ) <<0,0,1);
	if(Mtest.at<float>(2,0)>-0.2){//check if marker is wrong
		return;//TODO correct it and use it
	}
	//Ajout du marker
	sliding_markers.push_front(new_m);
	sliding_timestamp.push_front(time_marker);
	if(sliding_markers.size()>MAX_SLIDING_ARUCO){
		sliding_markers.pop_back();
		sliding_timestamp.pop_back();
	}

	//changement de Tvec pour prendre en compte l'offset
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


Point2f stable_marker::CentreInIm(){
	Point2f res=Point2f(0,0);
	for(list<Marker>::iterator i=sliding_markers.begin();i!=sliding_markers.end();i++){
		res+=(*i).getCenter();
	}
	if(sliding_markers.size()!=0)
		res/=(int)sliding_markers.size();
	return res;
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

aruco_cube::aruco_cube(int id_f,float c_size,Mat RotWorld2Cam, Mat TransWorld2Cam)
			:aruco_cube(id_f,c_size){
	rot_W2C = RotWorld2Cam;
	tra_W2C = TransWorld2Cam;
}

void aruco_cube::add_marker(Marker new_m,ros::Time time_marker){
	if((new_m.id-id_front)%DELTA_FACE!=0 )
		return ;
	int id=(new_m.id-id_front)/DELTA_FACE;
	if(id>=FACE_CUBE_TOT )
		return ;
	cube[id].add_marker(new_m,time_marker);
}

void aruco_cube::update_marker(vector<Marker> vect_m,ros::Time time_marker){
	for(int i=0; i<vect_m.size();i++){
		add_marker(vect_m[i],time_marker);
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
	cube_transCam=Mat::zeros(3,1,CV_32F);
	cube_rotCam  =Mat::zeros(3,3,CV_32F);
	for(int i=0;i<FACE_CUBE_TOT;i++){
		if( countNonZero( cube[i].M_rot!=Mat::zeros(3,3,CV_32F) ) > 0 ){
			cube_transCam+=cube[i].M_trans;
			cube_rotCam  +=cube[i].M_rot;
			tot++;
		}
	}
	if(tot>0){
		cube_transCam/=tot;
		cube_rotCam  /=tot;
	}
}

void aruco_cube::reproject2world(){
	cube_transWorld=rot_W2C.inv()*(cube_transCam-tra_W2C);
	cube_rotWorld  =rot_W2C.inv()* cube_rotCam;
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

Point2f aruco_cube::CentreInIm(){
	Point2f res=Point2f(0,0);
	int compt=0;
	for(int i=0 ;i<FACE_CUBE_TOT;i++){
		Point2f inter=cube[i].CentreInIm();
		if(inter!=Point2f(0,0)){
			compt++;
			res+=inter;
		}
	}
	if(compt!=0)
		res/=compt;
	return res;
}

Rect2d  aruco_cube::WatchingBindingBox(MatSize im_size){
	float cote=max_peri()/2;
	Point2f centre=CentreInIm();
	int x=max((int)(centre.x-cote),0);
	int y=max((int)(centre.y-cote),0);
	Rect2d res = Rect2d(x,
			  	  	    y,
			            min((int)(cote*2),im_size[1]-x),
			            min((int)(cote*2),im_size[0]-y));
	return res;
}


ros::Time aruco_cube::newest_time(){
	ros::Time res=ros::TIME_MIN;

	for(int i=0;i<FACE_CUBE_TOT;i++){
		if(cube[i].sliding_timestamp.size()>0){
			if( res.toSec()<cube[i].sliding_timestamp.front().toSec() )
				res= cube[i].sliding_timestamp.front();
		}
	}
	return res;
}

void aruco_cube::compute_all(){
	ros::Time time_comp=newest_time();
	if(time_comp!=ros::TIME_MIN)
		current_time=time_comp;

	compute_T_R();
	reproject2world();
}


geometry_msgs::PoseStamped  aruco_cube::marcker_pose(){
	if(countNonZero( cube_rotWorld!=Mat::zeros(3,3,CV_32F) ) == 0 || !ros::ok()){
		geometry_msgs::PoseStamped nul;
		nul.header.frame_id="-1";
		return nul;
	}
	float x_t, y_t, z_t,roll,yaw,pitch;
	x_t =  cube_transWorld.at<Vec3f>(0,0)[0];
	y_t =  cube_transWorld.at<Vec3f>(0,0)[1];
	z_t =  cube_transWorld.at<Vec3f>(0,0)[2];

	yaw   =  atan2(cube_rotWorld.at<float>(1,0), cube_rotWorld.at<float>(0,0));
	if(abs(cube_rotWorld.at<float>(2,2))>10e-3)
		roll  =  atan2(cube_rotWorld.at<float>(2,1), cube_rotWorld.at<float>(2,2));
	else
		roll  = M_PI*sgn( cube_rotWorld.at<float>(2,1) );
	double square=pow( pow(cube_rotWorld.at<float>(2,1),2)+
					   pow(cube_rotWorld.at<float>(2,2),2) ,.5);
	pitch =  atan2(-cube_rotWorld.at<float>(2,0), square);

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
	msg_ps.header.stamp = current_time;
	pose.position.x = x_t;
	pose.position.y = y_t;
	pose.position.z = z_t;
	pose.orientation = p_quat;
	msg_ps.pose = pose;
	return msg_ps;
}

void aruco_cube::aff_cube(Mat * current_image,CameraParameters CameraMatrix,bool unique){
	if(countNonZero( cube_rotCam!=Mat::zeros(3,3,CV_32F) ) == 0 )
		return;
	if(!unique){
		for(int i=0;i<FACE_CUBE_TOT;i++){
			cube[i].aff_slid(current_image,CameraMatrix,cube_size);
		}
	}

	vector<cv::Point3f>pts_aff=Cadre3D(cube_size/2);
	EasyPolyLine(current_image,Points3DtoCamPoints(pts_aff,cube_rotCam,cube_transCam,CameraMatrix),true,Scalar(0,255,0),2);
	//axes
	pts_aff=Axes3D(cube_size*2);
	EasyPolyLine(current_image,Points3DtoCamPoints(pts_aff,cube_rotCam,cube_transCam,CameraMatrix),true,Scalar(0,255,0),2);


	char id_str[3];
	sprintf(id_str,"%d",id_front);

	//procection 3D => 2D cam
	vector<cv::Point3f> objectPoints;
	objectPoints.push_back(Point3f(0     , 0      ,0     ));
	vector<Point2f> projectedPoints=Points3DtoCamPoints(objectPoints,cube_rotCam,cube_transCam,CameraMatrix);
	Point2d coin_bas_gauche_text=projectedPoints[0];

	float peri=max_peri();
	coin_bas_gauche_text.x+=peri/8;
    putText(*current_image,id_str,coin_bas_gauche_text,
    		FONT_HERSHEY_SCRIPT_SIMPLEX,peri/4./50.,
			Scalar(0,0,255),3);

}

void aruco_cube::aff_world(Mat * current_image,CameraParameters CameraMatrix){
	vector<cv::Point3f> table;
	table.push_back(Point3f(3,0,0));
	table.push_back(Point3f(3,2,0));
	table.push_back(Point3f(3,2,1));
	table.push_back(Point3f(3,2,0));
	table.push_back(Point3f(0,2,0));
	EasyPolyLine(current_image,Points3DtoCamPoints(table,rot_W2C,tra_W2C,CameraMatrix),
					 false,Scalar(255,0,255),1);

}

void cube_manager::push_back(aruco_cube aru_cub){
	cubes.push_back(aru_cub);
}


void cube_manager::update_current_image(){
	Mat inter;
	ImConv.getCurrentImage(&inter);
	lock.lock();
	current_image=inter;
	lock.unlock();

	if (current_image.empty()) {
		cout << ">>> Image EMPTY" << endl;
	}

}

void cube_manager::update_marker(vector<Marker> vect_m,ros::Time time_marker){
	lock.lock();
	for(int i=0; i<cubes.size();i++)cubes[i].update_marker(vect_m,time_marker);
	lock.unlock();
}

void cube_manager::DetectUpdate(bool Opti){
	vector<Marker> TheMarkers;
	Mat traitement_im;

	lock.lock();//récupération des données et calcul
	if(current_image.empty()){
		lock.unlock();
		return;
	}
	ros::Time im_time = ImConv.timestamp;
	if(Opti)
		current_image.copyTo(traitement_im,OptimisationMask);
	else
		current_image.copyTo(traitement_im);
    MDetector.detect(traitement_im, TheMarkers, TheCameraParameters, TheMarkerSize);
    lock.unlock();
    update_marker(TheMarkers,im_time);
    UpdateOptiMask();
}

void cube_manager::compute_all(){
	lock.lock();
	for(int i=0; i<cubes.size();i++)cubes[i].compute_all();
	lock.unlock();
}

void  cube_manager::aff_cube(bool unique ){
	lock.lock();
	for(int i=0; i<cubes.size();i++)cubes[i].aff_cube(&current_image,TheCameraParameters,unique );
	lock.unlock();
}

void cube_manager::aff_world(){
	lock.lock();
	if(cubes.size()==0){
		lock.unlock();
		return;
	}
	cubes[0].aff_world(&current_image,TheCameraParameters);
	lock.unlock();
}

cube_manager::cube_manager(float MarkSize,CameraParameters CamPara,string *topic,bool opti)
					:ImConv(topic){
	TheMarkerSize=MarkSize;

	//wait a image
	Mat current_image;
	while (current_image.empty()) {
		ros::spinOnce();
		ImConv.getCurrentImage(&current_image);
		usleep(1000);
	}

	//Config Camera Params & Optimask
	CamPara.resize(current_image.size());
	TheCameraParameters=CamPara;
	Mat mask(TheCameraParameters.CamSize, CV_8UC3, Scalar::all(0));
	mask.copyTo(OptimisationMask);

	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
}


cube_manager::
cube_manager(float MarkSize,float cube_size,CameraParameters CamPara,string *topic,
		     Mat rot_table,Mat tra_table,vector<int> cube_ids,bool opti)
						:cube_manager(MarkSize,CamPara,topic,opti){
	for(int i=0;i<cube_ids.size();i++){
		cubes.push_back(aruco_cube(cube_ids[i],cube_size,rot_table,tra_table));
	}
}


void  cube_manager::publish_marcker_pose(ros::Publisher pose_pub_markers){
	lock.lock();
	for(int i=0; i<cubes.size();i++){
		geometry_msgs::PoseStamped msg=cubes[i].marcker_pose();
		if(msg.header.frame_id!="-1")
			pose_pub_markers.publish(msg);
	}
	ros::spinOnce();
	lock.unlock();
}

void  cube_manager::UpdateOptiMask(){

	Mat mask(OptimisationMask.size(), CV_8UC3, Scalar::all(0));

	for(int i=0; i< cubes.size();i++){
		Rect2d box=cubes[i].WatchingBindingBox(OptimisationMask.size);
		mask(box).setTo(Scalar::all(255));
	}
	lock.lock();
	mask.copyTo(OptimisationMask);
	lock.unlock();
}

void  cube_manager::RunOpti(ros::Publisher pose_pub_markers){

    DetectUpdate(true);//on détect mais de manière optimisé!
    compute_all();
    publish_marcker_pose(pose_pub_markers);
}

ImageConverter::ImageConverter(string *topic) : it_(nh_)
{
  // subscribe to input video feed and publish output video feed
	string default_top="/cam1";
	if(topic==NULL){
		topic=&default_top;
	}
	image_sub_ = it_.subscribe(*topic, 1, &ImageConverter::imageCb, this);
	r_mutex=new std::recursive_mutex ();

}

ImageConverter::~ImageConverter()
{
  image_sub_.shutdown();
  printf("\n>> ROS Stopped Image Import \n");
}

void ImageConverter::getCurrentImage(cv::Mat *input_image) {
	int count=0;
	while((timestamp.toSec() - last_frame.toSec()) <= 0) {
		usleep(100);
		ros::spinOnce();
		count++;
		if(count>15000){//15.000*0.1ms=1.5s
			throw invalid_argument( "No Image on the node" );
		}
	}
	(*r_mutex).lock();
	*input_image = src_img;
	last_frame = timestamp;
	(*r_mutex).unlock();
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  ros::Time frame_time = ros::Time::now();
  timestamp = frame_time;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    (*r_mutex).unlock();
    return;
  }
  (*r_mutex).lock();
  src_img = cv_ptr->image;
  (*r_mutex).unlock();

  return ;
}

void threadOptimisation(cube_manager* c_manager,ros::Publisher pose_pub_markers){
	ros::Time ref= ros::TIME_MIN;
	while(true){
        ros::spinOnce();
		c_manager->update_current_image();
		ros::Time test=ros::Time::now();
		c_manager->RunOpti(pose_pub_markers);
		//cout<<"IN :"<<ros::Time::now()-test<<" ms"<<endl;
		ref=c_manager->ImConv.timestamp;
	}
}

