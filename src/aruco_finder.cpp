#include <aruco_finder.h>

int main(int argc,char **argv){
	if(argc!=2)
		throw std::invalid_argument(
				"Le nombre d'argument est mauvais.\nDonnez un yaml de config.");

	ros::init(argc, argv, "marker_detection");
	CameraParameters cam_params;
	string in_conf=(string)argv[1];
	cam_params.readFromXMLFile(in_conf);
	float mark_size=.067;
	int id_cam=1;
	string in_topic="cam1";
	string out_topic="markers";


	MarkerProcesser test(cam_params,
						 mark_size,
						 id_cam,
						 in_topic,
						 out_topic);
	cout<<"Begin to Process!"<<endl;
	int key;Mat im;
	bool opt=false;
	while((key=waitKey(1))!='x' &&
		   key!=27){
		if(key=='o')
			opt^=true;
		test.DetectUpdateMaskPublish(opt,&im);
		imshow("main process",im);
		imshow("optiMask",test.OptimisationMask.getOptiMask());
	}

}

//-----------------IM CONVERTER---------------------------------

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

//-----------------OPTI MASK---------------------------------

OptiMask::OptiMask(Size im_size)
	:OptiMask()
{
	staticMask=Mat(im_size, CV_8UC3, Scalar::all(0));
};

void OptiMask::cleanOldMask(){
	list<ros::Time>::iterator time = sliding_timestamp.end();
	while(sliding_timestamp.size()!=0 && time!=sliding_timestamp.begin()){
		time--;//pour choisir le temps précedent
		ros::Duration delta=ros::Time::now()-(*time);
		if(delta >ros::Duration(OLDEST_OPTI_MASK )){
			sliding_mask.pop_back();
			sliding_timestamp.pop_back();
		}
		else
			return;
	}
}

Rect2d OptiMask::
watchingBindingBox(Marker marker,Size im_size){
	float cote=marker.getPerimeter()/WATCHING_BOX_DIVIDER;
	Point2f centre=marker.getCenter();
	int x=max((int)(centre.x-cote),0);
	int y=max((int)(centre.y-cote),0);
	Rect2d res = Rect2d(x,
			  	  	    y,
			            min((int)(cote*2),im_size.width-x),
			            min((int)(cote*2),im_size.height-y));
	return res;
}

void OptiMask::
updateOptiMask(vector<Marker>markers){
	Mat new_mask;
	staticMask.copyTo(new_mask);
	for(int i=0;i<markers.size();i++){
		Rect2d box=watchingBindingBox(markers[i],
									staticMask.size());
		new_mask(box).setTo(Scalar::all(255));
	}
	sliding_mask.push_front(new_mask);
	sliding_timestamp.push_front(ros::Time::now());

	cleanOldMask();
};

Mat OptiMask::getOptiMask(){
	Mat res_mat;
	staticMask.copyTo(res_mat);
	Mat WhiteMask(staticMask.size(), CV_8UC3, Scalar::all(255));

	for(list<Mat>::iterator i=sliding_mask.begin();i!=sliding_mask.end();i++){
		//res_mat((*i)).setTo(Scalar::all(255));
		WhiteMask.copyTo(res_mat,(*i));
	}
	return res_mat;
}

//-----------------MARKER PROCESSER---------------------------------

MarkerProcesser::
MarkerProcesser(CameraParameters cam_params,float mark_size,int id_cam,string in_topic,string out_topic)
	:ImConv(&in_topic)
{
	//detect marker
	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
	//Minimum parameters
	Cam_id=id_cam;
	TheMarkerSize=mark_size;

	//wait a image
	Mat current_image;
	while (current_image.empty()) {
		ros::spinOnce();
		ImConv.getCurrentImage(&current_image);
		usleep(1000);
	}

	//Config Camera Params & Optimask
	cam_params.resize(current_image.size());
	TheCameraParameters=cam_params;
	OptimisationMask=OptiMask(current_image.size());

	//Output Publisher
	ros::NodeHandle n;
	pose_pub_markers= n.advertise<geometry_msgs::PoseStamped>(out_topic, 1);
}

void MarkerProcesser::
DetectUpdateMaskPublish(bool Opti,Mat* plot){

	//get im and mask if needed
	Mat current_im,trait_im;
	ImConv.getCurrentImage(&current_im);
	if(Opti){
		current_im.copyTo(trait_im,OptimisationMask.getOptiMask());
	}
	else{
		current_im.copyTo(trait_im);
	}
	//Detection
	vector<Marker>markers;
	MDetector.detect(trait_im, markers, TheCameraParameters, TheMarkerSize);

	//update optimask
	OptimisationMask.updateOptiMask(markers);
	//plot if needed
	if(plot!=NULL){
		trait_im.copyTo(*plot);
		aff_markers(markers,plot);
	}
	//publish
	publishMarckersPose(markers);
}

geometry_msgs::PoseStamped
MarkerProcesser::publishOneMarckerPose(Marker m)
{

	float x_t, y_t, z_t,roll,yaw,pitch;
	x_t =  m.Tvec.at<Vec3f>(0,0)[0];
	y_t =  m.Tvec.at<Vec3f>(0,0)[1];
	z_t =  m.Tvec.at<Vec3f>(0,0)[2];

	yaw   =  atan2(m.Rvec.at<float>(1,0), m.Rvec.at<float>(0,0));
	if(abs(m.Rvec.at<float>(2,2))>10e-3)
		roll  =  atan2(m.Rvec.at<float>(2,1), m.Rvec.at<float>(2,2));
	else
		roll  = M_PI*sgn( m.Rvec.at<float>(2,1) );
	double square=pow( pow(m.Rvec.at<float>(2,1),2)+
					   pow(m.Rvec.at<float>(2,2),2) ,.5);
	pitch =  atan2(-m.Rvec.at<float>(2,0), square);
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
	msg_ps.header.frame_id =to_string(Cam_id*1000+m.id);
	msg_ps.header.stamp = ros::Time::now();
	pose.position.x = x_t;
	pose.position.y = y_t;
	pose.position.z = z_t;
	pose.orientation = p_quat;
	msg_ps.pose = pose;
	return msg_ps;
}

void MarkerProcesser::
publishMarckersPose(vector<Marker>markers){
	for(int i =0;i<markers.size();i++){
		geometry_msgs::PoseStamped msg=
				publishOneMarckerPose(markers[i]);
		pose_pub_markers.publish(msg);
	}
}

void MarkerProcesser::
aff_markers(vector<Marker>markers,Mat *plot){
	if(plot==NULL)
		return;
	char id_str[3];
	Point2d coin_bas_gauche_text;
	for(int i=0;i<markers.size();i++){
		//axis
		CvDrawingUtils::draw3dAxis(*plot,
								TheCameraParameters,
								markers[i].Rvec,
								markers[i].Tvec,
								PLOT_AXIS_LENGHT);
		//id of marker
		sprintf(id_str,"%d",markers[i].id);
		Point2d coin_bas_droit_text=markers[i].getCenter()-Point2f(markers[i].getPerimeter()/10.,
																  -markers[i].getPerimeter()/10.);
		putText(*plot,id_str,coin_bas_droit_text,
		    		FONT_HERSHEY_SCRIPT_SIMPLEX,markers[i].getPerimeter()/4./50,
					Scalar(0,0,255),3);
	}
}

void MarkerProcesser::
RunOpti(){

}


//optimisation thread => use the opti mask
void threadUseMaskOptimisation(MarkerProcesser *mark_process){
	ros::Time ref= ros::TIME_MIN;
	while(true){
        ros::spinOnce();
		ros::Time test=ros::Time::now();
		mark_process->RunOpti();
		//cout<<"IN :"<<ros::Time::now()-test<<" ms"<<endl;
		ref=mark_process->ImConv.timestamp;
	}
}
