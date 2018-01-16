/*
 * RosSpectator.cpp
 *
 *  Created on: 16 janv. 2018
 *      Author: liehnfl
 */


//TODO add a correct use of ros time (real time is from ImConv)
//TODO if JeVois used create adapted define
//TODO add filtring function to delete/correct bad marker

#define PLOT

#include "RosSpectator.h"

int main(int argc,char **argv){
	if(argc!=2)
		throw std::invalid_argument(
				"Le nombre d'argument est mauvais.\nDonnez un yaml de config.");

	string node_name="marker_detection";
	node_name+=YAML::LoadFile(argv[1])["id_camera"].as<string>();

	ros::init(argc, argv, node_name);


	RosImageConverter ImConvertor= RosImageConverter( YAML::LoadFile(argv[1])["topic_in"].as<string>() );
	RosPublisherHandler publisher= RosPublisherHandler( YAML::LoadFile(argv[1])["id_camera"].as<int>()   ,
														   YAML::LoadFile(argv[1])["topic_out"].as<string>() );

	MarkerProcesser test(argv[1],&ImConvertor,&publisher);
	cout<<"Begin to Process!"<<endl;

	std::thread thread_opti(threadUseMaskOptimisation, &test);

	int key;Mat im;bool opt=false;
	while((key=waitKey(1))!='x' && key!=27){

		if(key=='o')
			opt^=true;

		test.DetectUpdateMaskPublish(opt,&im);
#ifdef PLOT
		imshow("main process",im);
		imshow("optiMask",test.OptimisationMask.getOptiMask());
#endif
	}
}



//-----------------ROS IM CONVERTER---------------------------------

RosImageConverter::RosImageConverter(string topic) : it_(nh_)
{
	r_mutex=new std::recursive_mutex ();
  // subscribe to input video feed and publish output video feed
	string default_top="/cam1";
	if(topic==""){
		return;
	}
	image_sub_ = it_.subscribe(topic, 1, &RosImageConverter::imageCb, this);


}

RosImageConverter::~RosImageConverter()
{
  image_sub_.shutdown();
  printf("\n>> ROS Stopped Image Import \n");
}

void RosImageConverter::getLastImage(cv::Mat *input_image) {
	(*r_mutex).lock();
	*input_image = src_img;
	(*r_mutex).unlock();
}

void RosImageConverter::getCurrentImage(cv::Mat *input_image) {
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

void RosImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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


//-----------------ROSPUBLISHER HANDLER---------------------------------

RosPublisherHandler::RosPublisherHandler(int id_cam,string topic){
	ros::NodeHandle n;
	pose_pub_markers= n.advertise<geometry_msgs::PoseStamped>(topic, 1);
	Cam_id=id_cam;
}

geometry_msgs::PoseStamped RosPublisherHandler::
TransformOneMarckerPose(Marker m)
{

	float x_t, y_t, z_t;
	x_t =  m.Tvec.at<Vec3f>(0,0)[0];
	y_t =  m.Tvec.at<Vec3f>(0,0)[1];
	z_t =  m.Tvec.at<Vec3f>(0,0)[2];

	tf::Quaternion tf_quat=Mat2Quaternion(m.Rvec);
	geometry_msgs::Quaternion quat;
 	tf::quaternionTFToMsg (tf_quat,quat);


	// See: http://en.wikipedia.org/wiki/Flight_dynamics
#ifdef PRINT_POSE
	cout<<"------------rotation-----------"<<endl;
	double roll,pitch,yaw;
	tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	printf( "Angle >> roll: %5.3f pitch: %5.3f yaw: %5.3f \n", (roll)*(180.0/CV_PI), (pitch)*(180.0/CV_PI), (yaw)*(180.0/CV_PI));
	printf( "Dist. >>  x_d: %5.3f   y_d: %5.3f z_d: %5.3f \n", x_t, y_t, z_t);
	cout<<"------------end rotation-----------"<<endl;
#endif
	// Now publish the pose message, remember the offsets
	geometry_msgs::PoseStamped msg_ps;
	geometry_msgs::Pose pose;

	//m.id = id_front;
	msg_ps.header.frame_id =to_string(Cam_id*CAM_FRAME_MULTIPLIOR+m.id*MARKER_FRAME_MULTIPLIOR);
	msg_ps.header.stamp = ros::Time::now();
	pose.position.x = x_t;
	pose.position.y = y_t;
	pose.position.z = z_t;
	pose.orientation = quat;
	msg_ps.pose = pose;
	return msg_ps;
}

void RosPublisherHandler::
publishMarckersPose(vector<Marker>markers){
	for(int i =0;i<markers.size();i++){
		geometry_msgs::PoseStamped msg=
				TransformOneMarckerPose(markers[i]);
		pose_pub_markers.publish(msg);
	}
}


