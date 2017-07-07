/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

ROS bits and integration added by Florian Lier flier at techfak dot uni-bielefeld dot de

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/


#include <aruco_cube.h>

using namespace aruco;
using namespace cv;

#define WIN_NAME "ROS ARUCO"
//#define THREADHOLD_VISU "THRESHOLD IMAGE"

template <typename T> inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class ImageConverter
{
  Mat src_img;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  std::recursive_mutex * r_mutex;


  ros::Time last_frame;
public:
  ros::Time timestamp;

  ImageConverter() : it_(nh_)
  {
    // subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cam1", 1, &ImageConverter::imageCb, this);
    r_mutex=new std::recursive_mutex ();
  }

  ImageConverter(string *topic) : it_(nh_)
  {
    // subscribe to input video feed and publish output video feed
	  string default_top="/cam1";
	if(topic==NULL){
		topic=&default_top;
	}
	image_sub_ = it_.subscribe(*topic, 1, &ImageConverter::imageCb, this);
	r_mutex=new std::recursive_mutex ();
  }

  ~ImageConverter()
  {
    image_sub_.shutdown();
    printf(">> ROS Stopped Image Import \n");
  }

  void getCurrentImage(cv::Mat *input_image) {
    while((timestamp.toSec() - last_frame.toSec()) <= 0) {
        usleep(500);
        ros::spinOnce();
    }
    (*r_mutex).lock();
    *input_image = src_img;
    last_frame = timestamp;
    (*r_mutex).unlock();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
  }

};

bool readArguments (string* top,string *File,float *Size, int argc,char **argv )
{
    if (argc<2) {
        cerr << ">>> Invalid number of arguments" << endl;
        cerr << ">>> Usage: (topic) [intrinsics.yml] [size]" <<endl;
        return false;
    }

    (*top)=argv[1];

    if (argc>=3)
    	(*File)=argv[2];
    if (argc>=4)
        (*Size)=atof(argv[3]);
    if (argc==3)
        cerr<< ">>> NOTE: You need makersize to see 3d info!" <<endl;

    return true;

}

bool allowed=true;
void sig_stop(int a)
{
	allowed=false;
}

int main(int argc,char **argv) {

	cv::Mat current_image;

	CameraParameters TheCameraParameters;
	MarkerDetector MDetector;
	vector<Marker> TheMarkers;
    aruco_cube test_cube(15);

    // ROS messaging init
	ros::init(argc, argv, "aruco_tf_publisher");
	ros::NodeHandle n;
    ros::spinOnce();

    int Thresmin,Thresmax;

    //var de gestion des entrées
	string topic;
	string TheIntrinsicFile;
	float TheMarkerSize=-1;
	if (readArguments(&topic,&TheIntrinsicFile, &TheMarkerSize, argc,argv)==false) {
		return 0;
	}
	string *topic_pointeur=NULL;
	if (topic != ""){
		topic="/"+topic;
		cout<<topic<<endl;
		topic_pointeur=&topic;
		//ic = ImageConverter();
	}
	ImageConverter ic  = ImageConverter(topic_pointeur);

    while (current_image.empty()) {
        ros::spinOnce();
        ic.getCurrentImage(&current_image);
        usleep(1000);
    }

	// Read camera parameters if passed
	if (TheIntrinsicFile != "") {
		TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
		TheCameraParameters.resize(current_image.size());
	}
	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
#ifdef DEBUG
	// Create gui
#ifdef THREADHOLD_VISU
	cv::namedWindow(THREADHOLD_VISU, 1);
#endif
	cv::namedWindow(WIN_NAME, 1);

	double inter1,inter2;
	MDetector.getThresholdParams(inter1, inter2);
	Thresmin=inter1;Thresmax=inter2;


	cv::createTrackbar("Thresmin", WIN_NAME, &Thresmin, 100);
	cv::createTrackbar("Thresmax", WIN_NAME, &Thresmax, 100);
#endif
	char key=0;

	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("aruco/pose", 1);
    ros::Publisher pose_pub_markers = n.advertise<visualization_msgs::MarkerArray>("/aruco/markerarray", 1);
	tf::TransformBroadcaster broadcaster;

	signal(SIGINT, sig_stop);
	// Capture until press ESC or until the end of the video
	while ((key != 'x') && (key != 27) && ros::ok()&& allowed) {

   		key = waitKey(1);

        ros::spinOnce();

        MDetector.setThresholdParams(max(Thresmin,3), max(Thresmax,3));

        ic.getCurrentImage(&current_image);

        if (current_image.empty()) {
            usleep(2000);
            cout << ">>> Image EMPTY" << endl;
            continue;
        }

        // Detection of markers in the image passed
        MDetector.detect(current_image, TheMarkers, TheCameraParameters, TheMarkerSize);
#ifdef THREADHOLD_VISU
        Mat threadhold_im=MDetector.getThresholdedImage();
        imshow(THREADHOLD_VISU,threadhold_im);
#endif

        test_cube.update_marker(TheMarkers);
        test_cube.compute_all();

        test_cube.aff_cube(&current_image,TheCameraParameters.CameraMatrix);

        float x_t, y_t, z_t;
        float roll,yaw,pitch;

        if ((countNonZero( test_cube.cube_rot!=Mat::zeros(3,3,CV_32F) ) != 0 )) {

            x_t = -test_cube.cube_trans.at<Vec3f>(0,0)[0];
            y_t =  test_cube.cube_trans.at<Vec3f>(0,0)[1];
            z_t =  test_cube.cube_trans.at<Vec3f>(0,0)[2];
            cv::Mat rot_mat=test_cube.cube_rot;

            yaw   =  atan2(rot_mat.at<float>(1,0), rot_mat.at<float>(0,0));
            if(abs(rot_mat.at<float>(2,2))>10e-3)
            	roll  =  atan2(rot_mat.at<float>(2,1), rot_mat.at<float>(2,2));
            else
            	roll  = M_PI*sgn( rot_mat.at<float>(2,1) );
			double square=pow( pow(rot_mat.at<float>(2,1),2)+
							   pow(rot_mat.at<float>(2,2),2) ,.5);
			pitch =  atan2(-rot_mat.at<float>(2,0), square);
			// See: http://en.wikipedia.org/wiki/Flight_dynamics


            printf( "Angle >> roll: %5.1f pitch: %5.1f yaw: %5.1f \n", (roll)*(180.0/CV_PI), (pitch)*(180.0/CV_PI), (yaw)*(180.0/CV_PI));
            printf( "Dist. >>  x_d: %5.1f   y_d: %5.1f z_d: %5.1f \n", x_t, y_t, z_t);

            geometry_msgs::Pose msg;
            geometry_msgs::PointStamped msg_ps;

            if (ros::ok()) {

                // Publish TF message including the offsets
                tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
                broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quat, tf::Vector3(x_t, y_t, z_t)), ros::Time::now(),"camera", "marker"));

                // Now publish the pose message, remember the offsets
                msg.position.x = x_t;
                msg.position.y = y_t;
                msg.position.z = z_t;
                geometry_msgs::Quaternion p_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw	);
                msg.orientation = p_quat;
                pose_pub.publish(msg);


                visualization_msgs::MarkerArray markers;
                visualization_msgs::Marker m;
                geometry_msgs::Pose pose;

                m.id = 0;
                m.header.frame_id = "aruco";
                m.header.stamp = ic.timestamp;
                pose.position.x = x_t;
                pose.position.y = y_t;
                pose.position.z = z_t;
                pose.orientation = p_quat;
                m.pose = pose;
                markers.markers.push_back(m);
                pose_pub_markers.publish(markers);

            }
        }

        // Show input with augmented information and the thresholded image
#ifdef DEBUG
        cv::imshow(WIN_NAME, current_image);
        //cv::imshow("THRESHOLD IMAGE", MDetector.getThresholdedImage());
#endif
        // Limit to 60hz
  		usleep(15000);

	}
}

