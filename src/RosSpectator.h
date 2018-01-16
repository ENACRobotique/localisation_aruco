/*
 * RosSpectator.h
 *
 *  Created on: 16 janv. 2018
 *      Author: liehnfl
 */

#ifndef SRC_ROSSPECTATOR_H_
#define SRC_ROSSPECTATOR_H_


#include <BaseMarkerProcess.h>


// classe qui gère les images arrivant de ROS
class RosImageConverter:public ImageHandler{
  Mat src_img;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  std::recursive_mutex * r_mutex;

  ros::Time last_frame;

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
public:
  ros::Time timestamp;

  //Constructor, Destructor
  RosImageConverter(string topic="");
  ~RosImageConverter();

  void getCurrentImage(cv::Mat *input_image);
  void getLastImage(cv::Mat *input_image);


};

//Ros publisher
class RosPublisherHandler:public PublisherHandler{
private:
	ros::Publisher pose_pub_markers;
	int Cam_id=0;

	geometry_msgs::PoseStamped TransformOneMarckerPose(Marker m);
public:
	RosPublisherHandler(int id_cam,string topic="");
	void publishMarckersPose(vector<Marker>markers);
};


#endif /* SRC_ROSSPECTATOR_H_ */
