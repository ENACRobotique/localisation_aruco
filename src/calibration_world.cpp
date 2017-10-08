/*
 * calibration_aruco.cpp
 *
 *  Created on: 8 oct. 2017
 *      Author: liehnfl
 */

#include "aruco_cube.h"

#include <yaml-cpp/yaml.h>

bool allowed=true;
void sig_stop(int a)
{
	allowed=false;
}

int main(int argc,char **argv) {

    // ROS messaging init
	ros::init(argc, argv, "aruco_cube_publisher");

    ros::spinOnce();

    string ConfigFile;
    CameraParameters TheCameraParameters;
    string topic;
	float TheMarkerSize=-1;
	int CalibMarkerID=-1;

	if(argc!=2)throw invalid_argument( "any YAML given!" );
	ConfigFile=argv[1];
	YAML::Node config = YAML::LoadFile(ConfigFile);
	if(!config["marker_size"]||!config["topic"]||!config["marker_id"])
		throw invalid_argument( "the data YAML need more arguments!" );

	//read params
	TheMarkerSize=config["marker_size"].as<float>();
	topic="/"+config["topic"].as<string>();
	CalibMarkerID=config["marker_id"].as<int>();
	TheCameraParameters.readFromXMLFile(ConfigFile);

	ImageConverter image_getter(&topic);
	Mat current_image;
	while (current_image.empty()) {// wait the first image
		ros::spinOnce();
		image_getter.getCurrentImage(&current_image);
		usleep(1000);
	}
	TheCameraParameters.resize(current_image.size());

	MarkerDetector MDetector;
	vector<Marker>TheMarkers;
	//end config
	signal(SIGINT, sig_stop);
	char key;
#ifdef DEBUG
	cv::namedWindow("Calib_view", 1);
#endif

	while(allowed && (key != 'x') && (key != 27)&& ros::ok() ){

		image_getter.getCurrentImage(&current_image);
		MDetector.detect(current_image, TheMarkers, TheCameraParameters, TheMarkerSize);
		if(TheMarkers.size()==1&& TheMarkers[0].id==CalibMarkerID){

			TheMarkers[0].draw(current_image,Scalar(0,255,0),3,true);
			CvDrawingUtils::draw3dAxis(current_image, TheMarkers[0], TheCameraParameters);

		}


#ifdef DEBUG
		imshow("Calib_view",current_image);
#endif



		key=waitKey(1);

	}




}
