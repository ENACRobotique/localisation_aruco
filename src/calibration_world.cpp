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
	Mat rotM2W, transM2W;

	if(argc!=2)throw invalid_argument( "any YAML given!" );
	ConfigFile=argv[1];
	YAML::Node config = YAML::LoadFile(ConfigFile);
	FileStorage fs2(ConfigFile, FileStorage::READ);
	if(!config["marker_size_calib"]||!config["topic"]||!config["marker_id"]||
	   fs2["rotMarker2World"  ].empty()||fs2["transMarker2World"].empty())
		throw invalid_argument( "the data YAML need more arguments!" );

	//read params
	TheMarkerSize=config["marker_size_calib"].as<float>();
	topic="/"+config["topic"].as<string>();
	CalibMarkerID=config["marker_id"].as<int>();
	TheCameraParameters.readFromXMLFile(ConfigFile);
	fs2["rotMarker2World"  ]>>rotM2W;
	fs2["transMarker2World"]>>transM2W;
	rotM2W.convertTo(rotM2W,CV_32F);
	transM2W.convertTo(transM2W,CV_32F);
	Mat transCenter2RightDown=(Mat_<float>(3, 1 ) <<-TheMarkerSize/2,-TheMarkerSize/2,0);


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
	char key;int id_mark;
	Mat rot_cam2World,trans_cam2World;
#ifdef DEBUG
	cv::namedWindow("Calib_view", 1);
#endif

	while(allowed && (key != 'x') && (key != 27)&& ros::ok() ){

		image_getter.getCurrentImage(&current_image);
		MDetector.detect(current_image, TheMarkers, TheCameraParameters, TheMarkerSize);
		id_mark=-1;
		for(int i=0;i<TheMarkers.size();i++){
			if(TheMarkers[i].id==CalibMarkerID)
				id_mark=i;
		}
		if(id_mark>=0){

			TheMarkers[id_mark].draw(current_image,Scalar(0,255,0),3,true);

			//calibration

			Rodrigues(TheMarkers[id_mark].Rvec,rot_cam2World);
			trans_cam2World=TheMarkers[id_mark].Tvec+rot_cam2World*(transM2W+transCenter2RightDown);

			rot_cam2World*=rotM2W;
			EasyPolyLine(&current_image,
					Points3DtoCamPoints(Axes3D(0.25),rot_cam2World,trans_cam2World,TheCameraParameters) );
			EasyPolyLine(&current_image,
								Points3DtoCamPoints(Cadre3D(0.25),rot_cam2World,trans_cam2World,TheCameraParameters),
								true,Scalar(0,0,255));

		}
#ifdef DEBUG
		imshow("Calib_view",current_image);
#endif
		key=waitKey(1);
	}


	cout<<"Fin de la calibration"<<endl;
	cout<<"rotationC2W:"<<endl<<rot_cam2World<<endl<<endl;
	cout<<"translationC2W:"<<endl<<trans_cam2World<<endl<<endl;
}
