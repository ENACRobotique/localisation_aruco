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

#include <sys/resource.h>

#include <yaml-cpp/yaml.h>

#define WIN_NAME "ROS ARUCO"
#define FPS_TEST
//#define THREADHOLD_VISU "THRESHOLD IMAGE"

bool readArguments (string *File, int argc,char **argv )
{
    if (argc!=2) {
        cerr << ">>> Invalid number of arguments" << endl;
        cerr << ">>> Usage: [config.yml] " <<endl;
        return false;
    }
    (*File)=argv[1];
    return true;

}

bool allowed=true;
void sig_stop(int a)
{
	allowed=false;
}

int main(int argc,char **argv) {
	//rend le process plus rapide, ou plutot moins interrompu
	setpriority(PRIO_PROCESS, getpid(), 10);

	cv::Mat current_image;

	//params Marker et aruco_cube
	CameraParameters TheCameraParameters;
	MarkerDetector MDetector;
	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
	vector<Marker> TheMarkers;

    // ROS messaging init
	ros::init(argc, argv, "aruco_cube_publisher");
	ros::NodeHandle n;
    ros::spinOnce();
#ifdef THREADHOLD_VISU
    int Thresmin,Thresmax;
#endif
    //var de gestion des entrées
	string topic;
	string TheIntrinsicFile;
	float TheMarkerSize=-1;
	float cube_size=-1;
	if (readArguments(&TheIntrinsicFile,argc,argv)==false) {
		return 0;
	}

	// Read camera parameters if passed and all other parameter
	if (TheIntrinsicFile != "") {
		TheCameraParameters.readFromXMLFile(TheIntrinsicFile);

		YAML::Node config = YAML::LoadFile(TheIntrinsicFile);
		if (config["cube_size"] &&config["marker_size"]&&config["topic"]){
			cube_size=config["cube_size"].as<float>();
			TheMarkerSize=config["marker_size"].as<float>();
			topic=config["topic"].as<string>();
		}
		else throw invalid_argument( "the data YAML need more arguments!" );;
	}

	string *topic_pointeur=NULL;
	if (topic != ""){
		topic="/"+topic;
		topic_pointeur=&topic;
	}
	ImageConverter ic  = ImageConverter(topic_pointeur);
    ros::Publisher pose_pub_markers = n.advertise<geometry_msgs::PoseStamped>("/aruco/markerarray", 1);
    //le gestionaire des cubes
    cube_manager test_cube;
    test_cube.push_back( aruco_cube(15,cube_size) );
    test_cube.push_back( aruco_cube(16,cube_size) );

	signal(SIGINT, sig_stop);
    //wait a image
    while (current_image.empty()) {
        ros::spinOnce();
        ic.getCurrentImage(&current_image);
        usleep(1000);
    }

	if (TheIntrinsicFile != "")
		TheCameraParameters.resize(current_image.size());
#ifdef DEBUG
	// Create gui
#ifdef THREADHOLD_VISU
	cv::namedWindow(THREADHOLD_VISU, 1);
	double inter1,inter2;
	MDetector.getThresholdParams(inter1, inter2);
	Thresmin=inter1;Thresmax=inter2;


	cv::createTrackbar("Thresmin", WIN_NAME, &Thresmin, 100);
	cv::createTrackbar("Thresmax", WIN_NAME, &Thresmax, 100);
#endif
	cv::namedWindow(WIN_NAME, 1);

#endif
#ifdef FPS_TEST
	ros::Duration mesure_fps[4];
	ros::Time mesure_temps;
	cout<<"Flag: Lect:\t\tMarkerProc\tCubeProc"<<endl;
#endif
	char key=0;
//------------------LOOP----------------------------------------------------
	// Capture until press ESC or until the end of the video
	while ((key != 'x') && (key != 27) && ros::ok()&& allowed) {
#ifdef FPS_TEST
		mesure_temps=ros::Time::now();
#endif
   		key = waitKey(1);
        ros::spinOnce();
#ifdef THREADHOLD_VISU
        MDetector.setThresholdParams(max(Thresmin,3), max(Thresmax,3));
#endif

		ic.getCurrentImage(&current_image);

        if (current_image.empty()) {
            usleep(2000);
            cout << ">>> Image EMPTY" << endl;
            continue;
        }
#ifdef FPS_TEST
		mesure_fps[0]=ros::Time::now()-mesure_temps;
		mesure_temps=ros::Time::now();
#endif
        // Detection of markers in the image passed
        MDetector.detect(current_image, TheMarkers, TheCameraParameters, TheMarkerSize);
#ifdef FPS_TEST
		mesure_fps[1]=ros::Time::now()-mesure_temps;
		mesure_temps=ros::Time::now();
#endif
        test_cube.update_marker(TheMarkers);
        test_cube.compute_all();
        test_cube.publish_marcker_pose(pose_pub_markers,ic.timestamp);
        test_cube.aff_cube(&current_image,TheCameraParameters);
#ifdef FPS_TEST
		mesure_fps[2]=ros::Time::now()-mesure_temps;
		mesure_temps=ros::Time::now();
#endif
        // Show input with augmented information and the thresholded image
#ifdef DEBUG
        imshow(WIN_NAME, current_image);
#ifdef THREADHOLD_VISU
        Mat threadhold_im=MDetector.getThresholdedImage();
        imshow(THREADHOLD_VISU,threadhold_im);

#endif
#endif

#ifdef FPS_TEST
		cout<<"\r";//pour ne pas pourrir le terminal
		cout<<"Temps:";
		for(int i=0;i<3;i++)
			cout<<mesure_fps[i]*1000<<"\t";
		cout<<"ms";
#endif
	}
}

