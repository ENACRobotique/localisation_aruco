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

#define WIN_NAME "ROS ARUCO"
//#define THREADHOLD_VISU "THRESHOLD IMAGE"


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

	//params Marker et aruco_cube
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
    ros::Publisher pose_pub_markers = n.advertise<geometry_msgs::PoseStamped>("/aruco/markerarray", 1);


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

	signal(SIGINT, sig_stop);
	char key=0;
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

        test_cube.publish_marcker_pose(pose_pub_markers,ic.timestamp);

        // Show input with augmented information and the thresholded image
#ifdef DEBUG
        cv::imshow(WIN_NAME, current_image);
        //cv::imshow("THRESHOLD IMAGE", MDetector.getThresholdedImage());
#endif
        // Limit to 60hz
  		usleep(15000);

	}
}

