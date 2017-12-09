/*
 * Reporter.h
 *
 *  Created on: 9 déc. 2017
 *      Author: liehnfl
 */

#ifndef SRC_REPORTER_H_
#define SRC_REPORTER_H_

#include "Utils.h"

typedef struct{
	int id_transfo;
	//Translation
	double x;
	double y;
	double z;
	//Quaternion
	double qw;
	double qx;
	double qy;
	double qz;
}Pose;

typedef struct{
	ros::Time timestamped;
	//Differents PoseS
	Pose Cam2mark;
	Pose Cam2Obj;
	Pose World2Obj;
}ProjectivPoses;

class PoseTempo{
public:
	list<geometry_msgs::PoseStamped> waiting_poses;

	ros::NodeHandle nh_;
	ros::Subscriber pose_sub;

	std::recursive_mutex * r_mutex;

	void poseCb(const geometry_msgs::PoseStamped& msg);

//public:
	//constructor
	PoseTempo(string *topic=NULL);
	~PoseTempo();

	list<geometry_msgs::PoseStamped> getPose(list<int>markers_ids);

};

class Target{
public:
	//sliding_windows of projectiv poses
	list<ProjectivPoses> sliding_mask;

	//contenaire des id targets à définir!

	//constructor
	Target();

	//
	void getIds();
	void setIds();

	void UpdateMarkers();
	void publish(ros::Publisher publisher);
	void aff_Cam_Projection(bool fusion=false);
private:
	ProjectivPoses reprojectNewMarker(Pose p);
	void cleanOldPoses();
	ProjectivPoses fusionDataFunction();

};

class Reporter{
public:
	list<Target> Targets;

	//input and output of the system
	PoseTempo tempo;
	ros::Publisher publisher_targets;

	//constructor
	Reporter();

	void addTargets();
	void deleteTargets();
	void processTargetingOnce();

};




#endif /* SRC_REPORTER_H_ */
