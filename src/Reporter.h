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

#define POSE_MAX_NUMBER 100
class PoseTempo{
public:
	vector<geometry_msgs::PoseStamped> waiting_poses;

	ros::NodeHandle nh_;
	ros::Subscriber pose_sub;

	std::recursive_mutex * r_mutex;

	void poseCb(const geometry_msgs::PoseStamped& msg);

//public:
	//constructor
	PoseTempo(string *topic=NULL);
	~PoseTempo();

	vector<geometry_msgs::PoseStamped> getPose(vector<Pose>markers_ids);
	vector<geometry_msgs::PoseStamped> getPose(vector<int>ids);

};

#define OLDEST_TARGET .25
class Target{
public:
	//sliding_windows of projectiv poses
	vector<ProjectivPoses> slidingPoses;
	//complementary transformation
	vector<Pose>Markers2Target;
	vector<Pose>World2Cam;
	//constructor
	Target(){};

	//for world and target transformation
	void getTransf();
	void getTransf();

	void updateProcessPublish(vector<geometry_msgs::PoseStamped> new_poses);
	void publish(ros::Publisher publisher);
	void aff_Cam_Projection(bool fusion=false);
private:
	void importPoses(vector<geometry_msgs::PoseStamped> new_poses);
	void importPoses(vector<ProjectivPoses> new_poses);
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
