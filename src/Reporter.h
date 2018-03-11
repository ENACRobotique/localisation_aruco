/*
 * Reporter.h
 *
 *  Created on: 9 déc. 2017
 *      Author: liehnfl
 */

#ifndef SRC_REPORTER_H_
#define SRC_REPORTER_H_

#include "Utils.h"
//ROS
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//Service & Message
#include "cube_pos/CalibRequest.h"
#include "Robot.h"
#include "Robots.h"
//libraries for quaternion fusion
#include <Eigen/Core>
#include <SymEigsSolver.h>

typedef struct{
	int id_transfo;
	//Translation
	double x;
	double y;
	double z;
	//Quaternion
	tf::Quaternion quat;
}Pose;

typedef struct{
	ros::Time timestamped;
	//Differents PoseS
	Pose Cam2mark;
	Pose Cam2Obj;
	Pose World2Obj;
}ProjectivPoses;

void plot_pose(Pose p);
void rotateWithQuat(double &x,double&y,double&z,tf::Quaternion q);
Pose invPose(Pose p);
Pose multiPose(Pose a,Pose b);
bool eigenvector_compute(Eigen::Matrix4d M,tf::Quaternion& mean_quat);

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
	PoseTempo(string topic="");
	~PoseTempo();

	vector<geometry_msgs::PoseStamped> getPose(vector<Pose>markers_ids);
	vector<geometry_msgs::PoseStamped> getPose(vector<int>ids);

};

#define OLDEST_TARGET .25
class Target{
public:
	//sliding_windows of projectiv poses
	vector<ProjectivPoses> slidingPoses;
	Pose fusionedPose;
	//complementary transformation
	vector<Pose>Markers2Target;
	vector<Pose>World2Cam;
	//constructor
	Target(){};
	Target(vector<Pose>cameras,vector<Pose>markers);

	void updateProcess(vector<geometry_msgs::PoseStamped> new_poses);
	void publish(ros::Publisher publisher);
	void aff_Cam_Projection(bool fusion=false);
//private:
	void importPoses(vector<geometry_msgs::PoseStamped> new_poses);
	void importPoses(vector<ProjectivPoses> new_poses);
	void reproject(ProjectivPoses& p);
	ProjectivPoses reprojectNewMarker(Pose p);
	void cleanOldPoses();
	void fusionDataFunction();

	void rotateWithQuat(double &x,double&y,double&z,tf::Quaternion q);
	int find_id_pose(vector<Pose> in,int id,int offset,int max);
};

class Reporter{
public:
	vector<Target> Targets;

	//input and output of the system
	PoseTempo tempo;
	ros::Publisher publisher_targets;

	//constructor
	Reporter(string yaml);

	void readYAML(string yaml);
	void readPose(YAML::Node pose_node,Pose &p);
	Pose readMarkerTransfo(YAML::Node pose_node);
	Pose readCamTransfo(YAML::Node pose_node);
	void addTargets();
	void deleteTargets();
	void processTargeting();

};




#endif /* SRC_REPORTER_H_ */
