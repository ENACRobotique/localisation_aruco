/*
 * Reporter.cpp
 *
 *  Created on: 9 déc. 2017
 *      Author: liehnfl
 */

#include "Reporter.h"



int main(int argc,char **argv){

	ros::init(argc, argv, "Reporter");

	//basic input
	string in_topic="markers";


	//target
	Target cube16;
	Pose temp={
		.id_transfo=16,
		.x=0,.y=0,.z=0,
		.qw=0,.qx=0,.qy=0,.qz=0};
	cube16.Markers2Target.push_back(temp);
	temp.id_transfo=26;
	cube16.Markers2Target.push_back(temp);
	temp.id_transfo=36;
	cube16.Markers2Target.push_back(temp);
	PoseTempo tempotest(&in_topic);

	for(int i=0;i<50;i++){
		ros::spinOnce();

		cout<<"taille tempo av: "<<tempotest.waiting_poses.size()<<endl;
		vector<geometry_msgs::PoseStamped> pose_temp=tempotest.getPose(cube16.Markers2Target);
		cube16.updateProcessPublish(pose_temp);
		cout<<"taille pose temp: "<<pose_temp.size()<<endl;
		cout<<"taille pose : "<<cube16.slidingPoses.size()<<endl;
		cout<<"taille tempo ap: "<<tempotest.waiting_poses.size()<<endl;
		cout<<"------------------------------------------------------------"<<endl;
		usleep(100000);
	}


}

//--------------------------PoseTempo---------------------------------

PoseTempo::PoseTempo(string *topic)
{
  //subscribe to input markers
	string default_top="/markers";
	if(topic==NULL){
		topic=&default_top;
	}
	pose_sub = nh_.subscribe(*topic, 1, &PoseTempo::poseCb, this);
	r_mutex=new std::recursive_mutex ();

}

PoseTempo::~PoseTempo()
{
	pose_sub.shutdown();
	printf("\n>> ROS Stopped PoseStamped Import \n");
}

void PoseTempo::
poseCb(const geometry_msgs::PoseStamped& msg){
	(*r_mutex).lock();

	waiting_poses.push_back(msg);
	while((int)waiting_poses.size()>POSE_MAX_NUMBER)
	{
		waiting_poses.pop_back();
	}
	(*r_mutex).unlock();
}

vector<geometry_msgs::PoseStamped>
PoseTempo::
getPose(vector<Pose>markers_ids){
	vector<int>idS;
	for(int i=0;i<markers_ids.size();i++)idS.push_back(markers_ids[i].id_transfo);

	return getPose(idS);
}

vector<geometry_msgs::PoseStamped>
PoseTempo::
getPose(vector<int>idS){

	vector<geometry_msgs::PoseStamped> res;
	(*r_mutex).lock();
	for(int i=waiting_poses.size()-1;i>=0;i--){
		geometry_msgs::PoseStamped pose=waiting_poses[i];
		int id=stoi(pose.header.frame_id)%CAM_FRAME_MULTIPLIOR;

		if(std::find(idS.begin(), idS.end(), id)
					!=idS.end()){
			res.push_back( pose );
			waiting_poses.erase( waiting_poses. begin() + i );
		}
	}
	(*r_mutex).unlock();
	return res;
}

//--------------------------Target---------------------------------


void Target::
updateProcessPublish(vector<geometry_msgs::PoseStamped> new_poses){
	importPoses(new_poses);
	cleanOldPoses();

	//fusion

	//publish
}


void Target::
cleanOldPoses(){
	for (vector<ProjectivPoses>::iterator old_pose = slidingPoses.end();
			old_pose != slidingPoses.begin(); ) {
	     --old_pose;
		ros::Duration delta=ros::Time::now()-(*old_pose).timestamped;
		cout<<(*old_pose).timestamped<<endl;
		if(delta >ros::Duration(OLDEST_TARGET))
			old_pose = slidingPoses.erase(old_pose);
	  else
		  return;
	}
}

void Target::
importPoses(vector<geometry_msgs::PoseStamped> new_poses){
	vector<ProjectivPoses>res;
	ProjectivPoses proj_pose={	};
	Pose interpose={};
	for(int i=0;i<new_poses.size();i++){
		interpose.id_transfo=stoi(new_poses[i].header.frame_id);
		interpose.x=new_poses[i].pose.position.x;
		interpose.y=new_poses[i].pose.position.y;
		interpose.z=new_poses[i].pose.position.z;
		interpose.qw=new_poses[i].pose.orientation.w;
		interpose.qx=new_poses[i].pose.orientation.x;
		interpose.qy=new_poses[i].pose.orientation.y;
		interpose.qz=new_poses[i].pose.orientation.z;

		proj_pose.Cam2mark=interpose;
		proj_pose.timestamped=new_poses[i].header.stamp;
		cout<<"stamp:"<<proj_pose.timestamped<<endl;
		//begin reproject

		//end reproject
		res.push_back(proj_pose);
	}
	return importPoses(res);
}

void Target::
importPoses(vector<ProjectivPoses> new_poses){
	slidingPoses.insert(slidingPoses.begin(),new_poses.begin(),new_poses.end());
}
