/*
 * Reporter.cpp
 *
 *  Created on: 9 déc. 2017
 *      Author: liehnfl
 */

#include "Reporter.h"

int main(int argc,char **argv){

	if (argc!=2) {
		cerr << ">>> Invalid number of arguments" << endl;
		cerr << ">>> Usage: [config.yml] " <<endl;
		return false;
	}

	ros::init(argc, argv, "Reporter");

	//basic input
	string in_topic="markers";

	Reporter report;
	//READ INPUT
	report.readYAML(argv[1]);

	return 0;

	//target
	Target cube16;
	Pose temp={
		.id_transfo=16,
		.x=1,.y=0,.z=0,
		.quat=tf::Quaternion(0,0,-0.707,0.707),
		};
	cube16.Markers2Target.push_back(temp);
	temp.id_transfo=26;
	cube16.Markers2Target.push_back(temp);
	temp.id_transfo=36;
	cube16.Markers2Target.push_back(temp);

	Mat cam2table=(Mat_<float>(3, 3 ) << 0.7071068, -0.7071068, 0,
										        -0.055479, -0.055479, -0.9969173,
										         0.704927, 0.704927, -0.0784591 );
	temp.id_transfo=-CAM_FRAME_MULTIPLIOR*1;
	temp.x=0;
	temp.y=.33;
	temp.z=0;
	temp.quat=Mat2Quaternion(cam2table);
	cube16.World2Cam.push_back(temp);


	PoseTempo tempotest(&in_topic);

	for(int i=0;i<50;i++){
		ros::spinOnce();

		vector<geometry_msgs::PoseStamped> pose_temp=tempotest.getPose(cube16.Markers2Target);
		cube16.updateProcessPublish(pose_temp);
		usleep(100000);
	}


}

//--------------------------Tools---------------------------------

inline void plot_pose(Pose p){
	cout<<"pose id\t"<<p.id_transfo<<endl;
	cout<<"x\t"<<p.x<<endl;
	cout<<"y\t"<<p.y<<endl;
	cout<<"z\t"<<p.z<<endl;
	cout<<"qw\t"<<p.quat.w()<<endl;
	cout<<"qx\t"<<p.quat.x()<<endl;
	cout<<"qy\t"<<p.quat.y()<<endl;
	cout<<"qz\t"<<p.quat.z()<<endl;
}

void
rotateWithQuat(double &x,double&y,double&z,tf::Quaternion q){
	tf::Quaternion v(x,y,z,0);
	v=q.inverse()*v*q;
	x=v.x();
	y=v.y();
	z=v.z();

}

Pose invPose(Pose p){
	Pose res=p;
	res.id_transfo=-res.id_transfo;
	res.quat=res.quat.inverse();
	rotateWithQuat(res.x,res.y,res.z,res.quat);
	return res;
}


Pose multiPose(Pose a,Pose b){
	Pose res;
	res.x=a.x;
	res.y=a.y;
	res.z=a.z;
	res.quat=a.quat*b.quat;
	res.id_transfo=b.id_transfo-a.id_transfo;
	rotateWithQuat(b.x,b.y,b.z,a.quat.inverse());
	res.x+=b.x;
	res.y+=b.y;
	res.z+=b.z;
	return res;
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
		interpose.quat=tf::Quaternion(new_poses[i].pose.orientation.x,
									  new_poses[i].pose.orientation.y,
									  new_poses[i].pose.orientation.z,
									  new_poses[i].pose.orientation.w);

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


//--------------------------Reporter---------------------------------

namespace YAML {
template<>
struct convert<Pose> {
  static YAML::Node encode(const Pose& p) {
	YAML::Node node;
    node.push_back(p.id_transfo);
    node.push_back(p.x);
    node.push_back(p.y);
    node.push_back(p.z);
    node.push_back(p.quat);
    return node;
  }
  static bool decode(const YAML::Node& node, Pose& p) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }
    cout<<"coucou"<<endl;
    p.x = node[0].as<double>();
    p.y = node[1].as<double>();
    p.z = node[2].as<double>();
    return true;
  }
};
template<>
struct convert<tf::Quaternion> {
  static YAML::Node encode(const tf::Quaternion& q) {
	YAML::Node node;
    node.push_back(q.w());
    node.push_back(q.x());
    node.push_back(q.y());
    node.push_back(q.z());
    return node;
  }
  static bool decode(const YAML::Node& node, tf::Quaternion& q) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }
    cout<<"coucou"<<endl;
    q=tf::Quaternion(node[1].as<double>(),
    			 	 node[2].as<double>(),
					 node[3].as<double>(),
					 node[0].as<double>());
    return true;
  }
};
}



void Reporter::
readPose(YAML::Node pose_node,Pose &p){

	p.x=pose_node["translation"][0].as<double>();
	p.y=pose_node["translation"][1].as<double>();
	p.z=pose_node["translation"][2].as<double>();

	p.quat=tf::Quaternion(pose_node["rotation"][1].as<double>(),
						  pose_node["rotation"][2].as<double>(),
						  pose_node["rotation"][3].as<double>(),
						  pose_node["rotation"][0].as<double>());
	if(p.id_transfo<0)
		p=invPose(p);
}

Pose Reporter::
readCamTransfo(YAML::Node pose_node){
	Pose p;
	p.id_transfo=pose_node["id_cam"].as<int>()*CAM_FRAME_MULTIPLIOR;
	readPose(pose_node,p);
	return p;
}
Pose Reporter::
readMarkerTransfo(YAML::Node pose_node){
	Pose p;
	p.id_transfo=pose_node["id_marker"].as<int>()*MARKER_FRAME_MULTIPLIOR;
	readPose(pose_node,p);
	return p;
}

void Reporter::
readYAML(string yaml){
	cout<< "input yaml:"<<yaml<<endl;
	YAML::Node config = YAML::LoadFile(yaml);
	cout<<"top in "<<config["topic_in"]<<endl;
	cout<<"top out"<<config["topic_out"]<<endl;
	cout<<"lenght of targets"<<config["targets"].size()<<endl;

	//read all targets
	vector<Pose>cameras;
	for(int i=0;i<config["cameras"].size();i++){
		cameras.push_back(readCamTransfo(config["cameras"][i]));
	}
	for(int i=0;i<cameras.size();i++){
		plot_pose(cameras[i]);
	}
	//read all targets
	for(int i=0;i<config["targets"].size();i++){
		YAML::Node target=config["targets"][i];
		cout<<"target ID: "<<target["id_target"]<<endl;
		vector<Pose>poses;
		for(int j=0;j<target["markers"].size();j++){
			Pose p=readMarkerTransfo(target["markers"][j]);
			p.id_transfo+=target["id_target"].as<int>()*TARGET_FRAME_MULTIPLIOR;
			poses.push_back(p);
		}
		for(int j=0;j<poses.size();j++){
			plot_pose(poses[j]);
		}
	}

}
