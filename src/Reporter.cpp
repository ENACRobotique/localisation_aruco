/*
 * Reporter.cpp
 *
 *  Created on: 9 déc. 2017
 *      Author: liehnfl
 */

#include "Reporter.h"

bool allowed=true;
void sig_stop(int a)
{
	allowed=false;
}

int main(int argc,char **argv){

	if (argc!=2) {
		cerr << ">>> Invalid number of arguments" << endl;
		cerr << ">>> Usage: [config.yml] " <<endl;
		return false;
	}


	ros::init(argc, argv, "Reporter");

	//READ INPUT
	Reporter report(argv[1]);

	signal(SIGINT, sig_stop);

	while(allowed){

		report.processTargeting();

		usleep(25000);
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
	res.x=-res.x;
	res.y=-res.y;
	res.z=-res.z;
	rotateWithQuat(res.x,res.y,res.z,res.quat);
	res.quat=res.quat.inverse();
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

bool eigenvector_compute(Eigen::Matrix4d M,tf::Quaternion& mean_quat){

	// Construct matrix operation object using the wrapper class DenseSymMatProd
	Spectra::DenseSymMatProd<double> op(M);
	// Construct eigen solver object, requesting the largest three eigenvalues
	Spectra::SymEigsSolver< double, Spectra::LARGEST_ALGE, Spectra::DenseSymMatProd<double> > eigs(&op, 3, 4);

	// Initialize and compute
	eigs.init();
	int nconv = eigs.compute();

	//Extract result
	if(eigs.info() != Spectra::SUCCESSFUL)
		return false;

	//take the greater Eigenvalues
	int index=0;
	Eigen::VectorXd evalues = eigs.eigenvalues();
	for(int i=0; i<nconv;i++){
		if(evalues[index]<evalues[i])
			index=i;
	}
	mean_quat=tf::Quaternion(eigs.eigenvectors().col(index)[1], //x
							 eigs.eigenvectors().col(index)[2], //y
							 eigs.eigenvectors().col(index)[3], //z
							 eigs.eigenvectors().col(index)[0]);//w
	return true;

}

bool fusionDataFunction(vector<Pose>poses,Pose &fusionedPose){
	//check if there data for fusion
	if(poses.size()==0)return false;
	//initialize
	fusionedPose.id_transfo=poses[0].id_transfo;
	fusionedPose.x=fusionedPose.y=fusionedPose.z=0;
	//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf page 4
	Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

	for(int i=0;i<(int)poses.size();i++){
		//mean position
		fusionedPose.x+=poses[i].x;
		fusionedPose.y+=poses[i].y;
		fusionedPose.z+=poses[i].z;
		//quaternion
		Eigen::Vector4d q;
		q <<poses[i].quat.w(),
			poses[i].quat.x(),
			poses[i].quat.y(),
			poses[i].quat.z();

		M.noalias()+=q*q.transpose();
	}
	//mean position
	fusionedPose.x/=poses.size();
	fusionedPose.y/=poses.size();
	fusionedPose.z/=poses.size();
	//quaternion
	M/=poses.size();
	//Mat K = 4*M - mat:eye;// improve efficacity

	if( !eigenvector_compute(M,fusionedPose.quat) )
		fusionedPose.quat=poses[0].quat;
	cout<<"----------------FUSIONNED POSE-------------------"<<endl;
	plot_pose(fusionedPose);
	return true;
}

//--------------------------PoseTempo---------------------------------

PoseTempo::PoseTempo(string topic)
{
	r_mutex=new std::recursive_mutex ();
  //subscribe to input markers
	if(topic==""){
		return;
	}
	pose_sub = nh_.subscribe(topic, 20, &PoseTempo::poseCb, this);
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
	for(int i=0;i<(int)markers_ids.size();i++)
		idS.push_back(markers_ids[i].id_transfo/MARKER_FRAME_MULTIPLIOR);

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

		if(std::find(idS.begin(), idS.end(), id/MARKER_FRAME_MULTIPLIOR)
					!=idS.end()){
			res.push_back( pose );
			waiting_poses.erase( waiting_poses. begin() + i );
		}
	}
	(*r_mutex).unlock();
	return res;
}

//--------------------------Target---------------------------------

Target::Target(vector<Pose>cameras,vector<Pose>markers,int id_t){
	World2Cam=cameras;
	Markers2Target=markers;
	id=id_t;
}

void Target::
updateProcess(vector<geometry_msgs::PoseStamped> new_poses){
	importPoses(new_poses);
	cleanOldPoses();

	vector<Pose>Pose_world;
	for(int i=0;i<(int)slidingPoses.size();i++)
		Pose_world.push_back(slidingPoses[i].World2Obj);

	if(id!=CALIB_TARGET)
		fusionDataFunction(Pose_world,fusionedPose);
}

void Target::
cleanOldPoses(){
	for (vector<ProjectivPoses>::iterator old_pose = slidingPoses.end();
			old_pose != slidingPoses.begin(); ) {
	     --old_pose;
		ros::Duration delta=ros::Time::now()-(*old_pose).timestamped;
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
	//cout<<"------------NB transfo : "<<new_poses.size()<<"-------------------"<<endl;
	for(int i=0;i<(int)new_poses.size();i++){
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

		//begin reproject

		reproject(proj_pose);
		cout<<"------------ID"<<proj_pose.Cam2mark.id_transfo<<"-------------------"<<endl;
		plot_pose(proj_pose.World2Obj);
		//end reproject
		res.push_back(proj_pose);
	}
	return importPoses(res);
}

void Target::
importPoses(vector<ProjectivPoses> new_poses){
	slidingPoses.insert(slidingPoses.begin(),new_poses.begin(),new_poses.end());
}
void Target::reproject(ProjectivPoses& p){
	//id of the marker and cam
	int id_cam=p.Cam2mark.id_transfo/CAM_FRAME_MULTIPLIOR;
	int idMark=(p.Cam2mark.id_transfo%CAM_FRAME_MULTIPLIOR)/MARKER_FRAME_MULTIPLIOR;
	//id u=in the vector Markers2Target & World2Cam
	id_cam= find_id_pose(     World2Cam,id_cam,   CAM_FRAME_MULTIPLIOR,INT_MAX);
	idMark= find_id_pose(Markers2Target,idMark,MARKER_FRAME_MULTIPLIOR,CAM_FRAME_MULTIPLIOR);
	if(id_cam<0 || idMark<0)
		return ;
	//reprojection
	p.Cam2Obj=multiPose(p.Cam2mark,Markers2Target[idMark]);
	p.Cam2Obj.id_transfo*=-1;
	p.World2Obj=multiPose(World2Cam[id_cam],p.Cam2Obj);
	p.World2Obj.id_transfo*=-1;
	p.Cam2Obj.id_transfo+=p.World2Obj.id_transfo*2;

}

int Target::find_id_pose(vector<Pose> in,int id,int offset,int max){
	for(int i=0;i<(int)in.size();i++){
		if((in[i].id_transfo%max)/offset==id){
			return i;
		}
	}
	return -1;
}
//--------------------------Reporter---------------------------------

Reporter::Reporter(string yaml)
	:tempo(YAML::LoadFile(yaml)["topic_in"].as<string>())
{
	readYAML(yaml);
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
	YAML::Node config = YAML::LoadFile(yaml);

	//read all targets
	vector<Pose>cameras;
	for(int i=0;i<(int)config["cameras"].size();i++){
		cameras.push_back(readCamTransfo(config["cameras"][i]));
	}
	//read/create all targets
	for(int i=0;i<(int)config["targets"].size();i++){
		YAML::Node target=config["targets"][i];
		vector<Pose>markers;
		for(int j=0;j<(int)target["markers"].size();j++){
			Pose p=readMarkerTransfo(target["markers"][j]);
			p.id_transfo+=target["id_target"].as<int>()*TARGET_FRAME_MULTIPLIOR;
			markers.push_back(p);
		}
		Targets.push_back(Target(cameras,markers,target["id_target"].as<int>()));
	}
	//create the publisher
	ros::NodeHandle n;
	publisher_targets=n.advertise<cube_pos::Robots>(config["topic_out"].as<string>(), 1);

}

cube_pos::Robot Reporter::
createMsg(Target t){
	cube_pos::Robot msg;
	msg.robot_id=t.fusionedPose.id_transfo;
	msg.pose.header.frame_id=t.fusionedPose.id_transfo;
	msg.pose.header.seq=0;
	msg.pose.header.stamp=ros::Time::now();
	msg.pose.pose.pose.position.x=t.fusionedPose.x;
	msg.pose.pose.pose.position.y=t.fusionedPose.y;
	msg.pose.pose.pose.position.z=t.fusionedPose.z;
 	tf::quaternionTFToMsg (t.fusionedPose.quat,
 						   msg.pose.pose.pose.orientation);
	//TODO covar

	msg.twist.header=msg.pose.header;
	//TODO twist
	return msg;
}

void Reporter::publish(){
	if(Targets.size()==0)
		return;

	cube_pos::Robots all_msg;
	for(int i=0;i<(int)Targets.size();i++){
		if( Targets[i].id!=CALIB_TARGET && Targets[i].slidingPoses.size()>0)
			all_msg.robots.push_back( createMsg(Targets[i]) );
	}
	if(all_msg.robots.size()==0)
		return;
	all_msg.header=all_msg.robots[0].pose.header;
	publisher_targets.publish(all_msg);

}
void  Reporter::processTargeting(){

	ros::spinOnce();
	for(int i=0;i<(int)Targets.size();i++){
		vector<geometry_msgs::PoseStamped> pose_temp=tempo.getPose(Targets[i].Markers2Target);

		Targets[i].updateProcess(pose_temp);
		cout<<"Target "<<Targets[i].id<<" nb:"<<Targets[i].slidingPoses.size()<<endl;

	}
	publish();
}
