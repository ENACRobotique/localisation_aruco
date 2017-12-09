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

	PoseTempo tempotest(&in_topic);

	for(int i=0;i<100;i++){
		ros::spinOnce();

		int nb_pose=tempotest.waiting_poses.size();
		cout<<"taille tempo: "<<nb_pose<<endl;

		usleep(10000);
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
	waiting_poses.push_front(msg);
	(*r_mutex).unlock();
}


