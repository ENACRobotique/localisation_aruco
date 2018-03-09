
#include <stdio.h>
#include <stdbool.h>
#include <string>
#include <errno.h>

#include <signal.h>

//UART connection
#include <wiringPi.h>
#include <wiringSerial.h>

//Message SPE TODO transfère it to a common header
#define MESSAGE_LENGHT 70
#define  MASK_SERIAL 0x42
#define ECHAP_SERIAL 0x54

// TF (easy transform)
#include <tf/transform_broadcaster.h>

//ROS libraires
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class MarkerMessage{
public:
	//header
	char header;
	int id;
	//quaternion
	double qw;
	double qx;
	double qy;
	double qz;
	//position
	double x;
	double y;
	double z;
	//time
	double time;
	char checksum;

	bool check_pass=false;

	char calcul_checksum(char* message){
		char res=(char)0;
		for(int i=0;i<MESSAGE_LENGHT-2;++i)
			res^=message[i];
		printf("Calcul de la check Sum %hhX\n",res);
		return res;
	}

	MarkerMessage(char* message){
		//header
		header=message[0];
		id= (*(int*)(message))>>8;
		id&=~(1<<8*3);
		//quaternion
		qw = *(double*)(message+04);
		qx = *(double*)(message+12);
		qy = *(double*)(message+20);
		qz = *(double*)(message+28);
		//position
		x = *(double*)(message+36);
		y = *(double*)(message+44);
		z = *(double*)(message+52);
		//Time
		time = *(double*)(message+60);
		//checksum
		checksum=message[MESSAGE_LENGHT-2];
		check_pass= (checksum==calcul_checksum(message) );
	}
	void print_message(){

		printf("header: %c\n",header);	
		printf("id: %d\n",id);	
		printf("quat:\n");
		printf("w: %lf\n",qw);
		printf("x: %lf\n",qx);
		printf("y: %lf\n",qy);
		printf("z: %lf\n",qz);
		printf("pos:\n");
		printf("x: %lf\n",x);
		printf("y: %lf\n",y);
		printf("z: %lf\n",z);
		printf("\ntime: %lf\n",time);
		printf("Checksum :%hhX\n Valid:%s\n",(int)checksum, 
									check_pass ? "true" : "false" );
	}
};



void publish(MarkerMessage message,ros::Publisher publi){
	tf::Quaternion tf_quat(message.qx,message.qy,
								  message.qz,message.qw);
	geometry_msgs::Quaternion quat;
 	tf::quaternionTFToMsg (tf_quat,quat);

	//Pose
	geometry_msgs::Pose pose;
	pose.position.x = message.x;
	pose.position.y = message.y;
	pose.position.z = message.z;
	pose.orientation = quat;

	//PoseStamped
	geometry_msgs::PoseStamped msg_ps;
	msg_ps.header.frame_id =std::to_string(message.id);
	msg_ps.header.stamp = ros::Time::now();//TODO - message.time
	msg_ps.pose = pose;
	//publish PoseStamped message
	publi.publish(msg_ps);
}

char readcharNmask(int fd){
	char res=serialGetchar(fd);
	if( (int)res == ECHAP_SERIAL )//ECHAP
		res = serialGetchar(fd)^MASK_SERIAL;
	return res;
}
void rattrape_decalage(char* message){
	printf("Ratrappage du message!!\n");
	for(int i =0;i<MESSAGE_LENGHT-1;i++)
		*(message+i)=*(message+i+1);
	*(message+MESSAGE_LENGHT-1)='F';
}

bool RUN_PROCESS=true;
void terminal_stop(int s){
	(void*)s;
	printf("Finish\n");
	RUN_PROCESS=false; 
}

//TODO add ros node input (params: UART_in,BaudRate,ROS_node)
int main(int argc, char* argv[])
{

	int fd ;
	//ros node
	std::string topic ="marker";
	
	ros::init(argc, argv, "UARTspectator1");
	ros::NodeHandle n;
	ros::Publisher pose_pub_markers= n.advertise<geometry_msgs::PoseStamped>(topic, 1);


	//init
	if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
		return 1 ;
	}

	if (wiringPiSetup () == -1)
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
		return 1 ;
	}

	char message[MESSAGE_LENGHT];
	//work
	signal (SIGINT,terminal_stop);
	while(RUN_PROCESS){
		ros::spinOnce();
		//Header Byte
		while( RUN_PROCESS && (serialDataAvail(fd)<=1) && (readcharNmask(fd)!=(int)'M'))
		{
			delay(1);
		}
		if(!RUN_PROCESS) continue;
		
		//receive all Bytes of the message
		bool message_OK=true;
		message[0]='M';
		for(int i=1;i<MESSAGE_LENGHT;i++){
			message[i]=readcharNmask(fd);
			if( (int)message[i]==-1){
				message_OK=false;
				break;
			}
		}
		//mauvaise réception( intéruption ou autre )	
		if( (!message_OK) || (message[MESSAGE_LENGHT-1]!='F') ){
				
			//tentative de rattrapage d'un glissement
			if( (message[1]='M') && (readcharNmask(fd)=='F') ){
				rattrape_decalage(message);
			}
			else{
				printf("BAD MESSAGE:\n");
				for(int i=0;i<MESSAGE_LENGHT;i++)
					printf("%3d|",(int)message[i]);
				printf("\n\n");

				continue;
			}
		

		}
		//decode
		MarkerMessage MarkMess(message);

		//si mauvaise réception : checksum incohérente
		if(!MarkMess.check_pass){
			printf("BAD checksum:\n");
			continue;
		}
		/*
		for(int i=0;i<MESSAGE_LENGHT;i++)
			printf("%3d|",(int)message[i]);
		printf("\n\n");
		*/
		//MarkMess.print_message();
		
		//Publish
		publish(MarkMess,pose_pub_markers);
	}

	return 0 ;
}
