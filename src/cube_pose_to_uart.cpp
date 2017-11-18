//
// Created by guilhem on 25/10/17.
//
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include "Robots.h"
#include "geometry_msgs/PoseStamped.h"
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <xbeep.h>
#define USE_CUSTOM_MSG 0
#define NODE_NAME "cube_pose_to_uart"
#define SUB_ROBOTS_TOPIC "/robots"
#define RADTOUINT16FACTOR 10430.219195527361
#define MSG_SIZE 13

int uart0_filestream = -1;
struct xbee_con *con;

typedef enum{
    MAIN_ALLY,
    SECONDARY_ALLY,
    MAIN_OPPONENT,
    SECONDARY_OPPONENT
}eRobotType;

typedef struct __attribute__((packed)){
    uint16_t x;
    uint16_t y;
    uint16_t theta;
}sRobotData;


typedef struct __attribute__((packed)){
    eRobotType robotId :8; //1bytes
    sRobotData robotPose; //6bytes
    sRobotData robotSpeed; //6bytes
}sRobotMsg;

typedef union {
    sRobotMsg msg;
    char data[MSG_SIZE];
} uRawMsg;

void initXbee(xbee_con* con){
    struct xbee *xbee;
    struct xbee_conAddress address;
    struct xbee_conSettings settings;
    xbee_err ret;

    if ((ret = xbee_setup(&xbee, "xbee1", "/dev/ttyAMA0", 115200)) != XBEE_ENONE) {
        printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
    }

    memset(&address, 0, sizeof(address));
    address.addr64_enabled = 1;
    address.addr64[0] = 0x00;
    address.addr64[1] = 0x00;
    address.addr64[2] = 0x00;
    address.addr64[3] = 0x00;
    address.addr64[4] = 0x00;
    address.addr64[5] = 0x00;
    address.addr64[6] = 0xFF;
    address.addr64[7] = 0xFF;
    if ((ret = xbee_conNew(xbee, &con, "64-bit Data", &address)) != XBEE_ENONE) {
        xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
    }

    /* getting an ACK for a broadcast message is kinda pointless... */
    xbee_conSettings(con, NULL, &settings);
    settings.disableAck = 1;
    xbee_conSettings(con, &settings, NULL);

}

void xbeeWrite(xbee_con* con, char * data, int length){
    xbee_conTx(con, NULL, data, length);
}

void newRobotPoseMessageCB(const cube_pos::Robots::ConstPtr& msg){
    for (auto robot : msg->robots) {
        uRawMsg rawMsg;
        sRobotMsg serialMsg{};
        tf::Pose poseForQuaternionConversion;
        tf::poseMsgToTF(robot.pose.pose.pose, poseForQuaternionConversion);
        double yaw_angle = tf::getYaw(poseForQuaternionConversion.getRotation());

        serialMsg.robotId = static_cast<eRobotType>(robot.robot_id);
        serialMsg.robotPose.x = static_cast<uint16_t>(robot.pose.pose.pose.position.x);
        serialMsg.robotPose.y = static_cast<uint16_t>(robot.pose.pose.pose.position.y); //TODO : Multiply to use fully the 2 bytes
        serialMsg.robotPose.theta = static_cast<uint16_t>(yaw_angle * RADTOUINT16FACTOR);
        serialMsg.robotSpeed.x = static_cast<uint16_t>(robot.twist.twist.twist.linear.x); // TODO : Expand to fully use 2 bytes
        serialMsg.robotSpeed.y = static_cast<uint16_t>(robot.twist.twist.twist.linear.y);
        serialMsg.robotSpeed.theta = static_cast<uint16_t>(robot.twist.twist.twist.angular.z);

        rawMsg.msg = serialMsg;
        xbeeWrite(con, rawMsg.data, MSG_SIZE);

    }
}

void newRobotPoseMessageCBGeoMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uRawMsg rawMsg;
    sRobotMsg serialMsg{};
    tf::Pose poseForQuaternionConversion;
    tf::poseMsgToTF(msg->pose, poseForQuaternionConversion);
    double yaw_angle = tf::getYaw(poseForQuaternionConversion.getRotation());

    //serialMsg.robotId = static_cast<eRobotType>(std::stoi(msg->header.frame_id));
    serialMsg.robotPose.x = static_cast<uint16_t>(msg->pose.position.x);
    serialMsg.robotPose.y = static_cast<uint16_t>(msg->pose.position.y); //TODO : Multiply to use fully the 2 bytes
    serialMsg.robotPose.theta = static_cast<uint16_t>(yaw_angle * RADTOUINT16FACTOR);

    rawMsg.msg = serialMsg;
    xbeeWrite(con, rawMsg.data, MSG_SIZE);
}


int main(int argc, char** argv) {
    initXbee(con);
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
#if USE_CUSTOM_MSG
    ros::Subscriber s = n.subscribe<cube_pos::Robots>(SUB_ROBOTS_TOPIC, 1000, newRobotPoseMessageCB);
#else
    ros::Subscriber s = n.subscribe<geometry_msgs::PoseStamped>(SUB_ROBOTS_TOPIC, 1000, newRobotPoseMessageCBGeoMsg);
#endif
    ROS_INFO_NAMED(NODE_NAME, "initialized.");
    ros::spin();
    return 0;
}