//
// Created by guilhem on 25/10/17.
//
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include "Robots.h"
#include "arduPi.h"
#define NODE_NAME "cube_pose_to_uart"
#define SUB_ROBOTS_TOPIC "/robots"
#define RADTOUINT16FACTOR 20860.756700940907
#define MSG_SIZE 13

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

void newRobotPoseMessageCB(const cube_pos::Robots::ConstPtr& msg){
    for (auto robot : msg->robots) {
        uRawMsg rawMsg;
        sRobotMsg serialMsg{};
        sRobotData pose{}, speed{};
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
        Serial.write(rawMsg.data, MSG_SIZE);

    }
}


int main(int argc, char** argv) {
    Serial.begin(9600);
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    ros::Subscriber s = n.subscribe<cube_pos::Robots>(SUB_ROBOTS_TOPIC, 1000, newRobotPoseMessageCB);
    ros::spin();
    return 0;
}