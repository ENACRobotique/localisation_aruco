//
// Created by guilhem on 25/10/17.
//
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include "Robots.h"
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

#define NODE_NAME "cube_pose_to_uart"
#define SUB_ROBOTS_TOPIC "/robots"
#define RADTOUINT16FACTOR 20860.756700940907
#define MSG_SIZE 13

int uart0_filestream = -1;

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

void enable_uart(){

    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
    uart0_filestream = -1;

    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    if (uart0_filestream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);
}

void uart_write(char * data, int length){
    if (uart0_filestream != -1)
    {
        ssize_t count = write(uart0_filestream, &data[0], static_cast<size_t>(length));		//Filestream, bytes to write, number of bytes to write
        if (count < 0)
        {
            printf("UART TX error\n");
        }
    }
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
        uart_write(rawMsg.data, MSG_SIZE);

    }
}


int main(int argc, char** argv) {
    enable_uart();
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    ros::Subscriber s = n.subscribe<cube_pos::Robots>(SUB_ROBOTS_TOPIC, 1000, newRobotPoseMessageCB);
    ros::spin();
    return 0;
}