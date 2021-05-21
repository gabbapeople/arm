
#ifndef ARM_DRIVER_HPP_
#define ARM_DRIVER_HPP_

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <serial/serial.h>

#define NUM_JOINTS 6

#define HEAD_BYTE 0x20
#define CHECKSUM_BYTE 0x40
#define PACKET_SIZE 10

union TargetPosition {
    float f;
    uint8_t b[4]; 
};

class ArmDriver {
public:
    ArmDriver(double update_rate);

private:
    ros::NodeHandle node;

    ros::Subscriber arm_target_pos_sub[NUM_JOINTS];
    std::string sub_topic_name[NUM_JOINTS] = {"arm/j0_target_pos", "arm/j1_target_pos", "arm/j2_target_pos", "arm/j3_target_pos", "arm/j4_target_pos", "arm/j5_target_pos"};

    TargetPosition arm_target_pos[NUM_JOINTS] = { 0 };

    ros::Timer driver_timer;

    serial::Serial serial_0;
    std::string port_name_0;
    int baudrate_0;

    serial::Serial serial_1;
    std::string port_name_1;
    int baudrate_1;

    serial::Serial serial_2;
    std::string port_name_2;
    int baudrate_2;

    void armDriverCallback(const ros::TimerEvent& event);

    void printTargetPositions();
    void armTargetPosCallback(const std_msgs::Float32::ConstPtr& msg, int& number);

    void sendTargetPositions(serial::Serial* serial, TargetPosition pos_0, TargetPosition pos_1);
};

ArmDriver::ArmDriver(double update_rate) {

    ros::param::get("/arm_uart_port_0", port_name_0);
    ros::param::get("/arm_uart_baudrate_0", baudrate_0);
    ros::param::get("/arm_uart_port_1", port_name_1);
    ros::param::get("/arm_uart_baudrate_1", baudrate_1);
    ros::param::get("/arm_uart_port_2", port_name_2);
    ros::param::get("/arm_uart_baudrate_2", baudrate_2);

    serial_0.setPort(port_name_0);
    serial_0.setBaudrate(baudrate_0);

    serial_1.setPort(port_name_1);
    serial_1.setBaudrate(baudrate_1);

    serial_2.setPort(port_name_2);
    serial_2.setBaudrate(baudrate_2);

    serial::Timeout to = serial::Timeout::simpleTimeout(10);

    serial_0.setTimeout(to);
    serial_0.open();
    while (true) {
        if (serial_0.isOpen()) {
            ROS_WARN("Node Msg : Arm driver : Serial Port 0 initialized");
            break;
        } else
            ROS_WARN("Node Error : Arm driver : Serial Port 0 is not open");
    }

    serial_1.setTimeout(to);
    serial_1.open();
    while (true) {
        if (serial_1.isOpen()) {
            ROS_WARN("Node Msg : Arm driver : Serial Port 1 initialized");
            break;
        } else
            ROS_WARN("Node Error : Arm driver : Serial Port 1 is not open");
    }

    serial_2.setTimeout(to);
    serial_2.open();
    while (true) {
        if (serial_2.isOpen()) {
            ROS_WARN("Node Msg : Arm driver : Serial Port 2 initialized");
            break;
        } else
            ROS_WARN("Node Error : Arm driver : Serial Port 2 is not open");
    }

    for (int i = 0; i < NUM_JOINTS; i++)
        arm_target_pos_sub[i] = node.subscribe<std_msgs::Float32>(sub_topic_name[i], 1, boost::bind(&ArmDriver::armTargetPosCallback, this, _1, i));

    driver_timer = node.createTimer(ros::Duration(update_rate), &ArmDriver::armDriverCallback, this);

    ROS_WARN("Node Msg : Arm driver : Is ready.");
}

void ArmDriver::armDriverCallback(const ros::TimerEvent& event) {
    //printTargetPositions();
    sendTargetPositions(&serial_0, arm_target_pos[0], arm_target_pos[1]);
    sendTargetPositions(&serial_1, arm_target_pos[2], arm_target_pos[3]);
    sendTargetPositions(&serial_2, arm_target_pos[4], arm_target_pos[5]);
}

void ArmDriver::armTargetPosCallback(const std_msgs::Float32::ConstPtr& msg, int& number) {
    arm_target_pos[number].f = msg->data;
}

void ArmDriver::sendTargetPositions(serial::Serial* serial, TargetPosition pos_0, TargetPosition pos_1) {
    uint8_t packet[PACKET_SIZE];
    uint16_t sum = 0;

    packet[0] = HEAD_BYTE;
    memmove(packet + 1, pos_0.b, sizeof(TargetPosition));
	memmove(packet + 5, pos_1.b, sizeof(TargetPosition));

    for (uint8_t i = 0; i < (PACKET_SIZE - 1); i++)
        sum += packet[i];

    packet[PACKET_SIZE - 1] = sum & CHECKSUM_BYTE;
    serial->write(packet, PACKET_SIZE);
}

void ArmDriver::printTargetPositions() {
    ROS_INFO("J0TP: %-6f J1TP: %-6f J2TP: %-6f J3TP: %-6f J4TP: %-6f J5TP: %-6f",
        arm_target_pos[0].f,
        arm_target_pos[1].f,
        arm_target_pos[2].f,
        arm_target_pos[3].f,
        arm_target_pos[4].f,
        arm_target_pos[5].f);
}

#endif // ARM_DRIVER_HPP_