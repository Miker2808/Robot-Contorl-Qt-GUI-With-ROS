#include "qnode.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <string>

QNode::QNode(int argc, char** argv) :
    init_argc(argc),
    init_argv(argv)
{

}

QNode::~QNode(){
    if(ros::isStarted()){
        ros::shutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc, init_argv, "RobotGUI"); // internal ROS method to start a node named "RobotGUI"

    if ( !ros::master::check()) { // checks if rosmaster is online, gui wont start without a ros running.
        //system("gnome-terminal -- roscore"); // used to run roscore from within the gui (not recommended)
        return false; // disable line if previous line is enabled
    }
    ros::start();
    ros::NodeHandle node;
    // subscribing objects
    safety_sub = node.subscribe("/robot/safety_master", 5, &QNode::safetyCallBack, this);
    heading_sub = node.subscribe("/robot/imu/heading", 5, &QNode::headingCallBack, this);
    motor_left_sub = node.subscribe("/robot/motor_left", 5, &QNode::motor_leftCallBack, this);
    motor_right_sub = node.subscribe("/robot/motor_right", 5, &QNode::motor_rightCallBack, this);
    gps_sub = node.subscribe("/robot/gps", 5, &QNode::gpsCallBack, this);
    robot_connection_sub = node.subscribe("/robot/connection", 2, &QNode::robot_connectionCallBack, this);
    battery_voltage_sub = node.subscribe("/robot/battery_voltage", 5, &QNode::battery_voltageCallBack, this);
    pitch_sub = node.subscribe("/robot/imu/pitch", 5, &QNode::pitchCallBack, this);
    roll_sub = node.subscribe("/robot/imu/roll", 5, &QNode::rollCallBack, this);

    // publishing objects
    motor_left_pub = node.advertise<std_msgs::Int64>("/robot/motor_left", 5);
    motor_right_pub = node.advertise<std_msgs::Int64>("/robot/motor_right", 5);
    safety_trigger_pub = node.advertise<std_msgs::Bool>("/robot/safety_master", 5);
    path_array_pub = node.advertise<std_msgs::String>("/robot/path_array", 5);
    gui_connection_pub = node.advertise<std_msgs::Bool>("/robot/gui_connection", 2);

    start();
    std::cout << "Successfully initialized node." << std::endl;
    ros_master_on = true;
    return true;
}

void QNode::run() {
    // QThread function
    ros::Rate loop_rate(10); // too fast loop rate crashed the GUI
    std::cout << "Node is running." << std::endl;
    while (ros::ok()){
        this->no_connection_cycle_counter += 1;     // ros will constantly check if "robot/connection" topic is publishing true
        std_msgs::Bool connection_msg;              // if it doesn't happend withink 10 iteration (which is less than a sec) it'll set itself as 'no connection'
        connection_msg.data = true;                 // to avoid it, make a node on the robot that constantly publishes true to the specified topic
        gui_connection_pub.publish(connection_msg); // additionally, the gui will publish true to "gui_connection" topic, to let the robot know it reads.
                                                    // you can remove this section if you dont need your robot to know it has connection.
        if (no_connection_cycle_counter > 10){
            this->connection_robot = false;
            Q_EMIT msgSubscribed();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
    //Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::safetyCallBack(const std_msgs::Bool::ConstPtr &msg){// done
    this->safety_master = msg->data;
    Q_EMIT msgSubscribed(); // tells the "UpdateGUI" to update the new published value
}

void QNode::headingCallBack(const std_msgs::Int64::ConstPtr &msg){
    this->heading = msg->data;
    Q_EMIT msgSubscribed();
}

void QNode::motor_leftCallBack(const std_msgs::Int64::ConstPtr &msg){
    this->motor_left = msg->data;
    Q_EMIT msgSubscribed();
}

void QNode::motor_rightCallBack(const std_msgs::Int64::ConstPtr &msg){
    this->motor_right = msg->data;
    Q_EMIT msgSubscribed();
}

void QNode::gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr &msg){
    this->latitude = msg->latitude;
    this->longitude = msg->longitude;
    this->sat_count = msg->status.service;
    Q_EMIT msgSubscribed();
}

void QNode::robot_connectionCallBack(const std_msgs::Bool::ConstPtr &msg){
    this->no_connection_cycle_counter = 0;
    this->connection_robot = msg->data;
    Q_EMIT msgSubscribed();
}

void QNode::battery_voltageCallBack(const std_msgs::Float64::ConstPtr &msg){
    this->battery_voltage = msg->data;
    Q_EMIT msgSubscribed();
}

void QNode::pitchCallBack(const std_msgs::Int64::ConstPtr &msg){
    this->pitch = msg->data;
    Q_EMIT msgSubscribed();
}

void QNode::rollCallBack(const std_msgs::Int64::ConstPtr &msg){
    this->roll = msg->data;
    Q_EMIT msgSubscribed();
}

