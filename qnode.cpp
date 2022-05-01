#include "qnode.h"
#include <iostream>


QNode::QNode(int argc, char** argv, QObject *parent) : QObject(parent)
{

    // Init ROS2
    rclcpp::init(argc, argv);

    // Create a static executor - faster but doesn't allow creating nodes during runtime
    // Use MultiThreadedExecutor for runtime node creation.
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr rosExecutor;
    rosExecutor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    m_RosNode = rclcpp::Node::make_shared("RobotGuiNode");
    rosExecutor->add_node(m_RosNode);

    // Create the required publishers, please note the ROS2 feature: QoS
    // by default we use the SystemDefaultQoS which is similar to ros1 tcp style.
    // If you want UDP data for imaging or unreliable network like RF use "rclcpp::SensorDataQoS()" instead.
    motorLeft_Pub = m_RosNode->create_publisher<std_msgs::msg::Int32>("/robot/motor_left", rclcpp::SystemDefaultsQoS());
    motorRight_Pub = m_RosNode->create_publisher<std_msgs::msg::Int32>("/robot/motor_right", rclcpp::SystemDefaultsQoS());
    safetyTrigger_Pub = m_RosNode->create_publisher<std_msgs::msg::Bool>("/robot/safety_master", rclcpp::SystemDefaultsQoS());
    pathList_Pub = m_RosNode->create_publisher<std_msgs::msg::String>("/robot/path_array", rclcpp::SystemDefaultsQoS());
    guiConnection_Pub = m_RosNode->create_publisher<std_msgs::msg::Bool>("/robot/gui_connection", rclcpp::SystemDefaultsQoS());

    // assign subscriptions
    m_Safety_Sub = m_RosNode->create_subscription<std_msgs::msg::Bool>("/robot/safety_master",
                                                                   rclcpp::SystemDefaultsQoS(),
                                                                   std::bind(&QNode::m_Safety_Callback,this,std::placeholders::_1));
    m_Heading_Sub = m_RosNode->create_subscription<std_msgs::msg::Int32>("/robot/imu/heading",
                                                                   rclcpp::SystemDefaultsQoS(),
                                                                   std::bind(&QNode::m_Heading_Callback,this,std::placeholders::_1));
    m_MotorLeft_Sub = m_RosNode->create_subscription<std_msgs::msg::Int32>("/robot/motor_left",
                                                                    rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&QNode::m_MotorLeft_Callback,this,std::placeholders::_1));
    m_MotorRight_Sub = m_RosNode->create_subscription<std_msgs::msg::Int32>("/robot/motor_right",
                                                                    rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&QNode::m_MotorRight_Callback,this,std::placeholders::_1));
    m_GPS_Sub = m_RosNode->create_subscription<sensor_msgs::msg::NavSatFix>("/robot/gps",
                                                                    rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&QNode::m_GPS_Callback,this,std::placeholders::_1));
    m_RobotConnection_Sub = m_RosNode->create_subscription<std_msgs::msg::Bool>("/robot/connection",
                                                                    rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&QNode::m_RobotConnection_Callback,this,std::placeholders::_1));
    m_BatteryVoltage_sub = m_RosNode->create_subscription<std_msgs::msg::Float32>("/robot/battery_voltage",
                                                                    rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&QNode::m_BatteryVoltage_Callback,this,std::placeholders::_1));
    m_Pitch_Sub = m_RosNode->create_subscription<std_msgs::msg::Int32>("/robot/imu/pitch",
                                                                    rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&QNode::m_Pitch_Callback,this,std::placeholders::_1));
    m_Roll_Sub = m_RosNode->create_subscription<std_msgs::msg::Int32>("/robot/imu/roll",
                                                                     rclcpp::SystemDefaultsQoS(),
                                                                     std::bind(&QNode::m_Roll_Callback,this,std::placeholders::_1));


    // place the spin of ROS2 (which is a blocking infinite loop) in a seperate thread and detach it
    // Unlike ROS1, we use std::thread for threading instead of the QThread
    // Might return to QThread in the future if I'll see game changing flaws.
    std::thread executor_thread(std::bind(&rclcpp::executors::StaticSingleThreadedExecutor::spin, rosExecutor));
    executor_thread.detach();
}

void QNode::m_Safety_Callback(const std_msgs::msg::Bool::SharedPtr msg){
    this->safety_master = msg->data;

    Q_EMIT this->msgSubscribed();
}

void QNode::m_Heading_Callback(const std_msgs::msg::Int32::SharedPtr msg){
    this->heading = msg->data;
    Q_EMIT this->msgSubscribed();

}

void QNode::m_MotorLeft_Callback(const std_msgs::msg::Int32::SharedPtr msg){
    this->motor_left = msg->data;
    Q_EMIT this->msgSubscribed();
}

void QNode::m_MotorRight_Callback(const std_msgs::msg::Int32::SharedPtr msg){
    this->motor_right = msg->data;
    Q_EMIT this->msgSubscribed();
}

void QNode::m_GPS_Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
    this->latitude = msg->altitude;
    this->longitude = msg->longitude;
    this->sat_count = msg->status.service; // not really what this value is used for, change to your liking
    Q_EMIT this->msgSubscribed();
}

void QNode::m_RobotConnection_Callback(const std_msgs::msg::Bool::SharedPtr msg){
    this->connection_robot = msg->data;
    Q_EMIT this->msgSubscribed();
}

void QNode::m_BatteryVoltage_Callback(const std_msgs::msg::Float32::SharedPtr msg){
    this->battery_voltage = msg->data;
    Q_EMIT this->msgSubscribed();
}

void QNode::m_Pitch_Callback(const std_msgs::msg::Int32::SharedPtr msg){
    this->pitch = msg->data;
    Q_EMIT this->msgSubscribed();
}

void QNode::m_Roll_Callback(const std_msgs::msg::Int32::SharedPtr msg){
    this->roll = msg->data;
    Q_EMIT this->msgSubscribed();
}
