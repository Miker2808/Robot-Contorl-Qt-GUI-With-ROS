#ifndef QNODE_H
#define QNODE_H

#include <QObject>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <thread>

class QNode : public QObject {
    Q_OBJECT
public:
    QNode(int argc = 0, char** argv = nullptr, QObject *parent = nullptr);

    // Designed so publisher and callback outputs are public,
    // while subscribers are private.

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorLeft_Pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorRight_Pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safetyTrigger_Pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pathList_Pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr guiConnection_Pub;

    // Pub messages:
    int motor_left = 0; // the motor data that is published and subscribed
    int motor_right = 0;
    int safety_trigger = true; // safety trigger to control safety_master if master allows (not in use)
    bool ros_master_on = false;
    bool connection_gui = true;

    // Sub messages:
    bool safety_master = true; // safety master to tell if motor lock is engaged
    int heading = 0;
    float latitude = 45.703547;
    float longitude = 21.302171;
    int sat_count = 0;
    bool connection_robot = false;
    float battery_voltage = 0.0;
    int pitch = 0;
    int roll = 0;

Q_SIGNALS:
    // used to tell the backend that new ros data arrived and to update the gui
    void msgSubscribed();

private:
    rclcpp::Node::SharedPtr m_RosNode;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_Heading_Sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_Safety_Sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_MotorLeft_Sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_MotorRight_Sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_GPS_Sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_RobotConnection_Sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_BatteryVoltage_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_Pitch_Sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_Roll_Sub;

    void m_Heading_Callback(const std_msgs::msg::Int32::SharedPtr msg);
    void m_Safety_Callback(const std_msgs::msg::Bool::SharedPtr msg);
    void m_MotorLeft_Callback(const std_msgs::msg::Int32::SharedPtr msg);
    void m_MotorRight_Callback(const std_msgs::msg::Int32::SharedPtr msg);
    void m_GPS_Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void m_RobotConnection_Callback(const std_msgs::msg::Bool::SharedPtr msg);
    void m_BatteryVoltage_Callback(const std_msgs::msg::Float32::SharedPtr msg);
    void m_Pitch_Callback(const std_msgs::msg::Int32::SharedPtr msg);
    void m_Roll_Callback(const std_msgs::msg::Int32::SharedPtr msg);



};

#endif // QNODE_H
