#ifndef QNODE_H
#define QNODE_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char**argv);
    virtual ~QNode();
    bool init();
    void run();
    ros::Publisher motor_left_pub;
    ros::Publisher motor_right_pub;
    ros::Publisher safety_trigger_pub;
    ros::Publisher path_array_pub;
    ros::Publisher gui_connection_pub;

    // callback
    void safetyCallBack(const std_msgs::Bool::ConstPtr& msg);
    void headingCallBack(const std_msgs::Int64::ConstPtr& msg);
    void motor_leftCallBack(const std_msgs::Int64::ConstPtr& msg);
    void motor_rightCallBack(const std_msgs::Int64::ConstPtr& msg);
    void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void self_drivingCallBack(const std_msgs::Bool::ConstPtr& msg);
    void relative_bearingCallBack(const std_msgs::Int64::ConstPtr& msg);
    void target_bearingCallBack(const std_msgs::Int64::ConstPtr& msg);
    void target_distanceCallBack(const std_msgs::Int64::ConstPtr& msg);
    void path_passedCallBack(const std_msgs::Int64::ConstPtr& msg);
    void target_cordsCallBack(const std_msgs::String::ConstPtr& msg); // Notice! its a STRING
    void robot_connectionCallBack(const std_msgs::Bool::ConstPtr& msg);
    void battery_voltageCallBack(const std_msgs::Float64::ConstPtr& msg);
    void pitchCallBack(const std_msgs::Int64::ConstPtr& msg);
    void rollCallBack(const std_msgs::Int64::ConstPtr& msg);

    // Variables:
    int no_connection_cycle_counter = 0;

        // Pub messages:
    int motor_left = 0; // the motor data that is published and subscribed
    int motor_right = 0;
    int safety_trigger = true; // safety trigger to control safety_master if master allows (not in use)
    bool ros_master_on = false;
    bool connection_gui = true;

        // Sub messages:
    bool safety_master = true; // safety master to tell if motor lock is engaged
    int heading = 0; // value of heading coming from the IMU
    float latitude = 32.046874;
    float longitude = 34.860803;
    int sat_count = 0;
    bool self_driving = false;
    int target_bearing = 0;
    int relative_bearing = 0;
    std::string target_cords = "NODATA";
    int path_passed = 0;
    int target_distance = 0;
    bool connection_robot = false;
    float battery_voltage = 0.0;
    int pitch = 0;
    int roll = 0;


Q_SIGNALS:
    void msgSubscribed();


private:
    int init_argc;
    char** init_argv;
    ros::Subscriber heading_sub;
    ros::Subscriber safety_sub;
    ros::Subscriber motor_left_sub;
    ros::Subscriber motor_right_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber self_driving_sub;
    ros::Subscriber relative_bearing_sub;
    ros::Subscriber target_bearing_sub;
    ros::Subscriber target_distance_sub;
    ros::Subscriber target_cords_sub;
    ros::Subscriber path_passed_sub;
    ros::Subscriber robot_connection_sub;
    ros::Subscriber battery_voltage_sub;
    ros::Subscriber pitch_sub;
    ros::Subscriber roll_sub;

};

#endif // QNODE_H
