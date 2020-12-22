#include "backend.h"
#include <QObject>
#include <QQuickItem>

BackEnd::BackEnd(int argc, char** argv, QQuickItem *item,QObject *parent) :
    QObject(parent), qnode(argc, argv)
{
    ros_initiated = qnode.init(); // initiates ROS using the init method.

    if (!ros_initiated){
        std::cout << "ROS is not online, closing" << std::endl;
        QCoreApplication::quit();
    }

    // Lines of connections between signals and slots, connects qnode variables with backend. and vice versa!
    QObject::connect(&qnode, SIGNAL(msgSubscribed()), this, SLOT(updateGUI()));
    // connect up button signals to slots
    QObject::connect(item, SIGNAL(upButtonPressed()), this, SLOT(onUpButton_Pressed()));
    QObject::connect(item, SIGNAL(upButtonReleased()), this, SLOT(onUpButton_Released()));
    // connect down button signals to slots
    QObject::connect(item, SIGNAL(downButtonPressed()), this, SLOT(onDownButton_Pressed()));
    QObject::connect(item, SIGNAL(downButtonReleased()), this, SLOT(onDownButton_Released()));
    // connect right button signals to slots
    QObject::connect(item, SIGNAL(rightButtonPressed()), this, SLOT(onRightButton_Pressed()));
    QObject::connect(item, SIGNAL(rightButtonReleased()), this, SLOT(onRightButton_Released()));
    // connect left button signals to slots
    QObject::connect(item, SIGNAL(leftButtonPressed()), this, SLOT(onLeftButton_Pressed()));
    QObject::connect(item, SIGNAL(leftButtonReleased()), this, SLOT(onLeftButton_Released()));
    // connect safety button signals to slots
    QObject::connect(item, SIGNAL(safetyButtonPressed()), this, SLOT(onSafetyButton_Pressed()));
    QObject::connect(item, SIGNAL(safetyButtonReleased()), this, SLOT(onSafetyButton_Released()));
    // connect control panel button signals to slots
    QObject::connect(item, SIGNAL(mainPanelButtonClicked()), this, SLOT(onMainPanelButton_Clicked()));
    QObject::connect(item, SIGNAL(mapPanelButtonClicked()), this, SLOT(onMapPanelButton_Clicked()));
    QObject::connect(item, SIGNAL(compPanelButtonClicked()), this, SLOT(onCompPanelButton_Clicked()));
    // connect slider value siganls to slots
    QObject::connect(item, SIGNAL(speedSliderValueChanged(int)), this, SLOT(onSpeedSlider_ValueChanged(int)));
    // connect map coordinate value signals to slots
    QObject::connect(item, SIGNAL(coordinateValueClicked(QString, QString)), this, SLOT(onCoordinateValueClicked(QString, QString)));
    QObject::connect(item, SIGNAL(startRouteButtonClicked()), this, SLOT(onStartRouteButton_Clicked()));
    QObject::connect(item, SIGNAL(zeroPosButtonClicked()), this, SLOT(onZeroPosButton_Clicked()));
    QObject::connect(item, SIGNAL(cancelRouteButtonClicked()), this, SLOT(onCancelRouteButton_Clicked()));

    // pretty much: launch settings, defines initiated values to variables.
    setLeftGaugeVal("0");
    setLeftArc(210);
    setRightGaugeVal("0");
    setRightArc(0);
    setLeftGaugeOpacity(100);
    setRightGaugeOpacity(100);
    setIsSafetyEngaged(true);
    setSafetyTriggerOpacity(0); // safety opacity follow the opacity of the buttons (showing when safety is off)
    setSafetyImageUrl("Safety-engaged.png");
    setMainPanelOpacity(100);
    setCompGraphicOpacity(0);
    setEmptyPanelOpacity(0);
    setMapEnabled(false);
    setMapPanelOpacity(0);

    // publish initiating values
    // engage safety on initiation
    std_msgs::Bool safety_master_init;
    safety_master_init.data = true;
    qnode.safety_trigger_pub.publish(safety_master_init);
    // reset driving path
    selected_path_str = "[]";
    std_msgs::String path_msg;
    path_msg.data = selected_path_str;
    qnode.path_array_pub.publish(path_msg);

}

// Functions area - all the methods are placed here, (In order of development!)

// convert std::string to QString
QString BackEnd::toQString(std::string const &s)
{
    return QString::fromUtf8(s.c_str());
}
// convert from QString to std::string
std::string BackEnd::fromQString(QString const &s)
{
    return std::string(s.toUtf8().constData());
}

// method that simplifies the publishing to motors, as they apear frequently in the project.
// creates two Int64 ros message objects, and publishes them through qnode.
void BackEnd::publishToMotors(int motor_left, int motor_right){
    std_msgs::Int64 motor_right_msg;
    std_msgs::Int64 motor_left_msg;
    motor_right_msg.data = motor_right;
    motor_left_msg.data = motor_left;
    qnode.motor_right_pub.publish(motor_right_msg);
    qnode.motor_left_pub.publish(motor_left_msg);
}
// function used to convert lat,long point to str in pythonic form.
std::string BackEnd::LatLongStrPoint(std::string latitude, std::string longitude){
    std::string point_str = "[";
    point_str += latitude;
    point_str += ", ";
    point_str += longitude;
    point_str += "]";
    return point_str;
}


// Main gui updating component - every change in the qnode values (published values)
// is triggering this method, which in return updates the values in the QML.
// handles pretty much all that is happening in the GUI.
void BackEnd::updateGUI(){
    int right_motor_val = qnode.motor_right;
    int left_motor_val = qnode.motor_left;
    int right_motor_abs = std::abs(right_motor_val);
    int left_motor_abs = std::abs(right_motor_val);
    bool safetyEngaged = qnode.safety_master;
    int headingValue = qnode.heading;
    bool self_driving = qnode.self_driving;
    int relative_bearing = qnode.relative_bearing;
    int target_bearing = qnode.target_bearing;
    int target_distance = qnode.target_distance;
    int path_passed = qnode.path_passed;
    QString target_cords = toQString(qnode.target_cords);
    bool robot_connection = qnode.connection_robot;


    // map info values - used for visualization
    setRelativeBearingVal(QString::number(relative_bearing));
    setTargetBearingVal(QString::number(target_bearing));
    setTargetDistanceVal(QString::number(target_distance));
    setTargetCordsVal(target_cords);
    setPathPassedVal(QStringLiteral("%1 %").arg(path_passed));

    // gps vals needed as FLOATS for the map (used for calculation rather than visualization)
    setGPSLatVal(qnode.latitude);
    setGPSLongVal(qnode.longitude);

    // set variables that are on the panel and independent of state - used for visualization
    setPanelHeadingVal(QString::number(headingValue));
    setCompRotationVal(headingValue);
    setPanelLatVal(QString::number(qnode.latitude));
    setPanelLongVal(QString::number(qnode.longitude));
    setPanelSatVal(QString::number(qnode.sat_count));
    setPanelBatteryVal(QString::number(qnode.battery_voltage));
    setPanelPitchVal(QString::number(qnode.pitch));
    setPanelRollVal(QString::number(qnode.roll));


    if (left_motor_val >= 0 and left_motor_val <= 400){
        setLeftGaugeVal(QString::number(left_motor_abs));
        int motor_toLeftArc = 210 - (left_motor_abs * 185) / 400; // convert motor values to arc values
        setLeftArc(motor_toLeftArc);
        setLeftGaugeColor("#4df6ff");
        setLeftGaugeOpacity(100);
    }    

    if (right_motor_val >= 0 and right_motor_val <= 400){
        setRightGaugeVal(QString::number(right_motor_abs));
        int motor_toRightArc = (right_motor_abs * 180) / 400; // convert motor values to arc values
        setRightArc(motor_toRightArc);
        setRightGaugeColor("#4df6ff");
        setRightGaugeOpacity(100);
    }

    if (right_motor_val < 0 and right_motor_val >= -400){
        int motor_toRightArc = (right_motor_abs * 180) / 400; // convert motor values to arc values
        setRightArc(motor_toRightArc);
        setRightGaugeColor("#eeeeee");
        setRightGaugeOpacity(0);

    }
    if (left_motor_val < 0 and left_motor_val >= -400){
        int motor_toLeftArc = 210 - (left_motor_abs * 185) / 400; // convert motor values to arc values
        setLeftArc(motor_toLeftArc);
        setLeftGaugeColor("#eeeeee");
        setLeftGaugeOpacity(0);
    }

    if (safetyEngaged == true){
        if (this->isSafetyButtonPressed == false){
            setSafetyImageUrl("Safety-engaged.png");
        }
        setButtonsEnabled(false);
        setSafetyTriggerOpacity(0);
        setPanelSafetyVal("ENGAGED");
        setButtonsOpacity(0);
    }

    if (safetyEngaged == false){
        if (this->isSafetyButtonPressed == false){
            setSafetyImageUrl("Safety-disengaged.png");
        }

        setSafetyTriggerOpacity(100);
        setPanelSafetyVal("DISENGAGED");
        if (self_driving == false){
            setButtonsEnabled(true);
            setButtonsOpacity(100);
        }
    }

    if (self_driving == true){
        setButtonsEnabled(false);
        setButtonsOpacity(0);
        setDrivingInfoOpacity(100);
    }

    if (self_driving == false and safetyEngaged == false){
        setButtonsEnabled(true);
        setButtonsOpacity(100);
        setDrivingInfoOpacity(0);
    }

    if (robot_connection == false){
        setPanelConnectVal("NO CONNECTION");
    }

    if (robot_connection == true){
        setPanelConnectVal("ONLINE");
    }

    if (robot_connection == false or safetyEngaged == true){
        setStartRouteButtonEnabled(false);
    }

    if (robot_connection == true and safetyEngaged == false){
        setStartRouteButtonEnabled(true);
    }


}

// button slots to make actions based on user buttons pressed.

void BackEnd::onUpButton_Pressed(){
    publishToMotors(speedControlVal, speedControlVal);
}

void BackEnd::onUpButton_Released(){
    publishToMotors(0, 0);
}

void BackEnd::onDownButton_Pressed(){
    publishToMotors(-1 * speedControlVal, -1 * speedControlVal);
}

void BackEnd::onDownButton_Released(){
    publishToMotors(0, 0);
}

void BackEnd::onRightButton_Pressed(){
    publishToMotors(speedControlVal, -1 * speedControlVal);
}

void BackEnd::onRightButton_Released(){
    publishToMotors(0, 0);
}

void BackEnd::onLeftButton_Pressed(){
    publishToMotors(-1 * speedControlVal, speedControlVal);
}

void BackEnd::onLeftButton_Released(){
    publishToMotors(0, 0);
}

void BackEnd::onSafetyButton_Pressed(){
    isSafetyButtonPressed = true;
    QString current_SafetyImage = getSafetyImageUrl();
    if (current_SafetyImage == "Safety-engaged.png"){
        setSafetyImageUrl("Safety-engaged-pressed.png");
    }
    else{
        setSafetyImageUrl("Safety-disengaged-pressed.png");
    }
}

void BackEnd::onSafetyButton_Released(){
    isSafetyButtonPressed = false;
    std_msgs::Bool safety_trigger;
    if (qnode.safety_master == false){
        safety_trigger.data = true;
        qnode.safety_trigger_pub.publish(safety_trigger);
        setSafetyImageUrl("Safety-engaged.png");
    }
    else if (qnode.safety_master == true){
        safety_trigger.data = false;
        qnode.safety_trigger_pub.publish(safety_trigger);
        setSafetyImageUrl("Safety-disengaged.png");
    }


}

void BackEnd::onMainPanelButton_Clicked(){
    setMainPanelOpacity(100);
    setCompGraphicOpacity(0);
    setEmptyPanelOpacity(0);
    setMapPanelOpacity(0);
    setSliderEnabled(true);
    setMapEnabled(false);
}

void BackEnd::onMapPanelButton_Clicked(){
    setMainPanelOpacity(0);
    setCompGraphicOpacity(0);
    setEmptyPanelOpacity(0);
    setMapPanelOpacity(100);
    setMapEnabled(true);
    setSliderEnabled(false);
}

void BackEnd::onCompPanelButton_Clicked(){
    setMainPanelOpacity(0);
    setMapPanelOpacity(0);
    setCompGraphicOpacity(100);
    setEmptyPanelOpacity(100);
    setSliderEnabled(false);
    setMapEnabled(false);
}

void BackEnd::onSpeedSlider_ValueChanged(const int &value){
    this->speedControlVal = value;
    setPanelSpeedVal(QString::number(this->speedControlVal));
}

void BackEnd::onCoordinateValueClicked(const QString &latitude, const QString &longitude){
    std::string latitude_str = fromQString(latitude);
    std::string longitude_str = fromQString(longitude);
    //std::cout << latitude_str << ", " << longitude_str << std::endl;
    double num_lat = latitude.toDouble();
    double num_long = longitude.toDouble();
    QGeoCoordinate coordinate;
    coordinate.setAltitude(25);
    coordinate.setLatitude(num_lat);
    coordinate.setLongitude(num_long);
    selected_path.addCoordinate(coordinate);
    setGeoPath(selected_path);
    std::string point_str = LatLongStrPoint(latitude_str, longitude_str);

    if (selected_path_str.size() == 2){ // these statements decide how to append new coordinate to keep pythonic structure
        selected_path_str.insert(1, point_str);
    }
    else{
        selected_path_str.insert(selected_path_str.size() - 1, ", ");
        selected_path_str.insert(selected_path_str.size() - 1, point_str);
    }

}

void BackEnd::onStartRouteButton_Clicked(){
    //std::cout << "StartRouteClicked" << std::endl;
    std_msgs::String path_msg;
    path_msg.data = selected_path_str;
    qnode.path_array_pub.publish(path_msg);
    setButtonsEnabled(false);
    setButtonsOpacity(0);
    setDrivingInfoOpacity(100);

}

void BackEnd::onZeroPosButton_Clicked(){
    //std::cout << "ZeroPosClicked" << std::endl;
}

void BackEnd::onCancelRouteButton_Clicked(){
    selected_path.clearPath();
    setGeoPath(selected_path);
    selected_path_str = "[]";
    std_msgs::String path_msg;
    path_msg.data = selected_path_str;
    qnode.path_array_pub.publish(path_msg);
    if (qnode.safety_master == false) {
        setButtonsEnabled(true);
        setButtonsOpacity(100);
    }
    setDrivingInfoOpacity(0);
    publishToMotors(0, 0);

}
