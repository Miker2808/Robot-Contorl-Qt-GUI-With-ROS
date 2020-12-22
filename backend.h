#ifndef BACKEND_H
#define BACKEND_H

#include <QString>
#include <QObject>
#include <qqml.h>
#include <QQuickItem>
#include <qnode.h>
#include <QGeoPath>

class BackEnd : public QObject
{
    Q_OBJECT
    // Properties, they connect the backend variables with qml variables, they are OBJECTS! with methods,
    // To get value from QML use the "get{VAR_NAME}" method, to write value to QML use the "set{VAR_NAME}" method
    // A signal "{VAR_NAME}Changed" is used to trigger your custom slot when QML value is changed.
    // Please notice the rules for variable names, commands (set/get) are low-caps (lowerCamelCase), Changed signal and variables are UpperCamelCase.

    Q_PROPERTY(QString LeftGaugeVal READ getLeftGaugeVal WRITE setLeftGaugeVal NOTIFY LeftGaugeValChanged)
    Q_PROPERTY(int LeftArc READ getLeftArc WRITE setLeftArc NOTIFY LeftArcChanged)
    Q_PROPERTY(QString RightGaugeVal READ getRightGaugeVal WRITE setRightGaugeVal NOTIFY RightGaugeValChanged)
    Q_PROPERTY(int RightArc READ getRightArc WRITE setRightArc NOTIFY RightArcChanged)
    Q_PROPERTY(int RightGaugeOpacity READ getRightGaugeOpacity WRITE setRightGaugeOpacity NOTIFY RightGaugeOpacityChanged)
    Q_PROPERTY(int LeftGaugeOpacity READ getLeftGaugeOpacity WRITE setLeftGaugeOpacity NOTIFY LeftGaugeOpacityChanged)
    Q_PROPERTY(QString LeftGaugeColor READ getLeftGaugeColor WRITE setLeftGaugeColor NOTIFY LeftGaugeColorChanged)
    Q_PROPERTY(QString RightGaugeColor READ getRightGaugeColor WRITE setRightGaugeColor NOTIFY RightGaugeColorChanged)
    Q_PROPERTY(bool ButtonsEnabled READ getButtonsEnabled WRITE setButtonsEnabled NOTIFY ButtonsEnabledChanged)
    Q_PROPERTY(int SafetyTriggerOpacity READ getSafetyTriggerOpacity WRITE setSafetyTriggerOpacity NOTIFY SafetyTriggerOpacityChanged)
    Q_PROPERTY(bool IsSafetyEngaged READ getIsSafetyEngaged WRITE setIsSafetyEngaged NOTIFY IsIsSafetyEngaged)
    Q_PROPERTY(QString SafetyImageUrl READ getSafetyImageUrl WRITE setSafetyImageUrl NOTIFY SafetyImageUrlChanged)
    Q_PROPERTY(QString PanelSafetyVal READ getPanelSafetyVal WRITE setPanelSafetyVal NOTIFY PanelSafetyValChanged)
    Q_PROPERTY(QString PanelConnectVal READ getPanelConnectVal WRITE setPanelConnectVal NOTIFY PanelConnectValChanged)
    Q_PROPERTY(QString PanelLongVal READ getPanelLongVal WRITE setPanelLongVal NOTIFY PanelLongValChanged)
    Q_PROPERTY(QString PanelLatVal READ getPanelLatVal WRITE setPanelLatVal NOTIFY PanelLatValChanged)
    Q_PROPERTY(QString PanelSatVal READ getPanelSatVal WRITE setPanelSatVal NOTIFY PanelSatValChanged)
    Q_PROPERTY(QString PanelHeadingVal READ getPanelHeadingVal WRITE setPanelHeadingVal NOTIFY PanelHeadingValChanged)
    Q_PROPERTY(QString PanelPitchVal READ getPanelPitchVal WRITE setPanelPitchVal NOTIFY PanelPitchValChanged)
    Q_PROPERTY(QString PanelRollVal READ getPanelRollVal WRITE setPanelRollVal NOTIFY PanelRollValChanged)
    Q_PROPERTY(QString PanelBatteryVal READ getPanelBatteryVal WRITE setPanelBatteryVal NOTIFY PanelBatteryValChanged)
    Q_PROPERTY(QString PanelSpeedVal READ getPanelSpeedVal WRITE setPanelSpeedVal NOTIFY PanelSpeedValChanged)
    Q_PROPERTY(int MainPanelOpacity READ getMainPanelOpacity WRITE setMainPanelOpacity NOTIFY MainPanelOpacityChanged)
    Q_PROPERTY(int EmptyPanelOpacity READ getEmptyPanelOpacity WRITE setEmptyPanelOpacity NOTIFY EmptyPanelOpacityChanged)
    Q_PROPERTY(int CompGraphicOpacity READ getCompGraphicOpacity WRITE setCompGraphicOpacity NOTIFY CompGraphicOpacityChanged)
    Q_PROPERTY(int CompRotationVal READ getCompRotationVal WRITE setCompRotationVal NOTIFY CompRotationValChanged)
    Q_PROPERTY(bool SliderEnabled READ getSliderEnabled WRITE setSliderEnabled NOTIFY SliderEnabledChanged)
    Q_PROPERTY(bool MapEnabled READ getMapenabled WRITE setMapEnabled NOTIFY MapEnabledChanged)
    Q_PROPERTY(QGeoPath GeoPath READ getGeoPath WRITE setGeoPath NOTIFY GeoPathChanged)
    Q_PROPERTY(int MapPanelOpacity READ getMapPanelOpacity WRITE setMapPanelOpacity NOTIFY MapPanelOpacityChanged)
    Q_PROPERTY(float GPSLatVal READ getGPSLatVal WRITE setGPSLatVal NOTIFY GPSLatValChanged)
    Q_PROPERTY(float GPSLongVal READ getGPSLongVal WRITE setGPSLongVal NOTIFY GPSLongValChanged)
    Q_PROPERTY(int ButtonsOpacity READ getButtonsOpacity WRITE setButtonsOpacity NOTIFY ButtonsOpacityChanged)
    Q_PROPERTY(int DrivingInfoOpacity READ getDrivingInfoOpacity WRITE setDrivingInfoOpacity NOTIFY DrivingInfoOpacityChanged)
    Q_PROPERTY(QString RelativeBearingVal READ getRelativeBearingVal WRITE setRelativeBearingVal NOTIFY RelativeBearingValChanged)
    Q_PROPERTY(QString TargetBearingVal READ getTargetBearingVal WRITE setTargetBearingVal NOTIFY TargetBearingValChanged)
    Q_PROPERTY(QString TargetDistanceVal READ getTargetDistanceVal WRITE setTargetDistanceVal NOTIFY TargetDistanceValChanged)
    Q_PROPERTY(QString TargetCordsVal READ getTargetCordsVal WRITE setTargetCordsVal NOTIFY TargetCordsValChanged)
    Q_PROPERTY(QString PathPassedVal READ getPathPassedVal WRITE setPathPassedVal NOTIFY PathPassedValChanged)
    Q_PROPERTY(bool StartRouteButtonEnabled READ StartRouteButtonEnabled WRITE setStartRouteButtonEnabled NOTIFY StartRouteButtonEnabledChanged)
    // Q_PROPERTY generates using "refractor" additionaly 3 methods and one object for each property, everything below, is automatically generated
    // What is automatically generated is kept at the bottom, manually generated at top. You'll see a comment for automatically generated methods.
    QML_ELEMENT


public:
    explicit BackEnd(int argc, char** argv, QQuickItem *item = nullptr,QObject *parent = nullptr);

    bool ros_initiated = false;

    QString toQString(std::string const &s);

    std::string fromQString(QString const &s);

    void publishToMotors(int motor_left, int motor_right);

    std::string LatLongStrPoint(std::string latitude, std::string longitude);

    bool isSafetyButtonPressed = false;


    // QML connection functions - generated by Qt
    QString getLeftGaugeVal() const{ return m_LeftGaugeVal; }

    int getLeftArc() const{ return m_LeftArc; }

    QString getRightGaugeVal() const{ return m_RightGaugeVal; }

    int getRightArc() const{ return m_RightArc; }

    int getRightGaugeOpacity() const{ return m_RightGaugeOpacity; }
    int getLeftGaugeOpacity() const{ return m_LeftGaugeOpacity; }

    QString getLeftGaugeColor() const{ return m_LeftGaugeColor; }
    QString getRightGaugeColor() const{ return m_RightGaugeColor; }

    bool getButtonsEnabled() const { return m_ButtonsEnabled; }

    int getSafetyTriggerOpacity() const { return m_SafetyTriggerOpacity; }

    bool getIsSafetyEngaged() const { return m_IsSafetyEngaged; }

    QString getSafetyImageUrl() const { return m_SafetyImageUrl; }

    QString getPanelSafetyVal() const { return m_PanelSafetyVal; }

    QString getPanelConnectVal() const { return m_PanelConnectVal; }

    QString getPanelLongVal() const { return m_PanelLongVal; }

    QString getPanelLatVal() const { return m_PanelLatVal; }

    QString getPanelHeadingVal() const { return m_PanelHeadingVal; }

    QString getPanelBatteryVal() const { return m_PanelBatteryVal; }

    QString getPanelSpeedVal() const { return m_PanelSpeedVal; }

    int getMainPanelOpacity() const { return m_MainPanelOpacity; }

    int getEmptyPanelOpacity() const { return m_EmptyPanelOpacity; }

    int getCompGraphicOpacity() const { return m_CompGraphicOpacity; }

    int getCompRotationVal() const { return m_CompRotationVal; }

    bool getSliderEnabled() const { return m_SliderEnabled; }

    bool getMapenabled() const { return m_MapEnabled; }

    QGeoPath getGeoPath() const { return m_GeoPath; }

    int getMapPanelOpacity() const { return m_MapPanelOpacity; }

    QString getPanelPitchVal() const { return m_PanelPitchVal; }

    QString getPanelRollVal() const { return m_PanelRollVal; }

    QString getPanelSatVal() const { return m_PanelSatVal; }

    float getGPSLatVal() const { return m_GPSLatVal; }

    float getGPSLongVal() const { return m_GPSLongVal; }

    int getButtonsOpacity() const { return m_ButtonsOpacity; }

    int getDrivingInfoOpacity() const { return m_DrivingInfoOpacity; }

    QString getRelativeBearingVal() const { return m_RelativeBearingVal; }

    QString getTargetBearingVal() const { return m_TargetBearingVal; }

    QString getTargetDistanceVal() const { return m_TargetDistanceVal; }

    QString getTargetCordsVal() const { return m_TargetCordsVal; }

    QString getPathPassedVal() const {  return m_PathPassedVal; }

    bool StartRouteButtonEnabled() const { return m_StartRouteButtonEnabled; }

public slots:
    void updateGUI(); // updates the gui when ROS data is published
    // upbutton slots
    void onUpButton_Pressed();
    void onUpButton_Released();
    // downbutton slots
    void onDownButton_Pressed();
    void onDownButton_Released();
    // rightbutton slots
    void onRightButton_Pressed();
    void onRightButton_Released();
    // leftbutton slots
    void onLeftButton_Pressed();
    void onLeftButton_Released();
    // safetybutton slots
    void onSafetyButton_Pressed();
    void onSafetyButton_Released();
    // control panel slots
    // main panel button slots
    void onMainPanelButton_Clicked();
    // map panel button slots
    void onMapPanelButton_Clicked();
    // compass panel button slots
    void onCompPanelButton_Clicked();
    // slider value changed slots
    void onSpeedSlider_ValueChanged(const int &value);
    // map slots
    void onCoordinateValueClicked(const QString &latitude, const QString &longitude);
    void onStartRouteButton_Clicked();
    void onZeroPosButton_Clicked();
    void onCancelRouteButton_Clicked();


    // QML connection Slots -- autogenerated by Qt (related to Q_PROPERTY)
    void setLeftGaugeVal(QString LeftGaugeVal)
    {
        if (m_LeftGaugeVal == LeftGaugeVal)
            return;

        m_LeftGaugeVal = LeftGaugeVal;
        emit LeftGaugeValChanged(m_LeftGaugeVal);
    }

    void setLeftArc(int LeftArc)
    {
        if (m_LeftArc == LeftArc)
            return;

        m_LeftArc = LeftArc;
        emit LeftArcChanged(m_LeftArc);
    }

    void setRightGaugeVal(QString RightGaugeVal)
    {
        if (m_RightGaugeVal == RightGaugeVal)
            return;

        m_RightGaugeVal = RightGaugeVal;
        emit RightGaugeValChanged(m_RightGaugeVal);
    }

    void setRightArc(int RightArc)
    {
        if (m_RightArc == RightArc)
            return;

        m_RightArc = RightArc;
        emit RightArcChanged(m_RightArc);
    }


    void setRightGaugeOpacity(int RightGaugeOpacity)
    {
        if (m_RightGaugeOpacity == RightGaugeOpacity)
            return;

        m_RightGaugeOpacity = RightGaugeOpacity;
        emit RightGaugeOpacityChanged(m_RightGaugeOpacity);
    }

    void setLeftGaugeOpacity(int LeftGaugeOpacity)
    {
        if (m_LeftGaugeOpacity == LeftGaugeOpacity)
            return;

        m_LeftGaugeOpacity = LeftGaugeOpacity;
        emit LeftGaugeOpacityChanged(m_LeftGaugeOpacity);
    }

    void setLeftGaugeColor(QString LeftGaugeColor)
    {
        if (m_LeftGaugeColor == LeftGaugeColor)
            return;

        m_LeftGaugeColor = LeftGaugeColor;
        emit LeftGaugeColorChanged(m_LeftGaugeColor);
    }

    void setRightGaugeColor(QString RightGaugeColor)
    {
        if (m_RightGaugeColor == RightGaugeColor)
            return;

        m_RightGaugeColor = RightGaugeColor;
        emit RightGaugeColorChanged(m_RightGaugeColor);
    }

    void setButtonsEnabled(bool ButtonsEnabled)
    {
        if (m_ButtonsEnabled == ButtonsEnabled)
            return;

        m_ButtonsEnabled = ButtonsEnabled;
        emit ButtonsEnabledChanged(m_ButtonsEnabled);
    }

    void setSafetyTriggerOpacity(int SafetyTriggerOpacity)
    {
        if (m_SafetyTriggerOpacity == SafetyTriggerOpacity)
            return;

        m_SafetyTriggerOpacity = SafetyTriggerOpacity;
        emit SafetyTriggerOpacityChanged(m_SafetyTriggerOpacity);
    }

    void setIsSafetyEngaged(bool IsSafetyEngaged)
    {
        if (m_IsSafetyEngaged == IsSafetyEngaged)
            return;

        m_IsSafetyEngaged = IsSafetyEngaged;
        emit IsIsSafetyEngaged(m_IsSafetyEngaged);
    }

    void setSafetyImageUrl(QString SafetyImageUrl)
    {
        if (m_SafetyImageUrl == SafetyImageUrl)
            return;

        m_SafetyImageUrl = SafetyImageUrl;
        emit SafetyImageUrlChanged(m_SafetyImageUrl);
    }



    void setPanelSafetyVal(QString PanelSafetyVal)
    {
        if (m_PanelSafetyVal == PanelSafetyVal)
            return;

        m_PanelSafetyVal = PanelSafetyVal;
        emit PanelSafetyValChanged(m_PanelSafetyVal);
    }

    void setPanelConnectVal(QString PanelConnectVal)
    {
        if (m_PanelConnectVal == PanelConnectVal)
            return;

        m_PanelConnectVal = PanelConnectVal;
        emit PanelConnectValChanged(m_PanelConnectVal);
    }

    void setPanelLongVal(QString PanelLongVal)
    {
        if (m_PanelLongVal == PanelLongVal)
            return;

        m_PanelLongVal = PanelLongVal;
        emit PanelLongValChanged(m_PanelLongVal);
    }

    void setPanelLatVal(QString PanelLatVal)
    {
        if (m_PanelLatVal == PanelLatVal)
            return;

        m_PanelLatVal = PanelLatVal;
        emit PanelLatValChanged(m_PanelLatVal);
    }

    void setPanelHeadingVal(QString PanelHeadingVal)
    {
        if (m_PanelHeadingVal == PanelHeadingVal)
            return;

        m_PanelHeadingVal = PanelHeadingVal;
        emit PanelHeadingValChanged(m_PanelHeadingVal);
    }

    void setPanelBatteryVal(QString PanelBatteryVal)
    {
        if (m_PanelBatteryVal == PanelBatteryVal)
            return;

        m_PanelBatteryVal = PanelBatteryVal;
        emit PanelBatteryValChanged(m_PanelBatteryVal);
    }

    void setPanelSpeedVal(QString PanelSpeedVal)
    {
        if (m_PanelSpeedVal == PanelSpeedVal)
            return;

        m_PanelSpeedVal = PanelSpeedVal;
        emit PanelSpeedValChanged(m_PanelSpeedVal);
    }


    void setMainPanelOpacity(int MainPanelOpacity)
    {
        if (m_MainPanelOpacity == MainPanelOpacity)
            return;

        m_MainPanelOpacity = MainPanelOpacity;
        emit MainPanelOpacityChanged(m_MainPanelOpacity);
    }

    void setEmptyPanelOpacity(int EmptyPanelOpacity)
    {
        if (m_EmptyPanelOpacity == EmptyPanelOpacity)
            return;

        m_EmptyPanelOpacity = EmptyPanelOpacity;
        emit EmptyPanelOpacityChanged(m_EmptyPanelOpacity);
    }

    void setCompGraphicOpacity(int CompGraphicOpacity)
    {
        if (m_CompGraphicOpacity == CompGraphicOpacity)
            return;

        m_CompGraphicOpacity = CompGraphicOpacity;
        emit CompGraphicOpacityChanged(m_CompGraphicOpacity);
    }

    void setCompRotationVal(int CompRotationVal)
    {
        if (m_CompRotationVal == CompRotationVal)
            return;

        m_CompRotationVal = CompRotationVal;
        emit CompRotationValChanged(m_CompRotationVal);
    }

    void setSliderEnabled(bool SliderEnabled)
    {
        if (m_SliderEnabled == SliderEnabled)
            return;

        m_SliderEnabled = SliderEnabled;
        emit SliderEnabledChanged(m_SliderEnabled);
    }

    void setMapEnabled(bool MapEnabled)
    {
        if (m_MapEnabled == MapEnabled)
            return;

        m_MapEnabled = MapEnabled;
        emit MapEnabledChanged(m_MapEnabled);
    }

    void setGeoPath(QGeoPath GeoPath)
    {
        if (m_GeoPath == GeoPath)
            return;

        m_GeoPath = GeoPath;
        emit GeoPathChanged(m_GeoPath);
    }

    void setMapPanelOpacity(int MapPanelOpacity)
    {
        if (m_MapPanelOpacity == MapPanelOpacity)
            return;

        m_MapPanelOpacity = MapPanelOpacity;
        emit MapPanelOpacityChanged(m_MapPanelOpacity);
    }

    void setPanelPitchVal(QString PanelPitchVal)
    {
        if (m_PanelPitchVal == PanelPitchVal)
            return;

        m_PanelPitchVal = PanelPitchVal;
        emit PanelPitchValChanged(m_PanelPitchVal);
    }

    void setPanelRollVal(QString PanelRollVal)
    {
        if (m_PanelRollVal == PanelRollVal)
            return;

        m_PanelRollVal = PanelRollVal;
        emit PanelRollValChanged(m_PanelRollVal);
    }


    void setPanelSatVal(QString PanelSatVal)
    {
        if (m_PanelSatVal == PanelSatVal)
            return;

        m_PanelSatVal = PanelSatVal;
        emit PanelSatValChanged(m_PanelSatVal);
    }

    void setGPSLatVal(float GPSLatVal)
    {
        //qWarning("Floating point comparison needs context sanity check");
        if (qFuzzyCompare(m_GPSLatVal, GPSLatVal))
            return;

        m_GPSLatVal = GPSLatVal;
        emit GPSLatValChanged(m_GPSLatVal);
    }

    void setGPSLongVal(float GPSLongVal)
    {
        //qWarning("Floating point comparison needs context sanity check");
        if (qFuzzyCompare(m_GPSLongVal, GPSLongVal))
            return;

        m_GPSLongVal = GPSLongVal;
        emit GPSLongValChanged(m_GPSLongVal);
    }

    void setButtonsOpacity(int ButtonsOpacity)
    {
        if (m_ButtonsOpacity == ButtonsOpacity)
            return;

        m_ButtonsOpacity = ButtonsOpacity;
        emit ButtonsOpacityChanged(m_ButtonsOpacity);
    }

    void setDrivingInfoOpacity(int DrivingInfoOpacity)
    {
        if (m_DrivingInfoOpacity == DrivingInfoOpacity)
            return;

        m_DrivingInfoOpacity = DrivingInfoOpacity;
        emit DrivingInfoOpacityChanged(m_DrivingInfoOpacity);
    }

    void setRelativeBearingVal(QString RelativeBearingVal)
    {
        if (m_RelativeBearingVal == RelativeBearingVal)
            return;

        m_RelativeBearingVal = RelativeBearingVal;
        emit RelativeBearingValChanged(m_RelativeBearingVal);
    }

    void setTargetBearingVal(QString TargetBearingVal)
    {
        if (m_TargetBearingVal == TargetBearingVal)
            return;

        m_TargetBearingVal = TargetBearingVal;
        emit TargetBearingValChanged(m_TargetBearingVal);
    }

    void setTargetDistanceVal(QString TargetDistanceVal)
    {
        if (m_TargetDistanceVal == TargetDistanceVal)
            return;

        m_TargetDistanceVal = TargetDistanceVal;
        emit TargetDistanceValChanged(m_TargetDistanceVal);
    }

    void setTargetCordsVal(QString TargetCordsVal)
    {
        if (m_TargetCordsVal == TargetCordsVal)
            return;

        m_TargetCordsVal = TargetCordsVal;
        emit TargetCordsValChanged(m_TargetCordsVal);
    }

    void setPathPassedVal(QString PathPassedVal)
    {
        if (m_PathPassedVal == PathPassedVal)
            return;

        m_PathPassedVal = PathPassedVal;
        emit PathPassedValChanged(m_PathPassedVal);
    }

    void setStartRouteButtonEnabled(bool StartRouteButtonEnabled)
    {
        if (m_StartRouteButtonEnabled == StartRouteButtonEnabled)
            return;

        m_StartRouteButtonEnabled = StartRouteButtonEnabled;
        emit StartRouteButtonEnabledChanged(m_StartRouteButtonEnabled);
    }

signals:
    // QML Connection signals autogenerated by Qt (Related to Q_PROPERTY)
    void LeftGaugeValChanged(QString LeftGaugeVal);
    void LeftArcChanged(int LeftArc);
    void RightGaugeValChanged(QString RightGaugeVal);
    void RightArcChanged(int RightArc);
    void RightGaugeOpacityChanged(double RightGaugeOpacity);
    void LeftGaugeOpacityChanged(double LeftGaugeOpacity);
    void LeftGaugeColorChanged(QString LeftGaugeColor);
    void RightGaugeColorChanged(QString RightGaugeColor);
    void ButtonsEnabledChanged(bool ButtonsEnabled);
    void SafetyTriggerOpacityChanged(int SafetyTriggerOpacity);
    void IsIsSafetyEngaged(bool IsSafetyEngaged);
    void SafetyImageUrlChanged(QUrl SafetyImageUrl);
    void PanelSafetyValChanged(QString PanelSafetyVal);
    void PanelConnectValChanged(QString PanelConnectVal);
    void PanelLongValChanged(QString PanelLongVal);
    void PanelLatValChanged(QString PanelLatVal);
    void PanelHeadingValChanged(QString PanelHeadingVal);
    void PanelBatteryValChanged(QString PanelBatteryVal);
    void PanelSpeedValChanged(QString PanelSpeedVal);
    void mainPanelOpacityChanged(int mainPanelOpacity);
    void emptyPanelOpacityChanged(int emptyPanelOpacity);
    void compGraphicOpacityChanged(int compGraphicOpacity);
    void MainPanelOpacityChanged(int MainPanelOpacity);
    void EmptyPanelOpacityChanged(int EmptyPanelOpacity);
    void CompGraphicOpacityChanged(int CompGraphicOpacity);
    void CompRotationValChanged(int CompRotationVal);
    void SliderEnabledChanged(bool SliderEnabled);
    void MapEnabledChanged(bool MapEnabled);
    void GeoPathChanged(QGeoPath GeoPath);
    void MapPanelOpacityChanged(int MapPanelOpacity);
    void PanelPitchValChanged(QString PanelPitchVal);
    void PanelRollValChanged(QString PanelRollVal);
    void PanelSatCountChanged(QString PanelSatCount);
    void PanelSatValChanged(QString PanelSatVal);  
    void GPSLatValChanged(float GPSLatVal);
    void GPSLongValChanged(float GPSLongVal);
    void ButtonsOpacityChanged(int ButtonsOpacity);
    void DrivingInfoOpacityChanged(int DrivingInfoOpacity);

    void RelativeBearingValChanged(QString RelativeBearingVal);

    void TargetBearingValChanged(QString TargetBearingVal);

    void TargetDistanceValChanged(QString TargetDistanceVal);

    void TargetCordsValChanged(QString TargetCordsVal);

    void PathPassedValChanged(QString PathPassedVal);

    void StartRouteButtonEnabledChanged(bool StartRouteButtonEnabled);

private:
    QNode qnode; // the QNode object used to communicated with ROS

    int speedControlVal = 200; // Qt variable used to control published speed (0 to 400)

    QGeoPath selected_path; // Qt variable for GUI updating of path polyline

    std::string selected_path_str = "[]"; // path used by robot for path driving

    // auto generated variables by Qt (Related to Q_PROPERTY)
    QString m_LeftGaugeVal;
    int m_LeftArc;
    QString m_RightGaugeVal;
    int m_RightArc;
    int m_RightGaugeOpacity;
    int m_LeftGaugeOpacity;
    QString m_LeftGaugeColor;
    QString m_RightGaugeColor;
    bool m_ButtonsEnabled;
    int m_SafetyTriggerOpacity;
    bool m_IsSafetyEngaged;
    QString m_SafetyImageUrl;

    QString m_PanelSafetyVal;
    QString m_PanelConnectVal;
    QString m_PanelLongVal;
    QString m_PanelLatVal;
    QString m_PanelHeadingVal;
    QString m_PanelBatteryVal;
    QString m_PanelSpeedVal;

    int m_MainPanelOpacity;
    int m_EmptyPanelOpacity;
    int m_CompGraphicOpacity;
    int m_CompRotationVal;
    bool m_SliderEnabled;
    bool m_MapEnabled;
    QGeoPath m_GeoPath;
    int m_MapPanelOpacity;
    QString m_PanelPitchVal;
    QString m_PanelRollVal;
    QString m_PanelSatVal;
    float m_GPSLatVal;
    float m_GPSLongVal;
    int m_ButtonsOpacity;
    int m_DrivingInfoOpacity;
    QString m_RelativeBearingVal;
    QString m_TargetBearingVal;
    QString m_TargetDistanceVal;
    QString m_TargetCordsVal;
    QString m_PathPassedVal;
    bool m_StartRouteButtonEnabled;
};

#endif // BACKEND_H
