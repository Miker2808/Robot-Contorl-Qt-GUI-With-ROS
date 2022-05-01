import QtQuick 2.12
import DesignerImports 1.0
import QtQuick.Studio.Components 1.0
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.11
import QtQuick.Studio.Effects 1.0
import QtQuick.Timeline 1.0
import QtQuick.Controls.Styles 1.4
import QtLocation 5.8
import QtPositioning 5.12



//import BackendConnection 1.0
Rectangle {

    id: root
    width: Constants.width
    height: Constants.height
    color: "#2d2d2d"

    // up button signals
    signal upButtonPressed()
    signal upButtonReleased()
    // down button signals
    signal downButtonPressed()
    signal downButtonReleased()
    // left button signals
    signal leftButtonPressed()
    signal leftButtonReleased()
    // right button signals
    signal rightButtonPressed()
    signal rightButtonReleased()
    // safety button signals
    signal safetyButtonPressed()
    signal safetyButtonReleased()
    // panel button signals
    signal mainPanelButtonClicked()
    signal mapPanelButtonClicked()
    signal compPanelButtonClicked()
    // slider value changed signal
    signal speedSliderValueChanged(value: int)
    // map singals
    signal coordinateValueClicked(latitude: string, longitude: string)
    signal startRouteButtonClicked()
    signal cancelRouteButtonClicked()
    signal zeroPosButtonClicked()




    Image {
        property int ui_opacity: backend.SafetyTriggerOpacity  //opacity comes as int from 0 to 100 from backend
        id: uiOverlayBlue
        x: -78
        y: 17
        opacity: 1
        anchors.fill: parent
        source: "UI-Overlay-blue.png"
        anchors.bottomMargin: 0
        mirror: false
        anchors.leftMargin: 0
        anchors.rightMargin: 0
        fillMode: Image.PreserveAspectFit
        anchors.topMargin: 0

        Behavior on ui_opacity {
            NumberAnimation{
                duration: 1000
            }
        }
    }

    Image {
        property int ui_opacity: 100 - backend.SafetyTriggerOpacity
        id: uIOverlayRed
        x: -78
        y: 17
        anchors.fill: parent
        source: "UI-Overlay-red.png"
        antialiasing: false
        anchors.rightMargin: 0
        anchors.bottomMargin: 0
        anchors.leftMargin: 0
        anchors.topMargin: 0
        mirror: false
        fillMode: Image.PreserveAspectFit
        opacity: ui_opacity / 100 //opacity comes as int from 0 to 100 from backend

        Behavior on ui_opacity {
            NumberAnimation{
                duration: 1000
            }
        }
    }


    Item {
        id: rightMotorGauge
        x: 1378
        y: 77
        width: 528
        height: 530



        Image {
            property int gauge_opacity: backend.SafetyTriggerOpacity
            id: motorRightGaugeImageBlue
            x: -45
            y: 16
            source: "powerClock-blue.png"
            mirror: false
            fillMode: Image.PreserveAspectFit
            opacity: gauge_opacity / 100

            Behavior on gauge_opacity {
                NumberAnimation{
                    duration: 1000
                }
            }
        }

        Image {
            property int gauge_opacity: 100 - backend.SafetyTriggerOpacity
            id: motorRightGaugeImageRed
            x: -45
            y: 16
            source: "powerClock-red.png"
            mirror: false
            fillMode: Image.PreserveAspectFit
            opacity: gauge_opacity / 100

            Behavior on gauge_opacity {
                NumberAnimation{
                    duration: 1000
                }
            }
        }


        Text {
            property int text_opacity: 100 - backend.RightGaugeOpacity
            id: rightGaugeReverse
            x: 178
            y: 215
            width: 156
            height: 36
            opacity: text_opacity / 100
            color: "#c2c2c2"
            text: "REVERSE"
            font.letterSpacing: 0.7
            font.pixelSize: 28
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.italic: false
            font.weight: Font.Thin
            font.underline: false
            font.family: "Verdana"
            minimumPixelSize: 18
            renderType: Text.QtRendering
            styleColor: "#ffffff"
            font.bold: false
            style: Text.Raised

            Behavior on text_opacity {
                NumberAnimation{
                    duration: 250
                }
            }
        }

        Text {
            property int text_val: backend.RightGaugeVal
            property int text_opacity: backend.RightGaugeOpacity
            id: rightGaugeValue
            x: 176
            y: 213
            width: 156
            height: 36
            color: "#c2c2c2"
            text: text_val
            font.letterSpacing: 0.7
            font.pixelSize: 28
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            styleColor: "#ffffff"
            font.underline: false
            minimumPixelSize: 18
            font.bold: false
            renderType: Text.QtRendering
            font.weight: Font.Thin
            font.family: "Verdana"
            font.italic: false
            style: Text.Raised
            opacity: text_opacity / 100

            Behavior on text_val{
                NumberAnimation{
                    duration: 500
                    easing.bezierCurve: [0.445, 0.05, 0.55, 0.95, 1, 1]
                }
            }

            Behavior on text_opacity {
                NumberAnimation{
                    duration: 250
                }
            }



        }

        GroupItem {
            x: 126
            y: 22
            Text {
                id: counterFour1
                x: 8
                y: 0
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "4"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                minimumPixelSize: 40
                textFormat: Text.PlainText
                minimumPointSize: 40
            }

            Text {
                id: counterThree1
                x: -124
                y: 152
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "3"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                minimumPixelSize: 40
                textFormat: Text.PlainText
                minimumPointSize: 40
            }

            Text {
                id: counterTwo1
                x: -109
                y: 341
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "2"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                minimumPixelSize: 40
                textFormat: Text.PlainText
                minimumPointSize: 40
            }

            Text {
                id: counterOne1
                x: 34
                y: 464
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "1"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                minimumPixelSize: 40
                textFormat: Text.PlainText
                minimumPointSize: 40
            }

            Text {
                id: counterZero1
                x: 231
                y: 447
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "0"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                minimumPixelSize: 40
                textFormat: Text.PlainText
                minimumPointSize: 40
            }
        }

        Text {
            id: motorRightGaugeName
            x: 177
            y: 264
            width: 156
            height: 25
            color: "#dddddd"
            text: qsTr("right motor power")
            font.letterSpacing: 0.4
            font.pixelSize: 14
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            styleColor: "#00000000"
            font.family: "Verdana"
            renderType: Text.QtRendering
            font.weight: Font.Thin
            style: Text.Raised
            font.bold: false
        }

        Text {
            id: x100Right
            x: 177
            y: 295
            width: 156
            height: 46
            color: "#f6f6f7"
            text: qsTr("X  100")
            font.letterSpacing: 0.7
            font.pixelSize: 22
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            styleColor: "#fdfdfd"
            font.underline: false
            font.italic: false
            minimumPixelSize: 16
            font.family: "Verdana"
            renderType: Text.QtRendering
            font.weight: Font.Thin
            style: Text.Raised
            font.bold: false
        }




        ArcItem {
            id: rightArc
            x: 69
            y: 79
            width: 373
            height: 371
            end: backend.RightArc
            rotation: 153
            dashPattern: [0.2,0.1,0,0]
            strokeColor: backend.RightGaugeColor
            begin: 0
            antialiasing: true
            roundBegin: false
            capStyle: 0
            arcWidth: 50
            radiusOuterAdjust: 0
            dashOffset: 0
            fillColor: "#00000000"
            strokeWidth: 45
            radiusInnerAdjust: 0
            strokeStyle: 1
            round: false
            arcWidthEnd: 50
            outlineArc: false


            Behavior on strokeColor {
                ColorAnimation{
                    duration: 250
                }
            }

            Behavior on end{
                NumberAnimation{
                    duration: 500
                    easing.bezierCurve: [0.445, 0.05, 0.55, 0.95, 1, 1]
                }
            }


        }
    }

    Item {

        id: leftMotorGauge
        x: 53
        y: 77
        width: 528
        height: 530
        opacity: 1

        Image {
            property int gauge_opacity: backend.SafetyTriggerOpacity
            id: motorLeftgaugeImageBlue
            x: -45
            y: 16
            source: "powerClock-blue.png"
            mirror: true
            fillMode: Image.PreserveAspectFit
            opacity: gauge_opacity / 100

            Behavior on gauge_opacity {
                NumberAnimation{
                    duration: 1000
                }
            }
        }

        Image {
            property int gauge_opacity: 100 - backend.SafetyTriggerOpacity
            id: motorLeftgaugeImageRed
            x: -45
            y: 16
            source: "powerClock-red.png"
            mirror: true
            fillMode: Image.PreserveAspectFit
            opacity: gauge_opacity / 100

            Behavior on gauge_opacity {
                NumberAnimation{
                    duration: 1000
                }
            }
        }

        Label {
            property int text_opacity: 100 - backend.LeftGaugeOpacity
            id: leftGaugeReverse
            x: 176
            y: 214
            width: 156
            height: 35
            opacity: text_opacity / 100
            color: "#c2c2c2"
            text: "REVERSE"
            font.letterSpacing: 0.7
            font.pixelSize: 28
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.italic: false
            font.weight: Font.Thin
            font.underline: false
            clip: false
            font.family: "Verdana"
            minimumPixelSize: 18
            renderType: Text.QtRendering
            styleColor: "#ffffff"
            font.bold: false
            style: Text.Raised

            Behavior on text_opacity {
                NumberAnimation{
                    duration: 250
                }
            }
        }

        Label {
            property int text_val: backend.LeftGaugeVal
            property int text_opacity: backend.LeftGaugeOpacity
            id: leftGaugeValue
            x: 177
            y: 215
            width: 156
            height: 35
            opacity: text_opacity / 100
            color: "#c2c2c2"
            text: text_val
            font.letterSpacing: 0.7
            font.pixelSize: 28
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            clip: false
            styleColor: "#ffffff"
            font.underline: false
            minimumPixelSize: 18
            font.bold: false
            renderType: Text.QtRendering
            font.weight: Font.Thin
            font.family: "Verdana"
            font.italic: false
            style: Text.Raised

            Behavior on text_val{
                NumberAnimation{
                    duration: 500
                    easing.bezierCurve: [0.445, 0.05, 0.55, 0.95, 1, 1]
                }
            }

            Behavior on text_opacity {
                NumberAnimation{
                    duration: 250

                }
            }
        }

        Text {
            id: x100Left
            x: 177
            y: 295
            width: 156
            height: 46
            color: "#dddddd"
            text: qsTr("X  100")
            font.letterSpacing: 0.7
            font.pixelSize: 22
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.underline: false
            font.italic: false
            styleColor: "#ffffff"
            minimumPixelSize: 16
            font.family: "Verdana"
            renderType: Text.QtRendering
            font.weight: Font.Thin
            style: Text.Raised
            font.bold: false
        }

        Text {
            id: motorLeftGaugeName
            x: 177
            y: 264
            width: 156
            height: 25
            color: "#dddddd"
            text: qsTr("left motor power")
            font.letterSpacing: 0.4
            font.pixelSize: 14
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            style: Text.Raised
            font.weight: Font.Thin
            font.bold: false
            renderType: Text.QtRendering
            font.family: "Verdana"
        }


        GroupItem {
            x: 126
            y: 22

            Text {
                id: counterFour
                x: 219
                y: 0
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "4"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                textFormat: Text.PlainText
                minimumPointSize: 40
                minimumPixelSize: 40
            }

            Text {
                id: counterThree
                x: 352
                y: 148
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "3"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                textFormat: Text.PlainText
                minimumPointSize: 40
                minimumPixelSize: 40
            }

            Text {
                id: counterTwo
                x: 343
                y: 344
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "2"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                textFormat: Text.PlainText
                minimumPointSize: 40
                minimumPixelSize: 40
            }

            Text {
                id: counterOne
                x: 183
                y: 465
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "1"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                textFormat: Text.PlainText
                minimumPointSize: 40
                minimumPixelSize: 40
            }

            Text {
                id: counterZero
                x: 0
                y: 447
                width: 28
                height: 26
                color: "#c2c2c2"
                text: "0"
                font.pixelSize: 23
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                textFormat: Text.PlainText
                minimumPixelSize: 40
                minimumPointSize: 40
            }
        }


        ArcItem {
            id: leftArc
            x: 75
            y: 80
            width: 367
            height: 370
            dashPattern: [0.2,0.1,0,0]
            begin: backend.LeftArc
            radiusOuterAdjust: 0
            round: false
            strokeColor: backend.LeftGaugeColor
            radiusInnerAdjust: 0
            roundBegin: false
            end: 210
            arcWidth: 50
            capStyle: 0
            arcWidthEnd: 50
            antialiasing: true
            dashOffset: 0
            strokeStyle: 1
            outlineArc: false
            strokeWidth: 45
            fillColor: "#00000000"

            Behavior on begin{
                NumberAnimation{
                    duration: 500
                    easing.bezierCurve: [0.445, 0.05, 0.55, 0.95, 1, 1]
                }
            }
            Behavior on strokeColor {
                ColorAnimation{
                    duration: 250
                }
            }

        }
    }



    Image {
        property int button_opacity: backend.ButtonsOpacity
        property bool button_enabled: backend.ButtonsEnabled
        id: buttonright
        x: 1056
        y: 725
        width: 150
        height: 150
        opacity: button_opacity / 100
        visible: true
        source: "button-blue.png"
        antialiasing: true
        rotation: 90
        fillMode: Image.PreserveAspectFit

        MouseArea{
            id: mouseAreaRightB
            anchors.fill: buttonright
            enabled: buttonright.button_enabled
            onPressed: {
                buttonright.source = "button-blue-pressed.png"
                root.rightButtonPressed()
            }
            onReleased: {
                buttonright.source = "button-blue.png"
                root.rightButtonReleased()

            }
        }

        Behavior on button_opacity {
            NumberAnimation{
                duration: 1000
            }
        }
    }

    Image {
        property int button_opacity: backend.ButtonsOpacity
        property bool button_enabled: backend.ButtonsEnabled
        id: buttonleft
        x: 710
        y: 725
        width: 150
        height: 150
        source: "button-blue.png"
        antialiasing: true
        rotation: 270
        fillMode: Image.PreserveAspectFit
        opacity: button_opacity / 100

        MouseArea{
            id: mouseAreaLeftB
            anchors.fill: buttonleft
            enabled: buttonleft.button_enabled
            onPressed: {
                buttonleft.source = "button-blue-pressed.png"
                root.leftButtonPressed()
            }
            onReleased: {
                buttonleft.source = "button-blue.png"
                root.leftButtonReleased()
            }
        }

        Behavior on button_opacity {
            NumberAnimation{
                duration: 1000
            }
        }
    }

    Image {
        property int button_opacity: backend.ButtonsOpacity
        property bool button_enabled: backend.ButtonsEnabled
        id: buttondown
        x: 885
        y: 897
        width: 150
        height: 150
        source: "button-blue.png"
        antialiasing: true
        rotation: 180
        fillMode: Image.PreserveAspectFit
        opacity: button_opacity / 100

        MouseArea{
            id: mouseAreaDownB
            anchors.fill: buttondown
            enabled: buttondown.button_enabled
            onPressed: {
                buttondown.source = "button-blue-pressed.png"
                root.downButtonPressed()
            }
            onReleased: {
                buttondown.source = "button-blue.png"
                root.downButtonReleased()
            }
        }

        Behavior on button_opacity {
            NumberAnimation{
                duration: 1000
            }
        }
    }

    Image {
        property int button_opacity: backend.ButtonsOpacity
        property bool button_enabled: backend.ButtonsEnabled
        id: buttonup
        x: 885
        y: 557
        width: 150
        height: 150
        source: "button-blue.png"
        antialiasing: true
        fillMode: Image.PreserveAspectFit
        opacity: button_opacity / 100

        MouseArea {
            id: mouseAreaUpB
            anchors.fill: buttonup
            enabled: buttonup.button_enabled
            onPressed: {
                buttonup.source = "button-blue-pressed.png"
                root.upButtonPressed()
            }
            onReleased: {
                buttonup.source = "button-blue.png"
                root.upButtonReleased()
            }
        }

        Behavior on button_opacity {
            NumberAnimation{
                duration: 1000
            }
        }
    }

    Image {
        property string safety_image: backend.SafetyImageUrl
        id: safetybutton
        x: 885
        y: 725
        width: 150
        source: backend.SafetyImageUrl
        fillMode: Image.PreserveAspectFit

        MouseArea {
            id: mouseAreaSafetyB
            anchors.fill: safetybutton
            enabled: true

            onPressed:{
                root.safetyButtonPressed()
                safetybutton.source = backend.SafetyImageUrl;

            }
            onReleased: {
                root.safetyButtonReleased()
                safetybutton.source = backend.SafetyImageUrl;

            }
        }
    }






    Item {
        id: controlPanel
        x: 682
        y: 18
        width: 17
        height: 11






        MouseArea {
            id: mainButton
            x: 383
            y: 8
            width: 183
            height: 39

            onClicked: {
                root.mainPanelButtonClicked()
            }
        }

        MouseArea {
            id: compButton
            x: 194
            y: 8
            width: 183
            height: 39

            onClicked: {
                root.compPanelButtonClicked()
            }
        }

        MouseArea {
            id: mapButton
            x: 5
            y: 8
            width: 183
            height: 39

            onClicked: {
                root.mapPanelButtonClicked()
            }
        }

        Image {
            property int speed_value: speedSlider.value
            id: controlPanelMain
            x: -13
            y: -10
            opacity: backend.MainPanelOpacity / 100
            source: "ControlPanelMain.png"
            fillMode: Image.PreserveAspectFit

            Text {
                id: safetyMainVal
                x: 201
                y: 65
                width: 124
                height: 22
                color: "#4df6ff"
                text: backend.PanelSafetyVal
                font.pixelSize: 18
                styleColor: "#0090e1"
                font.family: "Verdana"
            }

            Text {
                id: connMainVal
                x: 153
                y: 96
                width: 94
                height: 22
                color: "#4df6ff"
                text: backend.PanelConnectVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Text {
                id: latMainVal
                x: 130
                y: 126
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelLatVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Text {
                id: longMainVal
                x: 141
                y: 155
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelLongVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }



            Text {
                id: satMainVal
                x: 148
                y: 183
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelSatVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Text {
                id: headingMainVal
                x: 116
                y: 211
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelHeadingVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Text {
                id: pitchMainVal
                x: 90
                y: 243
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelPitchVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Text {
                id: rollMainVal
                x: 80
                y: 272
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelRollVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Text {
                id: batvoltMainVal
                x: 201
                y: 301
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelBatteryVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Text {
                id: speedMainVal
                x: 196
                y: 344
                width: 134
                height: 22
                color: "#4df6ff"
                text: backend.PanelSpeedVal
                font.pixelSize: 18
                font.family: "Verdana"
                styleColor: "#0090e1"
            }

            Slider {
                id: speedSlider
                x: 64
                y: 391
                width: 438
                height: 40
                to: 0
                from: 400
                value: 200
                enabled: backend.SliderEnabled

                onMoved: {
                    root.speedSliderValueChanged(speedSlider.value)
                }
            }



        }

        Item {
            property int compass_opacity: backend.CompGraphicOpacity / 100
            id: compassitem
            x: 52
            y: 37
            width: 200
            height: 200


            Image {
                id: controlPanelEmpty
                x: -65
                y: -47
                source: "controlpanel-empty.png"
                fillMode: Image.PreserveAspectFit
                opacity: backend.EmptyPanelOpacity / 100

            }

            Image {
                property int comp_heading: backend.CompRotationVal
                id: compassgraphic
                x: 0
                y: 0
                width: 466
                height: 445
                opacity: parent.compass_opacity
                source: "compassgraphic.png"
                rotation: comp_heading
                fillMode: Image.PreserveAspectFit

                Behavior on comp_heading {
                    NumberAnimation{
                        duration: 500
                        easing.bezierCurve: [0.445, 0.05, 0.55, 0.95, 1, 1]
                    }
                }

            }

            Text {
                property int heading_val: backend.PanelHeadingVal
                id: compassHeadingVal
                x: 158
                y: 196
                width: 151
                height: 54
                color: "#4df6ff"
                text: heading_val
                font.pixelSize: 30
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                font.underline: false
                font.bold: true
                styleColor: "#0090e1"
                font.family: "Verdana"
                opacity: parent.compass_opacity

                Behavior on heading_val {
                    NumberAnimation{
                        duration:500
                        easing.bezierCurve: [0.445, 0.05, 0.55, 0.95, 1, 1]
                    }
                }
            }

        }



        Image {
            property int map_opacity: backend.MapPanelOpacity
            property real gps_lat: backend.GPSLatVal
            property real gps_long: backend.GPSLongVal
            id: controlPanelMap
            x: -13
            y: -10
            opacity: map_opacity / 100
            source: "ControlPanelMap.png"
            fillMode: Image.PreserveAspectFit

            Map {
                property real latitude: 45.703547
                property real longitude: 21.302172
                property bool mapEnabled: backend.MapEnabled
                property MapCircle circle
                x: 31
                y: 69

                id: map
                visible: mapEnabled
                anchors.fill: parent
                minimumZoomLevel: 12
                maximumZoomLevel: 17.95
                copyrightsVisible: false
                anchors.bottomMargin: 68
                anchors.rightMargin: 21
                anchors.leftMargin: 18
                anchors.topMargin: 59
                activeMapType: map.supportedMapTypes[1]
                zoomLevel: 17
                center: QtPositioning.coordinate(45.703547, 21.302172) // init values
                plugin: Plugin {
                    name: 'esri';
                }

                MouseArea {
                    property var mouse_coordinate: map.toCoordinate(Qt.point(mouseX, mouseY))
                    property double cord_lat: mouse_coordinate.latitude
                    property double cord_long: mouse_coordinate.longitude
                    property string cord_lat_str: Number(cord_lat).toString()
                    property string cord_long_str: Number(cord_long).toString()
                    x: 0
                    y: 0
                    id: mouseArea
                    enabled: mapEnabled
                    anchors.fill: parent
                    anchors.bottomMargin: 0

                    onClicked: {
                        root.coordinateValueClicked(cord_lat_str, cord_long_str)
                        polyline.setPath(backend.GeoPath)

                    }
                }

                MapPolyline{
                    id: polyline
                    line.width: 3
                    line.color: 'cyan'

                }

                MapQuickItem {
                    property int comp_heading: backend.CompRotationVal
                    id: robotLocation
                    coordinate: QtPositioning.coordinate(controlPanelMap.gps_lat, controlPanelMap.gps_long)
                    anchorPoint.x: robotIcon.width/2
                    anchorPoint.y: robotIcon.height/2
                    rotation: comp_heading

                    sourceItem: Image{
                        id: robotIcon
                        width: 25
                        height: 25
                        source: "robot-icon.png"

                    }
                }
            }

            MouseArea {
                property bool enable_button: backend.CancelRouteButtonEnabled
                id: cancelRouteButton
                x: 18
                y: 418
                width: 183
                height: 38
                enabled: enable_button

                onClicked: {
                    root.cancelRouteButtonClicked()
                    polyline.setPath(backend.GeoPath)

                }
            }

            MouseArea {
                property bool enable_button: backend.ZeroPosButtonEnabled
                id: zeroPosButton
                x: 207
                y: 418
                width: 183
                height: 38
                enabled: enable_button

                onClicked: {
                    root.zeroPosButtonClicked()
                    map.center = QtPositioning.coordinate(controlPanelMap.gps_lat, controlPanelMap.gps_long)
                    map.zoomLevel = 17

                }
            }

            MouseArea {
                property bool enable_button: backend.StartRouteButtonEnabled
                id: startRouteButton
                x: 396
                y: 418
                width: 183
                height: 38
                enabled: enable_button

                onClicked: {
                    root.startRouteButtonClicked()
                }
            }

        }




    }

    Image {
        property int startScreenOpacity: 100  //opacity comes as int from 0 to 100 from backend
        property bool enabled_property: true
        enabled: enabled_property
        id: startScreenOverlay
        x: -78
        y: 17
        opacity: startScreenOpacity / 100
        visible: true
        anchors.fill: parent
        source: "StartScreen.png"
        anchors.bottomMargin: 0
        mirror: false
        anchors.leftMargin: 0
        anchors.rightMargin: 0
        fillMode: Image.PreserveAspectFit
        anchors.topMargin: 0
        SequentialAnimation on startScreenOpacity{
            PauseAnimation { duration: 2000 }
            NumberAnimation {
                duration: 1000
                to: 0
                //easing.bezierCurve: [0.445, 0.05, 0.55, 0.95, 1, 1]
                easing.bezierCurve: [1, 0, 1, -0.8]
                onRunningChanged: {
                    enabled_property = false
                }

            }
        }

    }

    Timeline {
        id: timeline
        animations: [
            TimelineAnimation {
                id: timelineAnimation
                running: false
                loops: 1
                duration: 1000
                to: 1000
                from: 0
            }
        ]
        enabled: true
        startFrame: 0
        endFrame: 400
    }






}






/*##^##
Designer {
    D{i:0;formeditorZoom:0.66}D{i:72}
}
##^##*/
