import QtQuick 2.0
import HelperWidgets 2.0
import QtQuick.Layouts 1.0


Column {
    anchors.left: parent.left
    anchors.right: parent.right

    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: qsTr("Fill Color")

        ColorEditor {
            caption: qsTr("Fill Color")
            backendValue: backendValues.fillColor
            supportGradient: true
            shapeGradients: true
        }


    }

    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: qsTr("Stroke Color")

        ColorEditor {
            caption: qsTr("Stroke Color")
            backendValue: backendValues.strokeColor
            supportGradient: false
        }
    }


    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: qsTr("Corner Radiuses")

        SectionLayout {
            rows: 2
            Label {
                text: qsTr("Radius")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.radius
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 200
                    stepSize: 1
                }
                ExpandingSpacer {

                }
            }
            Label {
                text: qsTr("Top Left Radius")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.topLeftRadius
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 200
                    stepSize: 1
                }
                ExpandingSpacer {

                }
            }

            Label {
                text: qsTr("Top Right Radius")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.topRightRadius
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 200
                    stepSize: 1
                }
                ExpandingSpacer {

                }
            }

            Label {
                text: qsTr("Bottom Right Radius")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.bottomRightRadius
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 200
                    stepSize: 1
                }
                ExpandingSpacer {

                }
            }

            Label {
                text: qsTr("Bottom Left Radius")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.bottomLeftRadius
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 200
                    stepSize: 1
                }
                ExpandingSpacer {

                }
            }
        }
    }

    BevelSection {
    }

    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: "Stroke Details"

        SectionLayout {
            rows: 2
            Label {
                text: qsTr("Stroke Width")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.strokeWidth
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 200
                    stepSize: 1
                }
                ExpandingSpacer {

                }
            }

            Label {
                text: qsTr("Border Mode")
            }

            SecondColumnLayout {
                BorderModeComboBox {
                }
            }


            Label {
                text: qsTr("Stroke Style")
            }

            SecondColumnLayout {
                ComboBox {
                    id: strokeStyle
                    model: ["None", "Solid", "Dash", "Dot", "Dash Dot", "Dash Dot Dot"]
                    backendValue: backendValues.strokeStyle
                    Layout.fillWidth: true
                    useInteger: true
                }
            }

            Label {
                Layout.alignment: Qt.AlignTop | Qt.AlignLeft
                text: qsTr("Dash Pattern")
            }

            DashPatternEditor {
                enableEditors: strokeStyle.currentIndex === 2
            }

            Label {
                text: qsTr("Joint Style")
            }

            SecondColumnLayout {
                ComboBox {
                    model: ["Miter Join", "Bevel Join", "Round Join"]
                    backendValue: backendValues.joinStyle
                    Layout.fillWidth: true
                    useInteger: true
                }
            }
            Label {
                text: qsTr("Dash Offset")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.dashOffset
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 1000
                    stepSize: 1
                }
            }

            Label {
                text: qsTr("Anti Aliasing")
            }
            SecondColumnLayout {
                CheckBox {
                    backendValue: backendValues.antialiasing
                    text: backendValues.antialiasing.value
                }
                ExpandingSpacer {

                }
            }
        }
    }
}
