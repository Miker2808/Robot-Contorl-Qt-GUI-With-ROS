import QtQuick 2.0
import HelperWidgets 2.0
import QtQuick.Layouts 1.0


Section {
    anchors.left: parent.left
    anchors.right: parent.right
    caption: qsTr("Corner Bevel")

    SectionLayout {
        rows: 2

        Label {
            text: qsTr("Bevel")
        }
        SecondColumnLayout {
            CheckBox {
                backendValue: backendValues.bevel
                text: backendValues.bevel.value
            }
            ExpandingSpacer {

            }
        }

        Label {
            text: qsTr("Top Left Bevel")
        }
        SecondColumnLayout {
            CheckBox {
                backendValue: backendValues.topLeftBevel
                text: backendValues.topLeftBevel.value
            }
            ExpandingSpacer {

            }
        }

        Label {
            text: qsTr("Top Right Bevel")
        }
        SecondColumnLayout {
            CheckBox {
                backendValue: backendValues.topRightBevel
                text: backendValues.topRightBevel.value
            }
            ExpandingSpacer {

            }
        }

        Label {
            text: qsTr("Bottom Right Bevel")
        }
        SecondColumnLayout {
            CheckBox {
                backendValue: backendValues.bottomRightBevel
                text: backendValues.bottomRightBevel.value
            }
            ExpandingSpacer {

            }
        }

        Label {
            text: qsTr("Bottom Left Bevel")
        }
        SecondColumnLayout {
            CheckBox {
                backendValue: backendValues.bottomLeftBevel
                text: backendValues.bottomLeftBevel.value
            }
            ExpandingSpacer {

            }
        }
    }
}
