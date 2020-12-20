/****************************************************************************
**
** Copyright (C) 2019 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of Qt Quick Designer Components.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

import QtQuick 2.0
import HelperWidgets 2.0
import QtQuick.Layouts 1.0


Column {
    anchors.left: parent.left
    anchors.right: parent.right


    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: qsTr("Radial Blur")

        SectionLayout {
            rows: 2
            Label {
                text: qsTr("Angle")
                toolTip: qsTr("This property defines the direction for the blur and at the same time the level of blurring. The larger the angle, the more the result becomes blurred. The quality of the blur depends on samples property.")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.angle
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 360
                    stepSize: 1
                }
                ExpandingSpacer {
                }
            }

            Label {
                text: qsTr("Samples")
                toolTip: qsTr("This property defines how many samples are taken per pixel when blur calculation is done. Larger value produces better quality, but is slower to render. This property is not intended to be animated. Changing this property may cause the underlying OpenGL shaders to be recompiled.")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.samples
                    Layout.preferredWidth: 80
                    decimals: 0
                    minimumValue: 0
                    maximumValue: 200
                    stepSize: 1
                }
                ExpandingSpacer {
                }
            }
        }
    }

    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: qsTr("Offsets")

        SectionLayout {
            rows: 2
            Label {
                text: qsTr("Horizontal offset")
                toolTip: qsTr("These properties define the offset in pixels for the perceived center point of the rotation.")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.horizontalOffset
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: -1000
                    maximumValue: 1000
                    stepSize: 1
                }
                ExpandingSpacer {
                }
            }

            Label {
                text: qsTr("Vertical offset")
                toolTip: qsTr("These properties define the offset in pixels for the perceived center point of the rotation.")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.verticalOffset
                    Layout.preferredWidth: 80
                    decimals: 1
                    minimumValue: -1000
                    maximumValue: 1000
                    stepSize: 1
                }
                ExpandingSpacer {
                }
            }
        }
    }

    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: qsTr("Caching and Border")

        SectionLayout {
            rows: 2
            Label {
                text: qsTr("Cached")
                toolTip: qsTr("This property allows the effect output pixels to be cached in order to improve the rendering performance.")
            }
            SecondColumnLayout {
                CheckBox {
                    Layout.fillWidth: true
                    backendValue: backendValues.cached
                    text: backendValues.cached.valueToString
                }
                ExpandingSpacer {
                }
            }
            Label {
                text: qsTr("Transparent border")
                toolTip: qsTr("This property defines the blur behavior near the edges of the item, where the pixel blurring is affected by the pixels outside the source edges. If the property is set to true, the pixels outside the source are interpreted to be transparent, which is similar to OpenGL clamp-to-border extension. The blur is expanded slightly outside the effect item area.")
            }
            SecondColumnLayout {
                CheckBox {
                    Layout.fillWidth: true
                    backendValue: backendValues.transparentBorder
                    text: backendValues.transparentBorder.valueToString
                }
            }
        }
    }
}
