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
        caption: qsTr("Gaussian Blur")

        SectionLayout {
            rows: 2
            Label {
                text: qsTr("Radius")
                toolTip: qsTr("This property defines the distance of the neighboring pixels which affect the blurring of an individual pixel. A larger radius increases the blur effect.")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.radius
                    decimals: 1
                    minimumValue: 0
                    maximumValue: 100
                    stepSize: 1
                }
                ExpandingSpacer {
                }
            }

            Label {
                text: qsTr("Deviation")
                toolTip: qsTr("This property is a parameter to the gaussian function that is used when calculating neighboring pixel weights for the blurring.")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.deviation
                    decimals: 2
                    minimumValue: 0
                    maximumValue: 50
                    stepSize: 1
                }
                ExpandingSpacer {
                }
            }

            Label {
                text: qsTr("Samples")
                toolTip: qsTr("This property defines how many samples are taken per pixel when blur calculation is done. Larger value produces better quality, but is slower to render.")
            }
            SecondColumnLayout {
                SpinBox {
                    backendValue: backendValues.samples
                    decimals: 0
                    minimumValue: 0
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
                toolTip: qsTr("When set to true, the exterior of the item is padded with a transparent edge, making sampling outside the source texture use transparency instead of the edge pixels.")
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
