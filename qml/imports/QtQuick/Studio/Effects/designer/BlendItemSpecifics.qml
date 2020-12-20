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
        caption: qsTr("Blend")

        SectionLayout {
            rows: 2
            Label {
                text: qsTr("Mode")
                toolTip: qsTr("This property defines the mode which is used when foregroundSource is blended over source.")
            }
            SecondColumnLayout {
                ComboBox {
                    id: blendMode

                    backendValue: backendValues.mode
                    Layout.fillWidth: true

                    useInteger: true
                    manualMapping: true

                    model: ["normal", "addition", "average", "color", "colorBurn", "colorDodge", "darken", "darkerColor", "difference", "divide", "exclusion", "hardLight", "hue", "lighten", "lighterColor", "lightness", "multiply", "negation", "saturation", "screen", "subtract", "softLight"]

                    property bool block: false

                    onValueFromBackendChanged: blendMode.fromBackendToFrontend()

                    onCurrentTextChanged: {
                        if (!__isCompleted)
                            return

                        if (block)
                            return

                        backendValues.mode.value = blendMode.model[blendMode.currentIndex]
                    }

                    Connections {
                        target: modelNodeBackend
                        onSelectionChanged: blendMode.fromBackendToFrontend()
                    }

                    function fromBackendToFrontend()
                    {
                        if (!__isCompleted)
                            return

                        block = true

                        currentIndex = blendMode.model.indexOf(backendValues.mode.value)

                        block = false
                    }
                }
                ExpandingSpacer {
                }
            }
        }
    }

    Section {
        anchors.left: parent.left
        anchors.right: parent.right
        caption: qsTr("Caching")

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
            }
        }
    }
}
