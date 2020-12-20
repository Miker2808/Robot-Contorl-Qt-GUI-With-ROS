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
import QtGraphicalEffects 1.0

Item {
    id: root

    default property alias contentStack: stack.children
    property alias radius: maskedBlur.radius
    property alias samples: maskedBlur.samples

    implicitWidth: Math.max(32, stack.implicitWidth)
    implicitHeight: Math.max(32, stack.implicitHeight)

    Item {
        z: -1
        id: stack
        implicitWidth: maskedBlur.source.width + maskedBlur.source.x
        implicitHeight: maskedBlur.source.height + maskedBlur.source.y
        visible: false
    }

    MaskedBlur {
        id: maskedBlur
        anchors.fill: parent
        source: root.background
        maskSource: root.foreground
        radius: 32
        samples: 16
    }

    property Item background
    property Item foreground

    Component.onCompleted: {
        root.background = stack.children[0]
        root.foreground = stack.children[1]
    }
}
