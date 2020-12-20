QT += quick
QT += positioning

CONFIG += c++11
CONFIG += qmltypes

QML_IMPORT_NAME = BackendConnection
QML_IMPORT_MAJOR_VERSION = 1

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        backend.cpp \
        main.cpp \
        qnode.cpp

RESOURCES += qml.qrc
RESOURCES += $$files(qml/*)

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH = qml/imports

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    backend.h \
    qnode.h


# all ROS dependencies (dynamically linked to default ros installation location)
#unix:!macx: LIBS += -L$$PWD/../../../../opt/ros/melodic/lib
unix:!macx: LIBS += -L"/opt/ros/melodic/lib"
LIBS += -L"/opt/ros/melodic/lib" -lroscpp
LIBS += -L"/opt/ros/melodic/lib" -lrosconsole
LIBS += -L"/opt/ros/melodic/lib" -lroscpp_serialization
LIBS += -L"/opt/ros/melodic/lib" -lxmlrpcpp
LIBS += -L"/opt/ros/melodic/lib" -lrostime
LIBS += -L"/opt/ros/melodic/lib" -lcpp_common
LIBS += -L"/opt/ros/melodic/lib" -lrosconsole_log4cxx
LIBS += -L"/opt/ros/melodic/lib" -lrosconsole_backend_interface


#INCLUDEPATH += $$PWD/../../../../opt/ros/melodic/include # relative paths
#DEPENDPATH += $$PWD/../../../../opt/ros/melodic/include

INCLUDEPATH += "/opt/ros/melodic/include"
DEPENDPATH += "/opt/ros/melodic/include"


DESTDIR = $$PWD/build # compiled application will be written here!

# include ros using pkgconfig (not recommended)
#unix: CONFIG += link_pkgconfig
#unix: PKGCONFIG += roscpp

#unix: CONFIG += link_pkgconfig
#unix: PKGCONFIG += sensor_msgs

#unix: CONFIG += link_pkgconfig
#unix: PKGCONFIG += std_msgs
