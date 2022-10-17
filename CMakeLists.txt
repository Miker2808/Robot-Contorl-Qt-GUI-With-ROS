cmake_minimum_required(VERSION 3.14)

project(ros2_robot_gui VERSION 0.1 LANGUAGES CXX)

# Fix for "Could NOT find Threads"
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
# end_of_fix

if(NOT DEFINED ${QT_DIR})
message(AUTHOR_WARNING "\n QT_DIR is not defined, meaning you attempt to run \"colcon build\" outside of QtCreator.\n Doing so will make \"CMAKE_PREFIX_PATH\" not have the path to Qt5Positioning
which is necessary for this project. It is recommened to build this project through QtCreator (See README for details) alternatively, make sure you set the correct path leading to the Qt5Positioning folder, for example \"/home/cooluser/Qt/5.15.2/gcc_64/lib/cmake\".\n Cmake will attempt to define \"CMAKE_PREFIX_PATH\" manually, but it may fail, refer to line 21 of CMakeLists.txt.\n\n")

# attempt custom QT_DIR
set(QT_DIR_CUSTOM "$ENV{HOME}/Qt") # Path leading to Qt binaries,
set(QT_VERSION_CUSTOM "5.15.2") # variable defining the version of Qt you are running.
# attempt to append CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "${QT_DIR_CUSTOM}/${QT_VERSION_CUSTOM}/gcc_64/lib/cmake")

else()
message(WARNING QT_DIR)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}/lib/cmake")
endif()


set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#list(APPEND QML_IMPORT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/qml/imports")
list(REMOVE_DUPLICATES QML_IMPORT_PATH)
set(QML_IMPORT_PATH ${QML_IMPORT_PATH}
    CACHE STRING "extra qml import paths"
    FORCE
)


#ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#ROS Messages
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Qt packages
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Quick Positioning REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick Positioning REQUIRED)

set(PROJECT_SOURCES
    main.cpp
    qml.qrc
    backend.cpp
    backend.h
    qnode.cpp
    qnode.h

)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(${PROJECT_NAME}
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(${PROJECT_NAME} SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(${PROJECT_NAME}
          ${PROJECT_SOURCES}
        )
    endif()
endif()


ament_target_dependencies(${PROJECT_NAME}
 rclcpp
 std_msgs
 sensor_msgs
 rmw_implementation
)

#target_compile_definitions(${PROJECT_NAME}
#   $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)

target_link_libraries(${PROJECT_NAME}
   Qt${QT_VERSION_MAJOR}::Core
   Qt${QT_VERSION_MAJOR}::Quick
   Qt${QT_VERSION_MAJOR}::Positioning
   )



set_target_properties(${PROJECT_NAME} PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_import_qml_plugins(${PROJECT_NAME})
    qt_finalize_executable(${PROJECT_NAME})
endif()

#Used for building a package, uncomment if wishing to build ros package
#ament_package()
