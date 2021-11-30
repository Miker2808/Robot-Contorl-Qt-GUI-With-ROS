# Robot Control QT GUI With ROS
## Introduction
This is an **open source** GUI developed for use on a track based robots which use dual mode controllers that can control 
either left or right side of the wheels/tracks.
The GUI is developed using Qt on C++ with QtQuick. The GUI is integrated with ROS and is directly subscribing and publishing to topics.

## Features

### Dynamic safety display
The GUI allows dynamic animated shifting between safety modes, the GUI subscribes to a safety topic and changes visually to depending on the safety mode.
When safety is engaged the buttons are disabled and will fade away. Additionally the GUI contains a safety button to manually enable and disable the safety mode

![Safety Demo](https://i.imgur.com/NIrsiO5.gif)

### Buttons for 2D motion control
The GUI includes a set of buttons to control the robot using a mouse. The features allows to take the full control of the GUI using only a mouse.

![Buttons Demo](https://s8.gifyu.com/images/buttons_demo.gif)

### 2 Dynamic Gauges for motor power visualization
The GUI consists of two gauges with value animation and a unique look for reverse driving
The gauges subscribe directly to '/robot/motor_left' and '/robot/motor_right' topics so you will be able to monitor the motion also with external controllers.

![Gauge Demo](https://s2.gifyu.com/images/gauges_demo.gif)

### Main panel with 3 different view modes
#### The Main Panel for data visualization and speed control
The main panel contains all the data necessary to know what's going on with the robot in real time, all the data comes straight
from the ROS topics and updates automatically with any new subscribed data, meaning you can change the frequency of updates depending
on how you optimize your ROS project. additionally, the panel comes with a slider to change the published speed, from 0 to 400
this project is open source, so you can tweak the gui however you want!

![Main Display Demo](https://s2.gifyu.com/images/main_dispaly_demo.gif)

#### Compass Panel as an additionaly heading visualization tool
The Compass Panel allows you to see your heading and it comes with a dynamic compass which rotates
to your heading, 360 degrees!

![Compass Display Demo](https://s8.gifyu.com/images/heading_display_demo.gif)

#### Map Panel with a satellite imagery and robot location
The Map Panel allows you to see your robot real time with satellite imagery depending on the GPS position published to the "/robot/gps" topic.
The robot will additionally rotate depending on your heading so you know where your robot is relative to the maps north.
Furthermore, the Map Panel allows the user to generate a coordinate array simply by clicking at the desired points on the map, after finishing with your path
you simply click on "Start Route" and it will instantly publish a Pythonic list of coordinates as a String!
If you are lost, you can always reset the map to your robot using "Reset Pos" button.

![Map Dispaly Demo](https://s8.gifyu.com/images/map_display_demo.gif)

## Requirements
To successfuly compile the project you need to set few things.

* Ubuntu specified in the branch name
* ROS installed in default directory of the version specified in the branch name
* Qt 5.15.0 up to 5.15.2. (Qt 5.15.12 is recommended)

**Qt 6.0.0 is not supported due to many libraries that are excluded from this version.**

## Installation/Compilation
For successful compilation, it is recommended to do few actions before building the project.
1. Open the project using **File** -> **Open Project** -> {**_navigate the .pro file_**}
2. Clean project using  **Build** -> **Clean All Projects for All Configurations**
3. run qmake using **Build** -> **Run qmake**
4. Finally, build the projects. 

If everything is done correctly the project should buld right away without errors. Please note that the GUI is using qmake for compilation
and its using the ROS and Qt dynamic libraries for compilation, so you cannot just ship the build file without the libraries.
You can ease with shipment by statically linking the Qt Libraries if you wish to.

**All the resources and art were designed and made by me**
