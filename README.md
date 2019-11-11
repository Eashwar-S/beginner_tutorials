# beginner_tutorials of ROS
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview:
This branch will provide an introduction for beginners who want to build client-server service to a node in ROS. It provides a step by step explaination of how to change talker node's frequency using client-server service

## Assumptions:

1. The user must have Ubuntu 16.04 LTS version installed.
2. The user must have ROS kinetic and catkin installed.

## Executables Added:
In CMakeLists.txt the following executables are added to execute cpp files in src folder:
```
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
add_executable(Client src/Client.cpp)
```

## Dependencies Added:
In CMakeLists.txt the following dependencies are added for executables:
```
add_dependencies(talker beginner_tutorials_generate_messages_cpp)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
add_dependencies(Client beginner_tutorials_generate_messages_cpp)
```

## Target Link Libraries Added:
In CMakeLists.txt the following target link libraries to link executables:
```
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(Client ${catkin_LIBRARIES})
```
## To integrate google test the following libraries and dependencies were added to CMakeLists.txt:
```
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(talkertest test/HW11_test.test test/talkertest.cpp)
  add_dependencies(talkertest talker beginner_tutorials_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
  target_link_libraries(talkertest ${catkin_LIBRARIES})
endif()
```
## Added C++11 Compile option:
This is done to take advantage of C++11 features
```
add_compile_options(-std=c++11)
```
## How to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Eashwar-S/beginner_tutorials.git
git checkout Week10_HW
cd ..
catkin_make
```
## How to run tests
```
cd catkin_ws
source devel/setup.bash
catkin_make run_tests
```
## To run tests using launch files
```
cd catkin_ws
source devel/setup.bash
rostest beginner_tutorials HW11_test.test
```

## How to run using terminal
First us to run ROS master
To do that open a new terminal and type
```
roscore
```

To run talker.cpp open a new terminal and type
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker <Publisher frequency>
```
Here Publisher frequency is a number greater than 0. For example
```
rosrun beginner_tutorials talker 10
```

To run listener.cpp open a new terminal and type
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

To run Client.cpp open a new terminal and type
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials Client
```
Please follow the same order.To stop the program press ctrl+C in each of the three terminals.

## How to run using launch files
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials Week10_HW.launch publisherFrequency:=<enter frequency>
```
## Sample Output:

In the terminal which is running talker we get output
```
[ INFO] [1572773551.337138854]: The publisher will publish messages at frequency 10 Hertz
[ INFO] [1572773551.337190996]: Hello World: Line : 0
[ INFO] [1572773551.437389878]: Hello World: Line : 1
[ INFO] [1572773551.537471104]: Hello World: Line : 2
[ INFO] [1572773551.637460487]: Hello World: Line : 3
[ INFO] [1572773551.737448533]: Hello World: Line : 4
[ INFO] [1572773551.837300288]: Hello World: Line : 5
[ INFO] [1572773551.937383684]: Hello World: Line : 6
[ INFO] [1572773552.037378873]: Hello World: Line : 7
[ WARN] [1572773552.037608768]: Modifying the Custom Message of the Publisher
[ INFO] [1572773552.037693599]: Modified Talker's message to :This is Eashwar
[DEBUG] [1572773552.037766938]: Is debug variable == 0
[ INFO] [1572773552.137369846]: Publisher is publishing the new Message
[ INFO] [1572773552.137489349]: This is Eashwar : Line : 8
[ INFO] [1572773552.237471625]: This is Eashwar : Line : 9
[ INFO] [1572773552.337365538]: This is Eashwar : Line : 10
[ INFO] [1572773552.437472127]: This is Eashwar : Line : 11
.
.
.
.
.


```
In the terminal which is running listener we get output
```
[ INFO] [1572773551.638199258]: I heard: Hello World: Line : 3
[ INFO] [1572773551.738033819]: I heard: Hello World: Line : 4
[ INFO] [1572773551.837575442]: I heard: Hello World: Line : 5
[ INFO] [1572773551.937876388]: I heard: Hello World: Line : 6
[ INFO] [1572773552.037985361]: I heard: Hello World: Line : 7
[ INFO] [1572773552.137965063]: I heard: This is Eashwar : Line : 8
[ INFO] [1572773552.237988411]: I heard: This is Eashwar : Line : 9
[ INFO] [1572773552.337947656]: I heard: This is Eashwar : Line : 10
[ INFO] [1572773552.438023245]: I heard: This is Eashwar : Line : 11
.
.
.
```
In the terminal which is running listener we get output
```
[ INFO] [1572773551.888259257]: Requesting server service to change publisher message
[ INFO] [1572773552.038114773]: Successfully changed Publisher's message to This is Eashwar from Hello World
```
## To verify TF frames
First use tf_echo.tf_echo reports the transform between any two frames broadcast over ROS.
Open new terminal and run talker 
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
Open another terminal and run tf_echo 
```
cd catkin_ws
source devel/setup.bash
rosrun tf tf_echo world talk
```
# Sample Output:
```
At time 1573480900.713
- Translation: [1.500, 3.000, 0.000]
- Rotation: in Quaternion [-0.339, 0.705, 0.050, 0.621]
            in RPY (radian) [-2.142, 1.142, -1.571]
            in RPY (degree) [-122.704, 65.408, -90.000]
At time 1573480901.413
- Translation: Quaternion [-0.339, 0.705, 0.050, 0.621]
            in RPY (radian) [-2.142, 1.142, -1.571]
            in RPY (degree) [-122.704, 65.408, -90.000]
.
.
.

```

Now run rqt_tf_tree.rqt_tf_tree is a runtime tool for visualizing the tree of frames being broadcast over ROS.
```
cd catkin_ws
source devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree
```
To view frames run the following commands:
Open a new terminal 
```
cd catkin_ws
source devel/setup.bash
cd catkin_ws/src/beginner_tutorials/results
rosrun tf view_frames
```
# Sample output:
```
Listening to /tf for 5.000000 seconds
Done Listening
dot - graphviz version 2.38.0 (20140413.2041)

Detected dot version 2.38
frames.pdf generated
```
view_frames creates a diagram of the frames being broadcast by tf over ROS. This will generate frames.pdf file. An sample example is provided in results folder.
#In order to view frames.pdf file:
```
cd catkin_ws/src/beginner_tutorials/results
evince TFframes.pdf
```
## To record the topics using rosbag using launch files
Open a new terminal and run the following commands:
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials Week10_HW.launch publisherFrequency:=<enter frequency> rosbagRecord:=true
```
This will record all the topics for 15 seconds and store it in record.bag file in results folder

## How to test .bag file
Open a new terminal; and run the following commands:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
Open another terminal; and run the following commands:
```
cd catkin_ws
source devel/setup.bash
cd catkin_ws/src/beginner_tutorials/results
rosbag play record.bag
```

## Sample Output
In the terminal running listener node
```
[ INFO] [1573480345.419338999]: I heard: ENPM808X Assignment : Line : 3
[ INFO] [1573480345.518954065]: I heard: ENPM808X Assignment : Line : 4
[ INFO] [1573480345.618976647]: I heard: ENPM808X Assignment : Line : 5
[ INFO] [1573480345.718802274]: I heard: ENPM808X Assignment : Line : 6
[ INFO] [1573480345.818916294]: I heard: ENPM808X Assignment : Line : 7
[ INFO] [1573480345.919014375]: I heard: ENPM808X Assignment : Line : 8
[ INFO] [1573480346.018842080]: I heard: ENPM808X Assignment : Line : 9
[ INFO] [1573480346.118822679]: I heard: ENPM808X Assignment : Line : 10
[ INFO] [1573480346.219029014]: I heard: ENPM808X Assignment : Line : 11
.
.
.

```
In the terminal running rosbag
```
[ INFO] [1573480344.831133182]: Opening record.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [DELAYED]  Bag Time: 1573476969.882342   Duration: 0.000000 / 14.884929   Delay
 [RUNNING]  Bag Time: 1573476969.882342   Duration: 0.000000 / 14.884929        
 [RUNNING]  Bag Time: 1573476969.882342   Duration: 0.000000 / 14.884929        
 [RUNNING]  Bag Time: 1573476969.882891   Duration: 0.000549 / 14.884929        
 [RUNNING]  Bag Time: 1573476969.983102   Duration: 0.100760 / 14.884929        
 [RUNNING]  Bag Time: 1573476970.083379   Duration: 0.201037 / 14.884929        
 [RUNNING]  Bag Time: 1573476970.097835   Duration: 0.215493 / 14.884929        
 [RUNNING]  Bag Time: 1573476970.098785   Duration: 0.216443 / 14.884929        
 [RUNNING]  Bag Time: 1573476970.099412   Duration: 0.217071 / 14.884929        
 [RUNNING]  Bag Time: 1573476970.166757   Duration: 0.284415 / 14.884929        
 [RUNNING]  Bag Time: 1573476970.167727   Duration: 0.285386 / 14.884929      
 .
 .
 .
```