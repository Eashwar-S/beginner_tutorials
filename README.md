# beginner_tutorials of Publisher/Subscriber node in ROS
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview:
This repository will provide an introduction for beginners who want to create and use Publisher/Subscriber node in ROS. It provides a step by step explaination of how to build and run a node and also how to subscribe and publish a topic.

## Assumptions:

1. The user must have Ubuntu 16.04 LTS version installed.
2. The user must have ROS kinetic and catkin installed.

## Executables Added:
In CMakeLists.txt the following executables are added to execute cpp files in src folder:
```
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
```

## Dependencies Added:
In CMakeLists.txt the following dependencies are added for executables:
```
add_dependencies(talker beginner_tutorials_generate_messages_cpp)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

## Target Link Libraries Added:
In CMakeLists.txt the following target link libraries to link executables:
```
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})
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
cd ..
catkin_make
```

## How to run
First us to run ROS master
To do that open a new terminal and type
```
roscore
```

To run talker.cpp open a new terminal and type
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```

To run listener.cpp open a new terminal and type
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
To stop the program press ctrl+C in each of the three terminals.

## Sample Output:

In the terminal which is running listener we get output
```
[ INFO] [1572206335.563977355]: hello world 0
[ INFO] [1572206335.664187572]: hello world 1
[ INFO] [1572206335.764084677]: hello world 2
[ INFO] [1572206335.864114938]: hello world 3
.
.
.
.


```
In the terminal which is running listener we get output
```
[ INFO] [1572206335.864863745]: I heard: [hello world 0]
[ INFO] [1572206335.964761637]: I heard: [hello world 1]
[ INFO] [1572206336.064633881]: I heard: [hello world 2]
[ INFO] [1572206336.164780497]: I heard: [hello world 3]
.
.
.
.

```


