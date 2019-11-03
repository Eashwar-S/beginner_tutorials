# beginner_tutorials of Publisher/Subscriber node in ROS
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
roslaunch beginner_tutorials Week10_HW.launch
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
