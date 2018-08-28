## 2. Install ROS and OpenCV

### 2.1. Install ROS Kinetic  [http://wiki.ros.org/kinetic/Installation/Ubuntu]

#### Setup your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Set up your keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

#### Make Debian package list Up-to-date 
```
sudo apt-get update
```

#### Install ROS-Base (Bare Bones)
TODO
It should be enough
```
sudo apt-get install ros-kinetic-ros-base
```

#### Initilize rosdep

```
sudo rosdep init
rosdep update
```

#### Create Project Folder
```
mkdir catkin_ws
```

### 2.2. Install Opencv
```
sudo	apt-get	install	ros-kinetic-opencv3
```

## 3. Create a Catkin Workspace
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

## 4. Create a ROS package called “newmind_test”.

```
cd ~/catkin_ws/src
catkin_create_pkg newmind_test actionlib message_generation roscpp rospy std_msgs actionlib_msgs
```


## 5. Create ImageAnalyzer.action
[actionlib](http://wiki.ros.org/actionlib)
[Server Execute Callback Tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)

[Server Goal Callback Tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29)

[Client Tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient)
### 5.1. Create the action file with the following content
```
# Define the goal
bool mode  # Specify the mode
---
# Define the result
float32[] color # Color
---
# Define a feedback message
# N/A
```

### 5.2. CMakeLists.txt
Add following lines to the CMakeLists.txt before catkin_package().
```
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)
add_action_files(DIRECTORY action FILES ImageAnalyzer.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)
```

TODOD
didnt do it
Add actionlib_msgs to catkin_package macro
```
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
```



## 6. Server and Client

### 6.1. Server
#### 6.1.2. CMakeLists.txt
```
add_executable(image_analyzer_server src/image_analyzer_server.cpp)
```

```
target_link_libraries(
  image_analyzer_server
  ${catkin_LIBRARIES}
)
```

```
add_dependencies(
  image_analyzer_server
  ${newmind_test_EXPORTED_TARGETS}
)
```

### 6.2. Client
#### 6.1.2. CMakeLists.txt


```
add_executable(image_analyzer_client src/image_analyzer_client.cpp)
```
```
target_link_libraries(
  image_analyzer_client
  ${catkin_LIBRARIES}
)
```

```
add_dependencies(
  image_analyzer_client
  ${newmind_test_EXPORTED_TARGETS}
)

```