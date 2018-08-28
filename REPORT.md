# newmind-prompt Report

## 1. Introduction
The report contains the performed steps from scratch.

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
The following tutorials were followed to complete the action server and action client nodes.  

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

## 7. Server
### Requirements
#### OpenCV
Since the server node requires OpenCV, the ros-kinetic-opencv3 library should be installed. In addition, it is required to add opencv to the CMakeLists.txt. The required changes are listed below.

```
find_package( OpenCV REQUIRED )
```

```
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
```
```
target_link_libraries(
  image_analyzer_server
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
```

#### ros/package.h
To find the relative path of the newmind_text project, the ros/package.h is needed. The path was used to obtain the path to the image.

### Code
The image_analyzer_server.cpp reads an image file from the path "catkin_ws/src/img/1.jpg". To find the most common or least common pixel color, the image_analyzer_server node creates a dictionary and counts the occurrence of each color performing pixel by pixel search.

## 8. Client
The client, sends goal to the server and prints out the resulting color. It sets the mode parameter for the server. The next steps change the procedure to set this parameter by using the ros parameter "mode".

## 9. The Launch File
The launch files takes an argument mode and sets the ros parameter mode equal to this argument. The default value for the mode is true.

The launch file starts the image_analyzer_server first, and the image_analyzer_client next. It sends the output of the client to the console (output="screen").

## 10. Test
### 1. Requirements
In order to run the project it is required to install the requirements.
### 2. Clone
Clone the repository under any folder. Clonning will create "newmind-prompt" folder, and it will become the working folder.

```
git clone https://github.com/seyfig/newmind-prompt.git
```

Change folder to newmind-prompt
```
cd newmind-prompt
```

### 3. Source ros (! This isn't required if it is already sourced)
```
source /opt/ros/kinetic/setup.bash
```
### 4. Catkin Make
```
catkin_make
```

### 5. Source project
```
source devel/setup.bash
```

### 6. Image file
The project has a default image at the path (src/img/1.jpg). To test with a different image, it is required to replace the 1.jpg file.

### 7. Launch file

With mode = true
```
roslaunch newmind_test test.launch
```

With mode = false

```
roslaunch newmind_test test.launch mode:=false
```
