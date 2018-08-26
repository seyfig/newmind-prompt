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
catkin_create_pkg newmind_test
```


## 5. Create ImageAnalyzer.action
[actionlib](http://wiki.ros.org/actionlib)
[Tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
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
Add actionlib_msgs to catkin_package macro
```
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
```

### 5.3. package.xml

package.xml should include the following dependencies.

```
<exec_depend>message_generation</exec_depend>
<build_depend>actionlib_msgs</build_depend>
<exec_depend>actionlib_msgs</exec_depend>
<build_depend>actionlib</build_depend>
<exec_depend>actionlib</exec_depend>
```