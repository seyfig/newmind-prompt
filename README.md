[//]: # (References)
[1]: ./REPORT.md "Report"

# newmind-prompt
## Project

The goal of this project is to complete the task of the newmind-prompt document.

A more detailed report can be found in [Report][1].  

The steps were completed on a Virtual Machine Ubuntu Ubuntu 16.04.5 LTS with 4 CPU Cores (i5-3427U CPU @ 1.80GHz × 4) and 2.5GB RAM.

## Requirements

### ROS Kinetic

The project was completed with ROS kinetic.

### OpenCV

The project utilizes ros-kinetic-opencv3 library.

## Run
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
The project searches for an image file called 1.jpg under the img folder of the src folder (src/img/1.jpg). This location was hard coded currently. To test with a different image, it is required to replace the 1.jpg file.

### 7. Launch file
There is a mode parameter to select how to search for the common pixel. If this parameter is true, the image_analyzer_server node finds the most common pixel color, and the least common pixel color otherwise.

Run with mode = true, since true is the default value there is no need to supply the argument.
```
roslaunch newmind_test test.launch
```


With mode = false

```
roslaunch newmind_test test.launch mode:=false
```

## The Most (or Least) Common Pixel Color
To find the most common or least common pixel color, the image_analyzer_server node creates a dictionary and counts the occurrence of each color performing pixel by pixel search. 