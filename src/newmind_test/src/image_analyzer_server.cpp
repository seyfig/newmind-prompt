#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <newmind_test/ImageAnalyzerAction.h>
#include <opencv2/highgui/highgui.hpp>

static bool pixel_equal(const cv::Vec3b &a, const cv::Vec3b &b)
{   return a == b;   }

class ImageAnalyzerAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<newmind_test::ImageAnalyzerAction> as_;
  std::string action_name_;
  // create messages that are used to published result
  newmind_test::ImageAnalyzerResult result_;

public:

  ImageAnalyzerAction(std::string name) :
    as_(nh_, name, boost::bind(&ImageAnalyzerAction::executeCB, this, _1), false),
    action_name_(name)
  {
    ROS_INFO("IA Constructor called");
    as_.start();
  }

  ~ImageAnalyzerAction(void)
  {
  }

  void executeCB(const newmind_test::ImageAnalyzerGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, mode: %s", action_name_.c_str(), (goal->mode ? "true" : "false"));

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    }


    if(success)
    {
      ROS_INFO("%s: Succeeded START", action_name_.c_str());
      cv::Mat image = cv::imread("./src/img/1.jpg", CV_LOAD_IMAGE_COLOR);
      //cv::Mat image = cv::imread("./src/img/t1.png", CV_LOAD_IMAGE_COLOR);
      int rows = image.rows;
      int cols = image.cols;
      ROS_INFO("Image is %d X %d", rows, cols);

      std::map<int,int> color_counts;
      for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
          cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(j,i));
          int key = 256 * 256 * color[2] + 256 * color[1] + color[0];
          int key_count = 0;
          std::map<int,int>::iterator iter;
          iter = color_counts.find(key);
          if (iter != color_counts.end()) {
            iter->second++;
            key_count = iter->second;
          }
          else {
            color_counts.insert(std::make_pair(key, 1));
            key_count = 1;
          }
          //ROS_INFO("Row:%d, Col:%d, Key:%d, color: %d,%d,%d, count:%d",i,j,key, color[0], color[1], color[2], key_count);

        }
      }

      int max_count = 0;
      int max_key = -1;
      int min_count = rows * cols + 1;
      int min_key = -1;
      std::map<int, int>::iterator it;
      for (it = color_counts.begin(); it != color_counts.end(); it++) {
        if (it->second > max_count) {
          max_count = it->second;
          max_key = it->first;
        }
        if (it->second < min_count) {
          min_count = it->second;
          min_key = it->first;
        }
      }
      std::vector<int> max_color;
      max_color.push_back(max_key % 256);
      max_color.push_back((max_key / 256) % 256);
      max_color.push_back((max_key / 256) / 256);
      ROS_INFO("Max Key:%d, color:%d,%d,%d, count:%d", max_key, max_color[0], max_color[1], max_color[2], max_count);

      std::vector<int> min_color;
      min_color.push_back(min_key % 256);
      min_color.push_back((min_key / 256) % 256);
      min_color.push_back((min_key / 256) / 256);
      ROS_INFO("Min Key:%d, color:%d,%d,%d, count:%d", min_key, min_color[0], min_color[1], min_color[2], min_count);

      result_.color.clear();
      result_.color.push_back((float)max_color[0]);
      result_.color.push_back((float)max_color[1]);
      result_.color.push_back((float)max_color[2]);

      ROS_INFO("%s: Succeeded SET", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);

      for (int i = 0; i < 3; i++) {
        ROS_INFO("%d. %f",i,result_.color[i]);
      }

      ROS_INFO("%s: Succeeded END", action_name_.c_str());
    }
  }


};



int main(int argc, char** argv)
{
  ROS_INFO("Init IA server");
  ros::init(argc, argv, "image_analzer_server");
  ROS_INFO("Constructor IA server");
  ImageAnalyzerAction imageAnalyzerServer("image_analzer");
  ROS_INFO("Created IA server instance");
  ros::spin();
  ROS_INFO("IA server start spin");

  return 0;
}
