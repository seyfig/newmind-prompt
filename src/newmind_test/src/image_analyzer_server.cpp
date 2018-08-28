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
    ROS_INFO("Creating Image Analyzer server instance");
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
      //cv::Mat image = cv::imread("./src/img/1.jpg", CV_LOAD_IMAGE_COLOR);
      cv::Mat image = cv::imread("./src/img/t1.png", CV_LOAD_IMAGE_COLOR);

      if (image.empty()) {
        as_.setAborted(result_);
        ROS_INFO("%s: Aborted, could not load the image", action_name_.c_str());
        return;
      }
      else {

      }

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

      int common_count = 0;
      int common_key = -1;
      std::map<int, int>::iterator it;
      std::vector<int> common_color;

      // Find most common
      if (goal->mode) {
        common_count = 0;
        for (it = color_counts.begin(); it != color_counts.end(); it++) {
          if (it->second > common_count) {
            common_count = it->second;
            common_key = it->first;
          }
        }
      }
      // Find least common
      else {
        common_count = rows * cols + 1;
        for (it = color_counts.begin(); it != color_counts.end(); it++) {
          if (it->second < common_count) {
            common_count = it->second;
            common_key = it->first;
          }
        }
      }

      common_color.push_back(common_key % 256);
      common_color.push_back((common_key / 256) % 256);
      common_color.push_back((common_key / 256) / 256);
      ROS_INFO("%s Key:%d, color:%d,%d,%d, count:%d",
        (goal->mode ? "MAX" : "MIN"),
        common_key,
        common_color[0],
        common_color[1],
        common_color[2],
        common_count);


      int max_count = 0;
      int max_key = -1;
      int min_count = rows * cols + 1;
      int min_key = -1;
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
      result_.color.push_back((float)common_color[0]);
      result_.color.push_back((float)common_color[1]);
      result_.color.push_back((float)common_color[2]);

      // set the action state to succeeded
      as_.setSucceeded(result_);
      ROS_INFO("%s: Succeeded END", action_name_.c_str());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_analzer_server");
  ImageAnalyzerAction imageAnalyzerServer("image_analzer");
  ros::spin();

  return 0;
}
