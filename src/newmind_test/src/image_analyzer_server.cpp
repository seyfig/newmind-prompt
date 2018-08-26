#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <newmind_test/ImageAnalyzerAction.h>

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
      result_.color.clear();
      result_.color.push_back(5.0);
      result_.color.push_back(5.0);
      result_.color.push_back(5.0);
      //[5,5,5];
      ROS_INFO("%s: Succeeded SET", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);

      for (int i = 0; i < 3; i++) {
        ROS_INFO("%d. %f",i,result_.color[i]);
      }
      //ROS_INFO(result_.color);
      //ROS_INFO(result_);

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
