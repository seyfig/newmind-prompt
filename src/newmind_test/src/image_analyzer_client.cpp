#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <newmind_test/ImageAnalyzerAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "image_analyzer_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<newmind_test::ImageAnalyzerAction> ac("image_analzer", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  newmind_test::ImageAnalyzerGoal goal;
  goal.mode = true;

  ROS_INFO("Sending goal with mode %s", (goal.mode ? "true" : "false"));
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    newmind_test::ImageAnalyzerResultConstPtr result_ = ac.getResult();
    for (int i = 0; i < 3; i++) {
      ROS_INFO("%d. %f",i,result_->color[i]);
    }
    //TODO goal doesnt have color, result has
    //ROS_INFO(goal.color);
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
