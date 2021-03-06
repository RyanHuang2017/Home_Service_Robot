#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickUp_goal;

  // set up the frame parameters
  pickUp_goal.target_pose.header.frame_id = "map";
  pickUp_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickUp_goal.target_pose.pose.position.x = -1.0;
  pickUp_goal.target_pose.pose.position.y = -1.0;
  pickUp_goal.target_pose.pose.orientation.w = -1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickUp_goal");
  ac.sendGoal(pickUp_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached the pick up section");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  
  // Wait 5 sec for move_base action server to come up
  ros::Duration(5.0);
  


  ///// define a drop off gaol
  move_base_msgs::MoveBaseGoal dropOff_goal;

  // set up the frame parameters
  dropOff_goal.target_pose.header.frame_id = "map";
  dropOff_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  dropOff_goal.target_pose.pose.position.x = 4.0;
  dropOff_goal.target_pose.pose.position.y = 4.0;
  dropOff_goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending dropOff_goal");
  ac.sendGoal(dropOff_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached both goal");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");



  return 0;
}
