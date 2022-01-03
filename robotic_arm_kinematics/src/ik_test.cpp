#include <robotic_arm_kinematics/inverse_kinematics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

using namespace robotic_arm_kinematics;

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

void getPoseInput(geometry_msgs::Pose &target_pose)
{
  std::cout<<"Enter the new set of target position: x,y,z,xR,yR,zR,wR"<<'\n';
  double x,y,z,xr,yr,zr,wr;

  std::vector<double>ret_val;

  std::cin>>x;
  ret_val.push_back(x);

  std::cin>>y;
  ret_val.push_back(y);

  std::cin>>z;
  ret_val.push_back(z);

  std::cin>>xr;
  ret_val.push_back(xr);

  std::cin>>yr;
  ret_val.push_back(yr);

  std::cin>>zr;
  ret_val.push_back(zr);

  std::cin>>wr;
  ret_val.push_back(wr);

  target_pose.position.x=ret_val[0];
  target_pose.position.y=ret_val[1];
  target_pose.position.z=ret_val[2];
  target_pose.orientation.x=ret_val[3];
  target_pose.orientation.y=ret_val[4];
  target_pose.orientation.z=ret_val[5];
  target_pose.orientation.w=ret_val[6];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "venkat_ik_node");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "robotic_arm";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  geometry_msgs::Pose target_pose;
  bool cont{true};
  std::vector<std::array<double, 4>> angle_op;

  //Create an Inverse Kinematics class object
  InverseKinematics ik_robotic_arm_solver;

  while(true)
  {
    std::cout<<"Enter yes to give a target pose and no to quit"<<'\n';
    std::string usr_ip;
    std::cin>>usr_ip;

    cont = (usr_ip=="yes") ? true: false;

    if (cont)
    {
      getPoseInput(target_pose);
      ik_robotic_arm_solver.CartToJnt(target_pose, angle_op);

      moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
      joint_group_positions[0] = angle_op[0][0]; 
      joint_group_positions[1] = angle_op[0][1];
      joint_group_positions[2] = angle_op[0][2];
      joint_group_positions[3] = angle_op[0][3];

      move_group_interface.setJointValueTarget(joint_group_positions);

      // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
      // The default values are 10% (0.1).
      // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
      // or set explicit factors in your code if you need your robot to move faster.
      move_group_interface.setMaxVelocityScalingFactor(0.05);
      move_group_interface.setMaxAccelerationScalingFactor(0.05);

      success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      for (auto a: joint_group_positions)
      {
        ROS_INFO("Joint Value:%f", a);
      }

      ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

      std::cout<<"Do you want to see the other set of solution? (yes or no)"<<'\n';
      std::string other_sol_str;
      std::cin>>other_sol_str;
      bool other_sol{false};
      other_sol = (other_sol_str=="yes") ? true: false;

      if (other_sol)
      {
        // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
        joint_group_positions[0] = angle_op[1][0]; 
        joint_group_positions[1] = angle_op[1][1];
        joint_group_positions[2] = angle_op[1][2];
        joint_group_positions[3] = angle_op[1][3];

        move_group_interface.setJointValueTarget(joint_group_positions);

        success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        for (auto a: joint_group_positions)
        {
          ROS_INFO("Joint Value:%f", a);
        }

        ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

        // Visualize the plan in RViz
        visual_tools.deleteAllMarkers();
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      }
    }

    else break; 
  }

  ros::shutdown();
  return 0;
}
