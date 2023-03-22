/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api_tutorial");

int main(int argc, char* argv[])
{
 rclcpp::init(argc, argv);

 rclcpp::NodeOptions node_options;
 node_options.automatically_declare_parameters_from_overrides(true);
 std::shared_ptr<rclcpp::Node> motion_planning_api_tutorial_node =
     rclcpp::Node::make_shared("motion_planning_api_tutorial", node_options);

 rclcpp::executors::SingleThreadedExecutor executor;
 executor.add_node(motion_planning_api_tutorial_node);
 std::thread([&executor]() { executor.spin(); }).detach();

 // BEGIN_TUTORIAL
 // Start
 // ^^^^^
 // Setting up to start using a planner is pretty easy. Planners are
 // setup as plugins in MoveIt and you can use the ROS pluginlib
 // interface to load any planner that you want to use. Before we can
 // load the planner, we need two objects, a RobotModel and a
 // PlanningScene. We will start by instantiating a
 // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
 // object, which will look up the robot description on the ROS
 // parameter server and construct a
 // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
 // for us to use.
 const std::string PLANNING_GROUP = "arm";
 robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_api_tutorial_node, "robot_description");
 const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
 /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
 moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
 const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

 // Using the
 // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`,
 // we can construct a
 // :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>`
 // that maintains the state of the world (including the robot).
 planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

 // We will now construct a loader to load a planner, by name.
 // Note that we are using the ROS pluginlib library here.
 std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
 planning_interface::PlannerManagerPtr planner_instance;
 std::string planner_plugin_name;

 // We will get the name of planning plugin we want to load
 // from the ROS parameter server, and then load the planner
 // making sure to catch all exceptions.
 if (!motion_planning_api_tutorial_node->get_parameter("planning_plugin", planner_plugin_name))
   RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
 try
 {
   planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
       "moveit_core", "planning_interface::PlannerManager"));
 }
 catch (pluginlib::PluginlibException& ex)
 {
   RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
 }
 try
 {
   planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
   if (!planner_instance->initialize(robot_model, motion_planning_api_tutorial_node,
                                     motion_planning_api_tutorial_node->get_namespace()))
     RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
   RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
 }
 catch (pluginlib::PluginlibException& ex)
 {
   const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
   std::stringstream ss;
   for (const auto& cls : classes)
     ss << cls << " ";
   RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                ex.what(), ss.str().c_str());
 }

 moveit::planning_interface::MoveGroupInterface move_group(motion_planning_api_tutorial_node, PLANNING_GROUP);

 // Visualization
 // ^^^^^^^^^^^^^
 // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
 // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
 namespace rvt = rviz_visual_tools;
 auto visual_tools = moveit_visual_tools::MoveItVisualTools {motion_planning_api_tutorial_node, "base_link",
                                                             "move_group_tutorial", move_group.getRobotModel()};
 visual_tools.enableBatchPublishing();
 visual_tools.deleteAllMarkers();  // clear all old markers
 visual_tools.trigger();

 /* Remote control is an introspection tool that allows users to step through a high level script
    via buttons and keyboard shortcuts in RViz */
 visual_tools.loadRemoteControl();

 /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
 Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
 text_pose.translation().z() = 1.75;
 visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

 /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
 visual_tools.trigger();

 // Pose Goal
 // ^^^^^^^^^
 // We will now create a motion plan request for the arm of the Sherlock
 // specifying the desired pose of the end-effector as input.
 visual_tools.trigger();

 for(int i = 0; i <2; i++)
 {
   geometry_msgs::msg::PoseStamped pose;
   pose.header.frame_id = "world";

   pose.pose.position.x = 1.0;
   pose.pose.position.y = 1.0;
   pose.pose.position.z = i + 1.0;
   pose.pose.orientation.w = 0.707;
   pose.pose.orientation.z = 0.707;
   pose.pose.orientation.y = 0.0;
   pose.pose.orientation.x = 0.0;

   // Display the goal state
   visual_tools.publishAxisLabeled(pose.pose, "goal_1");
   visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
   visual_tools.trigger();

   moveit::core::RobotState start_state(*move_group.getCurrentState());
   move_group.setStartState(start_state);
   move_group.setPoseTarget(pose, "gripper");
   move_group.setPlannerId("RRTstarkConfigDefault");
   move_group.setNumPlanningAttempts(10);
   move_group.setPlanningTime(5.0);

   // Set the maximum velocity scaling factor
   move_group.setMaxVelocityScalingFactor(1.0);

   // Set the maximum acceleration scaling factor
   move_group.setMaxAccelerationScalingFactor(1.0);

   auto const [success, plan] = [&move_group] {
     moveit::planning_interface::MoveGroupInterface::Plan msg;
     auto const ok = static_cast<bool>(move_group.plan(msg));
     return std::make_pair(ok, msg);
   }();

   // Visualize the result
   // ^^^^^^^^^^^^^^^^^^^^
   std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
       motion_planning_api_tutorial_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
                                                                                                1);
   moveit_msgs::msg::DisplayTrajectory display_trajectory;

   // display_trajectory.trajectory_start = response.trajectory_start;
   display_trajectory.trajectory.push_back(plan.trajectory_);
   visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
   visual_tools.trigger();
   display_publisher->publish(display_trajectory);

   if (success)
   {
     move_group.execute(plan);
   }
 }

 // END_TUTORIAL
 planner_instance.reset();

 rclcpp::shutdown();
 return 0;
}
