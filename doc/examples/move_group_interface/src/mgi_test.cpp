#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 3.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("arm")](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Set a target Pose
  while(rclcpp::ok()) {
    for (int i = 0; i < 1; i++) {

      geometry_msgs::msg::PoseStamped target_pose;
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.707;
      msg.orientation.z = 0.707;
      msg.orientation.y = 0.0;
      msg.orientation.x = 0.0;
      msg.position.x = -0.7;
      msg.position.y = 1.3;
      msg.position.z = 0.8 + i * (0.5);

      target_pose.pose = msg;
      target_pose.header.frame_id = "world";

      move_group_interface.setPoseTarget(target_pose, "gripper");
      move_group_interface.setPlannerId("RRTstarkConfigDefault");
      move_group_interface.setNumPlanningAttempts(10);
      move_group_interface.setPlanningTime(5.0);

      // Set the maximum velocity scaling factor
      move_group_interface.setMaxVelocityScalingFactor(1.0);

      // Set the maximum acceleration scaling factor
      move_group_interface.setMaxAccelerationScalingFactor(1.0);


      /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.75;
      // Display the goal state
      moveit_visual_tools.publishAxisLabeled(msg, "goal_1");
      moveit_visual_tools.publishText(text_pose, "Pose Goal (1)", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      moveit_visual_tools.trigger();


      // Create a plan to that target pose
      // prompt("Press 'next' in the RvizVisualToolsGui window to plan");
      draw_title("Planning");
      moveit_visual_tools.trigger();
      auto [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();

      // compute velocity information for the trajectory
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points = plan.trajectory_.joint_trajectory.points;
      for (size_t i1 = 1; i1 < points.size(); ++i1) {
        trajectory_msgs::msg::JointTrajectoryPoint& prev_point = points[i1 -1];
        trajectory_msgs::msg::JointTrajectoryPoint& curr_point = points[i1];
        double dt = curr_point.time_from_start.sec - prev_point.time_from_start.sec;
        for (size_t j = 0; j < prev_point.velocities.size(); ++j) {
          double joint_position_diff = curr_point.positions[j] - prev_point.positions[j];
          double joint_velocity = joint_position_diff / dt;
          curr_point.velocities[j] = joint_velocity;
        }
      }

      //    // add velocity smoothing to the trajectory using the AddRuckigTrajectorySmoothing adapter
      //    moveit::planning_interface::PlannerManagerPtr planner_manager = move_group.getPlannerInstance();
      //    moveit::planning_interface::PlanningPipelinePtr planning_pipeline = std::make_shared<moveit::planning_interface::PlanningPipeline>(move_group.getRobotModel(), move_group.getNodeHandle(), planner_manager);
      //    planning_pipeline->getRequestAdapterChainNonConst().insert(std::make_shared<moveit::planning_request_adapter::AddRuckigTrajectorySmoothing>());
      //    planning_pipeline->generatePlan(move_group.getPlanningScene(), move_group.getGoal(), plan);
      //
      //

      // Execute the plan
      bool success_flag = true;
      if (success) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        //      prompt("Press 'next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
      } else {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
        success_flag = false;
        break;
      }

      sleep(1);
    }

  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}