#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "tree_nodes.hpp"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string PLANNING_GROUP = "ur_manipulator";
static const std::string LOGNAME = "ur_behavior_tree";
static const std::vector<std::string> CONTROLLERS(1, "ur_controllers");

int main(int argc, char * argv[])
{
    //Make ROS2 node and spin thread
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    RCLCPP_INFO(LOGGER, "Initialize node");
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ur_behavior_tree", "", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    //Setup moveit objects
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

    //Rviz vizualization
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "ur_behavior_tree",
                                                        moveit_cpp_ptr->getPlanningSceneMonitor());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "ur_behavior_tree", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    //Move robot
    planning_components->setStartStateToCurrentState();
    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "base_link";
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x = 0.28;
    target_pose1.pose.position.y = -0.2;
    target_pose1.pose.position.z = 0.5;
    planning_components->setGoal(target_pose1, "tool0");

    const moveit_cpp::PlanningComponent::PlanSolution plan_solution1 = planning_components->plan();
    if(plan_solution1)
    {
        visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("tool0"), "start_pose");
        visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
        visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
        visual_tools.trigger();
    }

    //Setup and run behavior tree
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GotoHomePosition>("GotoHomePosition");

    auto tree = factory.createTreeFromFile("../behavior_trees/ur_bt.xml");
    tree.tickWhileRunning();

    return 0;
}