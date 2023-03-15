#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "tree_nodes.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const std::string PLANNING_GROUP = "ur_manipulator";
rclcpp::Node::SharedPtr node;
//static const std::string LOGNAME = "ur_behavior_tree";
//static const std::vector<std::string> CONTROLLERS(1, "ur_controllers");


std::vector<moveit_msgs::msg::CollisionObject> add_collision_environment(std::string frame_id)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(1);
    collision_objects[0].id = "mount_frame";
    collision_objects[0].header.frame_id = frame_id;

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(2);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.3556;
    collision_objects[0].primitives[0].dimensions[1] = 0.4318;
    collision_objects[0].primitives[0].dimensions[2] = 1.5;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(2);
    collision_objects[0].primitive_poses[0].position.x = 0; //*sin(45deg)
    collision_objects[0].primitive_poses[0].position.y = -0.4459;
    collision_objects[0].primitive_poses[0].position.z = -0.000;
    collision_objects[0].primitive_poses[0].orientation.w = 0.5;
//    collision_objects[0].operation = collision_objects[0].ADD;
    
    //Floor
    collision_objects[0].primitives[1].type = collision_objects[0].primitives[1].BOX;
    collision_objects[0].primitives[1].dimensions.resize(3);
    collision_objects[0].primitives[1].dimensions[0] = 2;
    collision_objects[0].primitives[1].dimensions[1] = 2;
    collision_objects[0].primitives[1].dimensions[2] = 0.01;

    //Floor pose
    collision_objects[0].primitive_poses[1].position.x = 0; //*sin(45deg)
    collision_objects[0].primitive_poses[1].position.y = 0;
    collision_objects[0].primitive_poses[1].position.z = 0.75;
    collision_objects[0].primitive_poses[1].orientation.w = 0.5;
    collision_objects[0].operation = collision_objects[0].ADD;
    
    return collision_objects;
}

int main(int argc, char * argv[])
{
    //Make ROS2 node and spin thread
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    RCLCPP_INFO(LOGGER, "Initialize node");
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("ur_behavior_tree", "", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = add_collision_environment("base_link");
    planning_scene_interface.applyCollisionObjects(collision_objects);

    //Setup and run behavior tree
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GotoHomePosition>("GotoHomePosition");

    auto tree = factory.createTreeFromFile("../behavior_trees/ur_bt.xml");
    tree.tickWhileRunning();

    return 0;
}