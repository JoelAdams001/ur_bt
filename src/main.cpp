#include <iostream>
#include "../include/ros.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include "../include/tree_nodes.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const std::string PLANNING_GROUP = "ur_manipulator";
rclcpp::Node::SharedPtr node;

moveit_msgs::msg::CollisionObject add_collision_environment(std::string frame_id)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "environment";
    collision_object.header.frame_id = frame_id;

    //Mount
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_Z] = 1.20;
    primitive.dimensions[primitive.BOX_X] = 0.15;
    primitive.dimensions[primitive.BOX_Y] = 0.15;

    geometry_msgs::msg::Pose primitive_pose;
    primitive_pose.orientation.w = 0.5;
    primitive_pose.position.x = 0; //*sin(45deg)
    primitive_pose.position.y = -0.4459;
    primitive_pose.position.z = 0.17;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(primitive_pose);
    
    //Floor
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2;
    primitive.dimensions[primitive.BOX_Y] = 2;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    primitive_pose.orientation.w = 0.5;
    primitive_pose.position.x = 0; //*sin(45deg)
    primitive_pose.position.y = 0;
    primitive_pose.position.z = 0.75;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(primitive_pose);

    //Cantilever
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.15;
    primitive.dimensions[primitive.BOX_Y] = 0.6;
    primitive.dimensions[primitive.BOX_Z] = 0.11;

    primitive_pose.orientation.w = 0.5;
    primitive_pose.position.x = 0;
    primitive_pose.position.y = -0.213; //-0.223
    primitive_pose.position.z = -0.06; //-0.045

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(primitive_pose);

    return collision_object;
}

int main(int argc, char * argv[])
{
    Ros ros(argc, argv,"ur_behavior_tree");
    ros.spinOnBackground();

    //Add environment to moveit planning scene
    moveit::planning_interface::MoveGroupInterface move_group(ros.node(), PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto collision_object = add_collision_environment("base_link");
    planning_scene_interface.applyCollisionObject(collision_object);

    //Setup and run behavior tree
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GotoHomePosition>("GotoHomePosition");
    factory.registerNodeType<FindObj>("FindObj");
    factory.registerSimpleCondition("RequestHelp", std::bind(RequestHelp));
    factory.registerSimpleCondition("GoToObj", std::bind(GoToObj));
    auto tree = factory.createTreeFromFile("/home/manipulatorlab/Ros2_ws/src/ur_bt/behavior_trees/ur_bt.xml");
    //BT::PublisherZMQ publish_zmw(tree);
    tree.tickWhileRunning();

    return 0;
}