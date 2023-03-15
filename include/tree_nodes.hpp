#include <iostream>
#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/bt_factory.h"
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("");

class GotoHomePosition : public BT::SyncActionNode
{
    public:
    GotoHomePosition(const std::string& name) : BT::SyncActionNode(name, {})
    {
        rclcpp::NodeOptions node_options;
        RCLCPP_INFO(LOGGER, "Initialize node");
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Magic!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};