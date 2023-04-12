#include <iostream>
#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/bt_factory.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "message_filters/sync_policies/approximate_time.h"
#include "ur_interfaces/msg/segment.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "ros.h"

namespace rvt = rviz_visual_tools;
using namespace message_filters;
using namespace message_filters::sync_policies;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("");

//Pass a vector to this function & fill with values ready to be passed to move_groups "setJointValueTarget"
void make_joint_array(std::vector<double> &array, double a, double b, double c, double d, double e, double f, bool degrees=false){
    double cf = 1; //Conversion factor for rad/deg conversion
    if (degrees){
        cf = 3.1415/180;
    }
    if (array.empty()){
        array.push_back(a*cf);
        array.push_back(b*cf);
        array.push_back(c*cf);
        array.push_back(d*cf);
        array.push_back(e*cf);
        array.push_back(f*cf);
    }
    else{
        array.at(0) = a*cf;
        array.at(1) = b*cf;
        array.at(2) = c*cf;
        array.at(3) = d*cf;
        array.at(4) = e*cf;
        array.at(5) = f*cf;
    }
}

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
           extern const std::string PLANNING_GROUP;

        auto node = Ros::instance()->node();
        
        //Setup moveit objects
        moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        //Rviz vizualization
        moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "ur_behavior_tree",
                                                            move_group.getRobotModel());
        visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools.publishText(text_pose, "ur_behavior_tree", rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::vector<double> joint_group_positions;
        make_joint_array(joint_group_positions, 1.397, 0.743, -0.710, -5.071, -1.675, 0.100, false /*Passing rad*/);
        move_group.setJointValueTarget(joint_group_positions);
        move_group.plan(my_plan);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

        //Move robot
        move_group.move();

        //Make plan
        // move_group.setGoalPositionTolerance(0.02);
        // move_group.setGoalOrientationTolerance(0.04);
        // geometry_msgs::msg::Pose target_pose1;
        // //target_pose1.header.frame_id = "base_link";
        // target_pose1.orientation.w = 0.1;
        // target_pose1.position.x = -0.056;
        // target_pose1.position.y = 0.294;
        // target_pose1.position.z = 0.194;
        // move_group.setPoseTarget(target_pose1);

        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        // RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        //Vizualize plan
        //visual_tools.publishAxisLabeled(target_pose1, "pose1");
        //visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
        //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        //visual_tools.trigger();

        std::cout << "Magic!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

BT::NodeStatus RequestHelp()
{
    std::cout << "Help Requested" << std::endl;
    extern rclcpp::Node::SharedPtr node;
    extern const std::string PLANNING_GROUP;
    //Setup moveit objects
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<double> joint_group_positions;
    make_joint_array(joint_group_positions, 1.505, 0.845, 0.509, -4.521, -1.527, 0.058, false /*Passing rad*/);
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    move_group.move();

    move_group.setMaxVelocityScalingFactor(0.6);
    move_group.setMaxAccelerationScalingFactor(0.3);

    make_joint_array(joint_group_positions, 1.905, 0.846, 0.509, -4.522, -0.910, 0.058, false /*Passing rad*/);
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    move_group.move();

    make_joint_array(joint_group_positions, 1.133, 0.846, 0.509, -4.450, -2.076, 0.0485, false /*Passing rad*/);
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    move_group.move();

    make_joint_array(joint_group_positions, 1.905, 0.846, 0.509, -4.522, -0.910, 0.058, false /*Passing rad*/);
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    move_group.move();

    make_joint_array(joint_group_positions, 1.505, 0.845, 0.509, -4.521, -1.527, 0.058, false /*Passing rad*/);
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    move_group.move();

    system("read");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FindObjML()
{
    std::cout << "Finding Object using ML model" << std::endl;

    return BT::NodeStatus::FAILURE;
}

struct Header
{
  rclcpp::Time stamp;
};


struct Msg
{
  Header header;
  int data;
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

class FindObjPC : public BT::SyncActionNode
{
    public:
    FindObjPC(const std::string& name) : BT::SyncActionNode(name, {})
    {
        RCLCPP_INFO(LOGGER, "Initialize node");
        auto seg_sub = Ros::instance()->node()->create_subscription<ur_interfaces::msg::Segment>("/detect_server/segmentations", 10, FindObjPC::seg_callback);
        auto img_sub = Ros::instance()->node()->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", 10, FindObjPC::img_callback);
    }

    void seg_callback(const ur_interfaces::msg::Segment seg)
    {
        seg_ = seg;
    }

    void img_callback(const sensor_msgs::msg::Image img)
    {
        img_ = img;
    }

    BT::NodeStatus tick() override
    {
        return BT::NodeStatus::SUCCESS;
    }
    
    private:
        ur_interfaces::msg::Segment seg_;
        sensor_msgs::msg::Image img_;

};

BT::NodeStatus GoToObj()
{
    std::cout << "Going to object" << std::endl;

    return BT::NodeStatus::FAILURE;
}