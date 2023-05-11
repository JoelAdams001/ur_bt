//Structural libraries
#include <iostream>
#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/bt_factory.h"
#include "ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//ROS Messages
#include "ur_interfaces/msg/segment.hpp"
#include "ur_interfaces/msg/segimgcombo.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

//Additional Libraries
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <image_geometry/pinhole_camera_model.h>

using std::placeholders::_1;
namespace rvt = rviz_visual_tools;
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
        
        // //Setup moveit objects
        // moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // const moveit::core::JointModelGroup* joint_model_group =
        // move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        // //Rviz vizualization
        // moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "ur_behavior_tree",
        //                                                     move_group.getRobotModel());
        // visual_tools.deleteAllMarkers();
        // visual_tools.loadRemoteControl();

        // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        // text_pose.translation().z() = 1.75;
        // visual_tools.publishText(text_pose, "ur_behavior_tree", rvt::WHITE, rvt::XLARGE);
        // visual_tools.trigger();

        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // std::vector<double> joint_group_positions;
        // make_joint_array(joint_group_positions, 1.397, 0.743, -0.710, -5.071, -1.675, 0.100, false /*Passing rad*/);
        // move_group.setJointValueTarget(joint_group_positions);
        // move_group.plan(my_plan);
        // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

        // //Move robot
        // move_group.move();

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
    auto node = Ros::instance()->node();
    extern const std::string PLANNING_GROUP;
    //Setup moveit objects
    // moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // std::vector<double> joint_group_positions;
    // make_joint_array(joint_group_positions, 1.505, 0.845, 0.509, -4.521, -1.527, 0.058, false /*Passing rad*/);
    // move_group.setJointValueTarget(joint_group_positions);
    // move_group.plan(my_plan);
    // move_group.move();

    // move_group.setMaxVelocityScalingFactor(0.6);
    // move_group.setMaxAccelerationScalingFactor(0.3);

    // make_joint_array(joint_group_positions, 1.905, 0.846, 0.509, -4.522, -0.910, 0.058, false /*Passing rad*/);
    // move_group.setJointValueTarget(joint_group_positions);
    // move_group.plan(my_plan);
    // move_group.move();

    // make_joint_array(joint_group_positions, 1.133, 0.846, 0.509, -4.450, -2.076, 0.0485, false /*Passing rad*/);
    // move_group.setJointValueTarget(joint_group_positions);
    // move_group.plan(my_plan);
    // move_group.move();

    // make_joint_array(joint_group_positions, 1.905, 0.846, 0.509, -4.522, -0.910, 0.058, false /*Passing rad*/);
    // move_group.setJointValueTarget(joint_group_positions);
    // move_group.plan(my_plan);
    // move_group.move();

    // make_joint_array(joint_group_positions, 1.505, 0.845, 0.509, -4.521, -1.527, 0.058, false /*Passing rad*/);
    // move_group.setJointValueTarget(joint_group_positions);
    // move_group.plan(my_plan);
    // move_group.move();

    return BT::NodeStatus::SUCCESS;
}

ur_interfaces::msg::Segment _seg;
sensor_msgs::msg::Image _img;
sensor_msgs::msg::CameraInfo _info;

void FindObjCallback(const ur_interfaces::msg::Segimgcombo::SharedPtr msg)
{
    _seg = msg->seg;
    _img = msg->img;
}

void CamInfoCallback(const sensor_msgs::msg::CameraInfo msg)
{
    _info = msg;
}

cv::Point3d scalarMultiply(const cv::Point3d& pt3d, int scalar)
{
    return cv::Point3d(pt3d.x*scalar, pt3d.y*scalar, pt3d.z*scalar);
}

class FindObj : public BT::StatefulActionNode
{
    public:
    FindObj(const std::string& name) : BT::StatefulActionNode(name, {})
    {}

    BT::NodeStatus onStart()
    {
        RCLCPP_INFO(LOGGER, "Initialize node");
        combo_sub_ = Ros::instance()->node()->create_subscription<ur_interfaces::msg::Segimgcombo>("/detect_server/combo", 1, &FindObjCallback);
        cam_info_sub_ = Ros::instance()->node()->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", 1, &CamInfoCallback);
        img_pub_ = Ros::instance()->node()->create_publisher<sensor_msgs::msg::Image>("/segmentation_mask", 10);
        pc_pub_ = Ros::instance()->node()->create_publisher<sensor_msgs::msg::PointCloud2>("/segment_pointcloud", 10);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override 
    {
        //RCLCPP_INFO(LOGGER, "In Findobj tick!");
        try
        {
            //Create blank image with only segmentation points having pixel values
            cv_ptr_ = cv_bridge::toCvCopy(_img, _img.encoding);
            cv::Mat mask_cv = cv::Mat::zeros(cv::Size(cv_ptr_->image.cols,cv_ptr_->image.rows), CV_8UC1);
            for (auto [i, j] : _seg.pts)
            {
                mask_cv.at<uchar>(j, i) = 255;
            }
            //std::cout << _seg.pts.size() << std::endl;
            auto mask_img = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_cv).toImageMsg();
            //img_pub_->publish(*mask_img);

            //Point in polygon algorithm
            image_geometry::PinholeCameraModel cam_model;
            try
            {
                cam_model.fromCameraInfo(_info);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("pinhole_model"), "camera_info : %s", e.what());
            }
            

            cv::Mat test_img = cv::Mat::zeros(cv::Size(cv_ptr_->image.cols,cv_ptr_->image.rows), CV_8UC1);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam_xyz (new pcl::PointCloud<pcl::PointXYZ>());
            cloud_cam_xyz->height = 1;
            pcl::PCLPointCloud2::Ptr cloud_cam_pcl2 (new pcl::PCLPointCloud2 ());
            cv::Point3d pt3d_cv;
            cv::Point2d pt2d;
            pcl::PointXYZ pt3d_pcl;
            sensor_msgs::msg::PointCloud2 cloud_ros;
            bool ray = false;
            bool c_current = false;
            bool c_prev = false;
            int val;
            for (int row = 0; row < mask_cv.rows; ++row){
                ray = false;
                for (int col = 0; col < mask_cv.cols; ++col){
                    if(mask_cv.at<uchar>(row, col) == 255)
                    {
                        c_current = true; 
                    }
                    else{
                        c_current = false;
                    }
                    if ((c_current == true) & (c_prev == false))
                    {
                        ray = !ray;
                    }
                    if (ray)
                    {
                        test_img.at<uchar>(row, col) = 255;
                        val = cv_ptr_->image.at<uchar>(row, col);
                        pt2d.x = row;
                        pt2d.y = col;
                        pt3d_cv = cam_model.projectPixelTo3dRay(pt2d);
                        pt3d_cv = scalarMultiply(pt3d_cv, val);
                        pt3d_pcl.x = pt3d_cv.x;
                        pt3d_pcl.y = pt3d_cv.y;
                        pt3d_pcl.z = pt3d_cv.z;
                        cloud_cam_xyz->points.push_back(pt3d_pcl);
                        cloud_cam_xyz->width = cloud_cam_xyz->points.size();
                        pcl::toPCLPointCloud2(*cloud_cam_xyz, *cloud_cam_pcl2);
                        pcl_conversions::fromPCL(*cloud_cam_pcl2, cloud_ros);
                    }
                    if (c_current == true)
                    {
                        c_prev = true;
                    }
                    else
                    {
                        c_prev = false;
                    }
                }
            }
            auto test_img_ros = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", test_img).toImageMsg();
            img_pub_->publish(*test_img_ros);
            cloud_ros.header.frame_id = "camera_depth_optical_frame";
            pc_pub_->publish(cloud_ros);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("image_processing"), "cv_bridge : %s", e.what());
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted()
    {
        RCLCPP_INFO(LOGGER, "FindObj Halted");
    }
    
    private:
        cv_bridge::CvImagePtr cv_ptr_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
        rclcpp::Subscription<ur_interfaces::msg::Segimgcombo>::SharedPtr combo_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

};

BT::NodeStatus GoToObj()
{
    std::cout << "Going to object" << std::endl;

    return BT::NodeStatus::FAILURE;
}