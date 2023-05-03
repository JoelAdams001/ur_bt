#ifndef NODE_HPP
#define NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>

#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("image_subscriber"), __VA_ARGS__)

class Ros {
public:
    static Ros *instance(void) { return s_self; }
    Ros(int argc, char *argv[], const std::string &node_name);
    ~Ros();
    // Execute ROS
    void spin(void);
    void spinOnBackground(void); // Multithreading
    void shutdown(void);
    // No
    rclcpp::Node::SharedPtr node(void) { return m_node; }
private:
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    rclcpp::Node::SharedPtr m_node;
    // Instance
    static Ros *s_self;
};

#endif // NODE_H