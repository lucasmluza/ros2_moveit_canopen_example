#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

using std::placeholders::_1;

class CANopenJointStatePublisher : public rclcpp::Node
{
public:
    CANopenJointStatePublisher() : Node("canopen_joint_state_publisher")
    {

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SystemDefaultsQoS());

        std::string urdf_xml = this->declare_parameter("robot_description", std::string(""));

        if (urdf_xml.empty())
        {
            throw std::runtime_error("robot_description parameter must not be empty");
        }

        urdf::Model model;

        // Initialize the model
        if (!model.initString(urdf_xml))
        {
            throw std::runtime_error("Unable to initialize urdf::model from robot description");
        }

        // Initialize the KDL tree
        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(model, tree))
        {
            throw std::runtime_error("Failed to extract kdl tree from robot description");
        }

        for (auto const &joint : model.joints_)
        {
            RCLCPP_INFO(this->get_logger(), "Get joint: '%s'", joint.first.c_str());
            joint_names_.push_back(joint.first.c_str());
            joint_positions_.push_back(0.0);

            subscriptions_.push_back(this->create_subscription<sensor_msgs::msg::JointState>(
                "/" + joint.first + "/joint_states",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&CANopenJointStatePublisher::topic_callback, this, _1)));
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // ROS2 CANopen publishes a <joint_name>/joint_states with a vector of only one element
        if (msg->name.size() == 1)
        {
            // Search index that is associated with the joint name
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[0]);
            if (it != joint_names_.end())
            {
                int index = it - joint_names_.begin();
                if (joint_positions_.at(index) != msg->position[0])
                {
                    joint_positions_.at(index) = msg->position[0];
                }
            }
        }

        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = now();
        message.name = joint_names_;
        message.position = joint_positions_;
        publisher_->publish(message);
    }

    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subscriptions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANopenJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}