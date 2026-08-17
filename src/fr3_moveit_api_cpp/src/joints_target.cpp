#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <fr3_interfaces/msg/joint_cmd_move_it.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using JointCmdMoveIt = fr3_interfaces::msg::JointCmdMoveIt;
class JointTarget
{

public:
    JointTarget(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm_links");
        arm_->setMaxAccelerationScalingFactor(0.5);
        arm_->setMaxVelocityScalingFactor(0.5);

        joint_cmd_sub_ = node_->create_subscription<JointCmdMoveIt>(
            "joint_cmd_moveit", 10,
            std::bind(&JointTarget::jointCallback, this, std::placeholders::_1));
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    rclcpp::Subscription<JointCmdMoveIt>::SharedPtr joint_cmd_sub_;

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            interface->execute(plan);
        }
    }

    

    void jointCallback(const JointCmdMoveIt &msg)
    {
        if (msg.arm_joints.size() == 7)
        {
            goToJointTarget(msg.arm_joints);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joints_target");
    auto joints_target = JointTarget(node);
    // joints_target.goToJointTarget({0.0, 0.0, 0.0, -2.34, 0.0, 3.14, 0.0});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}