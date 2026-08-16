#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    // move it groups of arm and gripper

    auto arm_links = moveit::planning_interface::MoveGroupInterface(node, "arm_links");
    arm_links.setMaxVelocityScalingFactor(0.5);
    arm_links.setMaxAccelerationScalingFactor(0.5);

    auto gripper_links = moveit::planning_interface::MoveGroupInterface(node, "gripper_links");

    arm_links.setStartStateToCurrentState();
    arm_links.setNamedTarget("Pose_1");

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (arm_links.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        arm_links.execute(plan1);
    }

    rclcpp::shutdown();
    return 0;
}