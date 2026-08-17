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

    // normla planner

    // arm_links.setStartStateToCurrentState();
    // arm_links.setNamedTarget("Pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success = (arm_links.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success)
    // {
    //     arm_links.execute(plan1);
    // }

    // ===============================================================================================

    // Joint planner:

    // std::vector<double> joints = {0.0, 0.0, 0.0, -0.35, 0.0, 1.57, 1.57};

    // arm_links.setStartStateToCurrentState();
    // arm_links.setJointValueTarget(joints);
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success = (arm_links.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success){
    //     arm_links.execute(plan1);
    // }

    // ===============================================================================================

    // Pose goal:

    // geometry_msgs::msg::PoseStamped target_pose;

    // target_pose.header.frame_id = "base_link";
    // target_pose.pose.position.x = 0.40;
    // target_pose.pose.position.y = -0.3;
    // target_pose.pose.position.z = 0.50;

    // target_pose.pose.orientation.x = 1.0;
    // target_pose.pose.orientation.y = 0.0;
    // target_pose.pose.orientation.z = 0.0;
    // target_pose.pose.orientation.w = 0.0;

    // arm_links.setStartStateToCurrentState();
    // arm_links.setPoseTarget(target_pose);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success = (arm_links.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success)
    // {
    //     arm_links.execute(plan1);
    // }


    // Cartesian Path

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm_links.getCurrentPose().pose;
    pose1.position.x += 0.15;
    waypoints.push_back(pose1);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm_links.computeCartesianPath(waypoints,0.01,trajectory);

    if (fraction == 1){
        arm_links.execute(trajectory);
    }




    rclcpp::shutdown();
    return 0;
}