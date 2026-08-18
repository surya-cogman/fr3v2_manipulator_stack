#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <fr3_interfaces/msg/pose_target_move_it.hpp>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PoseTargetMoveIt = fr3_interfaces::msg::PoseTargetMoveIt;
class PoseTarget
{

public:
    PoseTarget(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm_links");
        arm_->setMaxAccelerationScalingFactor(1.0);
        arm_->setMaxVelocityScalingFactor(1.0);

        pose_sub_ = node_->create_subscription<PoseTargetMoveIt>(
            "/pose_target",10,
            std::bind(&PoseTarget::PoseCallback,this,std::placeholders::_1)
        );

        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PoseTarget::checkMessage, this)
        );
        received_msg_ = false;
    }



    void goToPoseTarget(double x, double y, double z,
                        double roll, double pitch, double yaw)
    {

        arm_->setStartStateToCurrentState();

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();
        arm_->setPoseTarget(target_pose);
        planAndExecute(arm_);
    }

    void checkMessage()
    {
        if (received_msg_)
        {
            goToPoseTarget(
                x_,
                y_,
                z_,
                roll_,
                pitch_,
                yaw_);

            received_msg_ = false;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    rclcpp::Subscription<PoseTargetMoveIt>::SharedPtr pose_sub_;

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;

        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            interface->execute(plan);
        }
    }

    // subscriber pose params
    bool received_msg_;
    double x_;
    double y_;
    double z_;
    double roll_;
    double pitch_;
    double yaw_;

    void PoseCallback(const PoseTargetMoveIt::SharedPtr msg)
    {
        x_ = msg->x;
        y_ = msg->y;
        z_ = msg->z;

        roll_ = msg->roll;
        pitch_ = msg->pitch;
        yaw_ = msg->yaw;

        received_msg_ = true;
    }

    // Timer

    rclcpp::TimerBase::SharedPtr timer_;



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pose_target");
    auto pose_target = PoseTarget(node);
    // pose_target.goToPoseTarget(
    //     0.4, 0.0, 0.4,
    //     0.0, 0.0, 0.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}