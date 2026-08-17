#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class NamedTarget
{
public:
    NamedTarget(std::shared_ptr<rclcpp::Node> node)
    {

        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm_links");
        arm_->setMaxAccelerationScalingFactor(1.0);
        arm_->setMaxVelocityScalingFactor(1.0);
    }

    // Move from home to Pose1
    void goToNamedTarget(const std::string &name){
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(name);

            PlanAndExecute(arm_);
    }

    //  Move through sequences

    void motionSeq(){
        goToNamedTarget("Pose_1");
        goToNamedTarget("Pose_2");
        goToNamedTarget("Home_Pose");
    }
    

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;

    void PlanAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {

        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            interface->execute(plan);
        }
    }
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("named_target");
    auto named_target = NamedTarget(node);
    named_target.motionSeq(); // add your required pose
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}