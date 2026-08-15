#include <rclcpp/rclcpp.hpp>

int main(int argc,char **argv){

    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);



    rclcpp::shutdown();
    return 0;
}