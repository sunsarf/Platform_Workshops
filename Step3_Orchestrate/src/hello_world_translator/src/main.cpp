#include "rclcpp/rclcpp.hpp"
#include "translator_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldTranslator>());
    rclcpp::shutdown();
    return 0;
}