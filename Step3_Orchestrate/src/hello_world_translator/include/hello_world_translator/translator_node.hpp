#ifndef HELLOWORLD_HPP
#define HELLOWORLD_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloWorldTranslator : public rclcpp::Node
{   
    public:
        HelloWorldTranslator();
        ~HelloWorldTranslator();
        void translate(const std_msgs::msg::String::SharedPtr msg);
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std_msgs::msg::String output_msg;
};

#endif // HELLOWORLD_HPP