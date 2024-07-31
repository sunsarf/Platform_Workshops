#include "translator_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

HelloWorldTranslator::HelloWorldTranslator(): Node("hello_world_translator")
{
    RCLCPP_INFO(this->get_logger(), "Hello World Translator has been started!");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter",
        10,
        std::bind(&HelloWorldTranslator::translate, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("/charla", 10);
}

HelloWorldTranslator::~HelloWorldTranslator()
{
}

void HelloWorldTranslator::translate(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data.find("Hello, World!:") == std::string::npos){
        RCLCPP_WARN(this->get_logger(), "Cannot translate: %s", output_msg.data.c_str());
        return;
    }
    else{
        output_msg.data = "Hola, Mundo!: " + msg->data.substr(14);
        publisher_->publish(output_msg);
    }
    return;
}