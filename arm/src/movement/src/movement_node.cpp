#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <typeinfo>


#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <messages/msg/key_state.hpp>

rclcpp::Node::SharedPtr nodeHandle;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32<std::allocator<void> >, std::allocator<void> > > talon1SpeeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32<std::allocator<void> >, std::allocator<void> > > talon2SpeeedPublisher;


void talon1SpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
    talon1SpeeedPublisher->publish(speed);
}

void talon2SpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
    talon2SpeeedPublisher->publish(speed);
}

void 

int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("movement");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting movement");

    talon1SpeeedPublisher=nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_1_speed",1);
    talon2SpeeedPublisher=nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_2_speed",1);

    auto linear1SpeedSuscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("linear_1_speed",1,talon1SpeedCallback);
    auto linear2SpeedSuscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("linear_2_speed",1,talon2SpeedCallback);

    RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");
	rclcpp::Rate rate(30);

    while(rclcpp::ok()){

        rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}

