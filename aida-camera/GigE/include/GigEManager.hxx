#ifndef __GIGE_MANAGER__
#define __GIGE_MANAGER__

// STL LIBRARIES
#include <chrono>
using namespace std::chrono_literals;
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <fstream>
#include <sstream>

// ROS2 LIBRARIES
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

// OTHER LIBRARIES
#include "Camera.hxx"

class GigE : public rclcpp::Node
{
    public:
        GigE(std::string node_name);
        ~GigE();
        // Setter
        void sleep(unsigned);
        void loopForEver();
        void setPublisher();
        void warmUp();
        void getFrame();
        void publishFrame();
        void operate();

        // Getter
        auto getCameraInfo() -> void;
        auto getHeader() -> std_msgs::msg::Header;

        // Extras
        void log(std::string);
        void log_err(std::string);


    private:
        Camera cam_;
        rclcpp::Node::SharedPtr node_;
        
        std::shared_ptr<image_transport::ImageTransport> image_transport_;
        image_transport::CameraPublisher image_transport_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::Image::SharedPtr frame_;

        sensor_msgs::msg::CameraInfo camera_info_;

        bool port_opened_;
        std::string frame_id_;
        std_msgs::msg::Header header_;
        int queue_size_;

 
};

#endif