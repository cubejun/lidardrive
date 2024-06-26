#ifndef _JETSON_HPP_
#define _JETSON_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include "lidardrive/dxl.hpp"
#include <functional>
#include <memory>
#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;
class Sub : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
        void mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg);
        
        double gain = 1;//50rpm0.7;
    public:
        Sub();
        Dxl dxl;
        int lvel, rvel, err;
};
#endif //_PUB_HPP_

