#include "lidardrive/jetson.hpp"
Sub::Sub() : Node("dxlsub")
{
    if(!dxl.open())
    {
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        //return -1;
    } 
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(&Sub::mysub_callback, this, dxl, _1);
    sub_ = this->create_subscription<std_msgs::msg::Int32>("err", rclcpp::SensorDataQoS(), fn);
}
void Sub::mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg)
{
    int chk = 0;
    err = intmsg->data;
    if(err == 100){
        lvel = 100;//왼쪽 바퀴 속도
	    rvel = 100;//오른쪽 바퀴 속도
    }
    else  if(err == 200){
        lvel = 100;//왼쪽 바퀴 속도
	    rvel = 100;//오른쪽 바퀴 속도
    }
    else  if(err == 300){
        lvel = -100;//왼쪽 바퀴 속도
	    rvel = -100;//오른쪽 바퀴 속도
    }
    else  if(err == 400){
        lvel = 100;//왼쪽 바퀴 속도
	    rvel = 100;//오른쪽 바퀴 속도
    }
    else  if(err == 500){
        lvel = -100;//왼쪽 바퀴 속도
	    rvel = -100;//오른쪽 바퀴 속도
    }
    else  if(err == 600){
        lvel = 100;//왼쪽 바퀴 속도
	    rvel = 100;//오른쪽 바퀴 속도
    }
    else  if(err == 700){
        lvel = -100;//왼쪽 바퀴 속도
	    rvel = -100;//오른쪽 바퀴 속도
    }
    else {
        lvel = 100 - gain * -err;//왼쪽 바퀴 속도
	    rvel = -(102 + gain * -err);//오른쪽 바퀴 속도
    }

    RCLCPP_INFO(this->get_logger(), "Received message: %d %d", lvel, rvel);
    mdxl.setVelocity(lvel, rvel);
}
