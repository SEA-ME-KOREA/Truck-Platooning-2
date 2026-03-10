#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>

class TruckControl : public rclcpp::Node {

public:
    TruckControl(boost::shared_ptr<carla::client::Vehicle> vehicle_);

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ThrottleSubscriber_;
    void SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void ThrottleSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    carla::rpc::VehicleControl control;
    std::string steer_topic_name;
    std::string throttle_topic_name;
    bool initial_hand_brake;
    std::chrono::steady_clock::time_point brake_start_time;
    bool braking_timer_started = false;
    const float brake_to_handbrake_threshold = 2.0f; // 2초 이상이면 handbrake 적용

};