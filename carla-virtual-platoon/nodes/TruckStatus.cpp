#include "TruckStatus.hpp"

namespace {
geometry_msgs::msg::Quaternion quaternionFromEulerDeg(float roll_deg, float pitch_deg, float yaw_deg) {
    constexpr double kPi = 3.14159265358979323846;
    const double roll = static_cast<double>(roll_deg) * kPi / 180.0;
    const double pitch = static_cast<double>(pitch_deg) * kPi / 180.0;
    const double yaw = static_cast<double>(yaw_deg) * kPi / 180.0;

    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}
}  // namespace

TruckStatusPublisher::TruckStatusPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_)
    : Node("truck_status_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),Vehicle_(vehicle_) {

    this->get_parameter_or("info_topic_name",info_topic_name,std::string("velocity_info"));
    AccelPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("accel",1);
    VelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>(info_topic_name,1);
    PosePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose3d", 1);
    PitchPublisher_ = this->create_publisher<std_msgs::msg::Float32>("pitch_deg", 1);
    timer_1ms_accel = this->create_wall_timer(1ms, std::bind(&TruckStatusPublisher::TruckStatusPublisher_accel_callback, this));
    timer_1ms_velocity = this->create_wall_timer(1ms, std::bind(&TruckStatusPublisher::TruckStatusPublisher_velocity_callback, this));
    timer_50ms_pose = this->create_wall_timer(50ms, std::bind(&TruckStatusPublisher::TruckStatusPublisher_pose_callback, this));
    ShutdownSubscriber = this->create_subscription<std_msgs::msg::String>("/shutdown_topic", 10, std::bind(&TruckStatusPublisher::shutdown_callback, this, std::placeholders::_1));
    DistanceSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("min_distance", 10, std::bind(&TruckStatusPublisher::DistanceSubCallback, this, std::placeholders::_1));

    gettimeofday(&init_, NULL);
    timer_100ms_record = this->create_wall_timer(100ms, std::bind(&TruckStatusPublisher::TruckStatus_record_callback, this));
}

void TruckStatusPublisher::TruckStatusPublisher_accel_callback() {
    acc_ = Vehicle_->GetAcceleration();
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.push_back(acc_.x);
    message.data.push_back(acc_.y);
    message.data.push_back(acc_.z);
    float result_acc = acc_.x; // m/s
    acceleration_ = result_acc;
    AccelPublisher_->publish(message);
}


void TruckStatusPublisher::TruckStatusPublisher_velocity_callback() {
    vel_ = Vehicle_->GetVelocity();
    auto message = std_msgs::msg::Float32();
    float result_vel = std::sqrt(std::pow(vel_.x,2)  + std::pow(vel_.y,2) + std::pow(vel_.z,2)); // m/s
    message.data = result_vel;
    velocity_ = result_vel;
    VelocityPublisher_->publish(message);
}

void TruckStatusPublisher::TruckStatusPublisher_pose_callback() {
    const auto tf = Vehicle_->GetTransform();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = tf.location.x;
    pose_msg.pose.position.y = tf.location.y;
    pose_msg.pose.position.z = tf.location.z;
    pose_msg.pose.orientation =
        quaternionFromEulerDeg(tf.rotation.roll, tf.rotation.pitch, tf.rotation.yaw);
    PosePublisher_->publish(pose_msg);

    std_msgs::msg::Float32 pitch_msg;
    pitch_msg.data = tf.rotation.pitch;
    PitchPublisher_->publish(pitch_msg);
}

void TruckStatusPublisher::DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    distance_ = msg->data;
}

void TruckStatusPublisher::shutdown_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received shutdown message: '%s'", msg->data.c_str());
    rclcpp::shutdown(); // 종료 명령
}

void TruckStatusPublisher::TruckStatus_record_callback() {
    if(velocity_ != 0.) recordData(init_);
}

void TruckStatusPublisher::recordData(struct timeval startTime){
    struct timeval currentTime;
    char file_name[] = "SCT_log00.csv";
    static char file[128] = {0x00, };
    char buf[256] = {0x00,};
    static bool flag = false;
    double diff_time;
    log_path_ = "/home/nvidia/platoon_ws/logfiles/";
    std::ifstream read_file;
    std::ofstream write_file;
    if(!flag){
      for(int i = 0; i < 100; i++){
        file_name[7] = i/10 + '0';  //ASCII
        file_name[8] = i%10 + '0';
        sprintf(file, "%s%s", log_path_.c_str(), file_name);
        read_file.open(file);
        if(read_file.fail()){  //Check if the file exists
          read_file.close();
          write_file.open(file);
          break;
        }
        read_file.close();
      }
      write_file << "Time, Velocity, Acceleration, Distance" << std::endl; 
      flag = true;
    }
    if(flag){
  //    std::scoped_lock lock(lane_mutex_, rlane_mutex_, vel_mutex_, dist_mutex_, rep_mutex_);
      gettimeofday(&currentTime, NULL);
      diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
      sprintf(buf, "%.10e, %.3f, %.3f, %.3f", diff_time, velocity_, acceleration_, distance_);
      write_file.open(file, std::ios::out | std::ios::app);
      write_file << buf << std::endl;
    }
    write_file.close();
}
