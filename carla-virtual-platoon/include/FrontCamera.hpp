#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>

class FrontCameraPublisher : public rclcpp::Node {
public:
    FrontCameraPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~FrontCameraPublisher() {
        // 두 액터가 모두 유효할 때만 Destroy 호출
        if (cam_actor && cam_actor->IsAlive()) {
            cam_actor->Destroy();
        }
        if (ss_cam_actor && ss_cam_actor->IsAlive()) {
            ss_cam_actor->Destroy();
        }
    }

private:
    // --- 기존 RGB 카메라를 위한 함수 ---
    void publishImage(const csd::Image &carla_image);

    // ✨ --- SS 카메라를 위한 함수 선언 추가 ---
    void publishSemanticImage(const csd::Image &carla_image);
    void publishImage1(const csd::Image &carla_image);

    // --- 기존 RGB 카메라 멤버 변수 ---
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    boost::shared_ptr<carla::client::Sensor> camera;
    boost::shared_ptr<carla::client::Actor> cam_actor;
    carla::geom::Transform camera_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> camera_bp;
    float rgbcam_x;
    float rgbcam_y;
    float rgbcam_z;
    float rgbcam_pitch;
    float rgbcam_yaw;
    float rgbcam_roll;
    std::string rgbcam_sensor_tick;
    std::string rgbcam_topic_name;
    std::string rgbcam_image_size_x;
    std::string rgbcam_image_size_y;
    std::string rgbcam_fov;
    
    // ✨ --- SS 카메라를 위한 멤버 변수 선언 추가 ---
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ss_publisher_;
    boost::shared_ptr<carla::client::Sensor> ss_camera;
    boost::shared_ptr<carla::client::Actor> ss_cam_actor;
    carla::geom::Transform ss_camera_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> ss_camera_bp;
    float sscam_x;
    float sscam_y;
    float sscam_z;
    float sscam_pitch;
    float sscam_yaw;
    float sscam_roll;
    std::string sscam_sensor_tick;
    std::string sscam_topic_name;
    std::string sscam_image_size_x;
    std::string sscam_image_size_y;
    std::string sscam_fov;

    // role_name_은 공통으로 사용 가능하므로 그대로 둡니다.
    std::string role_name_; 
};

