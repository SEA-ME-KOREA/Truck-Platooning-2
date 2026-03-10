// FrontCamera.cpp
#include "FrontCamera.hpp"
#include <algorithm>
#include <cctype>

namespace {
std::string sanitize_numeric_attr(std::string value) {
    value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char c) {
        return std::isspace(c);
    }), value.end());
    if (!value.empty() && (value.back() == 'f' || value.back() == 'F')) {
        value.pop_back();
    }
    return value;
}
}  // namespace

FrontCameraPublisher::FrontCameraPublisher(boost::shared_ptr<carla::client::Actor> actor)
    : Node("front_camera_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)) {

    // 공통으로 사용할 QoS 프로파일
    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.best_effort();

// ==========================
// 기존 RGB 카메라 설정 (원본 유지)
// ==========================
this->get_parameter_or("rgbcam/x",rgbcam_x,2.0f);
this->get_parameter_or("rgbcam/y",rgbcam_y,0.0f);
this->get_parameter_or("rgbcam/z",rgbcam_z,3.5f);
this->get_parameter_or("rgbcam/pitch",rgbcam_pitch, -15.0f);
this->get_parameter_or("rgbcam/yaw",rgbcam_yaw,0.0f);
this->get_parameter_or("rgbcam/roll",rgbcam_roll,0.0f);
this->get_parameter_or("rgbcam/sensor_tick",rgbcam_sensor_tick,std::string("0.033333f"));
this->get_parameter_or("rgbcam/image_size_x",rgbcam_image_size_x,std::string("640"));
this->get_parameter_or("rgbcam/image_size_y",rgbcam_image_size_y,std::string("480"));
this->get_parameter_or("rgbcam/fov",rgbcam_fov,std::string("90.0f"));
this->get_parameter_or("rgbcam_topic_name",rgbcam_topic_name,std::string("carla/image_raw"));

rgbcam_sensor_tick = sanitize_numeric_attr(rgbcam_sensor_tick);
rgbcam_fov = sanitize_numeric_attr(rgbcam_fov);

publisher_ = this->create_publisher<sensor_msgs::msg::Image>(rgbcam_topic_name, custom_qos);

camera_bp = boost::shared_ptr<carla::client::ActorBlueprint>(const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.camera.rgb")));
assert(camera_bp != nullptr);
camera_bp->SetAttribute("sensor_tick", rgbcam_sensor_tick);
camera_bp->SetAttribute("image_size_x", rgbcam_image_size_x);
camera_bp->SetAttribute("image_size_y", rgbcam_image_size_y);
camera_bp->SetAttribute("fov", rgbcam_fov);

camera_transform = cg::Transform{ cg::Location{rgbcam_x, rgbcam_y, rgbcam_z}, cg::Rotation{rgbcam_pitch, rgbcam_yaw, rgbcam_roll}}; // pitch, yaw, roll.
cam_actor = world->SpawnActor(*camera_bp, camera_transform, actor.get());
camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);
camera->Listen([this](auto data) {
    auto image = boost::static_pointer_cast<csd::Image>(data);
    assert(image != nullptr);
    publishImage(*image);
});

// =========================================================
// ✨ 추가: Semantic Segmentation 카메라 설정 및 생성
// =========================================================
// config.yaml에 sscam 네임스페이스로 추가한 파라미터를 읽습니다.
this->get_parameter_or("sscam/x", sscam_x, 2.3f);
this->get_parameter_or("sscam/y", sscam_y, 0.0f);
this->get_parameter_or("sscam/z", sscam_z, 7.0f);
this->get_parameter_or("sscam/pitch", sscam_pitch, -35.0f);
this->get_parameter_or("sscam/yaw", sscam_yaw, 0.0f);
this->get_parameter_or("sscam/roll", sscam_roll, 0.0f);
this->get_parameter_or("sscam/sensor_tick", sscam_sensor_tick, std::string("0.033333f"));
this->get_parameter_or("sscam/image_size_x", sscam_image_size_x, std::string("640"));
this->get_parameter_or("sscam/image_size_y", sscam_image_size_y, std::string("480"));
this->get_parameter_or("sscam/fov", sscam_fov, std::string("90.0f"));
this->get_parameter_or("sscam_topic_name", sscam_topic_name, std::string("front_camera_ss"));

sscam_sensor_tick = sanitize_numeric_attr(sscam_sensor_tick);
sscam_fov = sanitize_numeric_attr(sscam_fov);

// SS 퍼블리셔 생성
ss_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(sscam_topic_name, custom_qos);

// SS 블루프린트 찾기
ss_camera_bp = boost::shared_ptr<carla::client::ActorBlueprint>(const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.camera.semantic_segmentation")));
if (!ss_camera_bp) {
    RCLCPP_ERROR(this->get_logger(), "[FrontCameraPublisher] Failed to find sensor.camera.semantic_segmentation blueprint. Semantic camera will not be spawned.");
} else {
    ss_camera_bp->SetAttribute("sensor_tick", sscam_sensor_tick);
    ss_camera_bp->SetAttribute("image_size_x", sscam_image_size_x);
    ss_camera_bp->SetAttribute("image_size_y", sscam_image_size_y);
    ss_camera_bp->SetAttribute("fov", sscam_fov);

    // SS 카메라 변환 (RGB와 동일 위치를 기본으로 권장)
    ss_camera_transform = cg::Transform{
        cg::Location{sscam_x, sscam_y, sscam_z},
        cg::Rotation{sscam_pitch, sscam_yaw, sscam_roll}
    };

    try {
        ss_cam_actor = world->SpawnActor(*ss_camera_bp, ss_camera_transform, actor.get());
        ss_camera = boost::static_pointer_cast<cc::Sensor>(ss_cam_actor);

        // SS 카메라 Listen
        ss_camera->Listen([this](auto data) {
            auto image = boost::static_pointer_cast<csd::Image>(data);
            if (!image) { return; }
            publishSemanticImage(*image);
        });
        RCLCPP_INFO(this->get_logger(), "[FrontCameraPublisher] Semantic Segmentation camera spawned and listening on topic '%s'", sscam_topic_name.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[FrontCameraPublisher] Failed to spawn semantic camera: %s", e.what());
    }
}
}

void FrontCameraPublisher::publishImage(const csd::Image &carla_image) {
auto msg = std::make_unique<sensor_msgs::msg::Image>();
// Set the header
msg->header.stamp = this->now();
msg->header.frame_id = "1"; // Set appropriate frame ID


// Set image properties
msg->height = carla_image.GetHeight();
msg->width = carla_image.GetWidth();
msg->encoding = "rgb8"; // Assuming the image is in RGB8 format
msg->is_bigendian = false;
msg->step = carla_image.GetWidth() * 3; // 3 bytes per pixel for RGB8 encoding

// Allocate memory for ROS message data
msg->data.resize(msg->step * msg->height);

// Copy image data
const auto* raw_data = reinterpret_cast<const uint8_t*>(carla_image.data());
for (int i = 0; i < static_cast<int>(msg->height * msg->width); ++i) {
    // Skip alpha channel by offsetting the raw data index
    int raw_index = i * 4; // 4 bytes per pixel (BGRA)
    int msg_index = i * 3; // 3 bytes per pixel (RGB)
    msg->data[msg_index]     = raw_data[raw_index + 2]; // Red (BGR -> RGB)
    msg->data[msg_index + 1] = raw_data[raw_index + 1]; // Green
    msg->data[msg_index + 2] = raw_data[raw_index + 0]; // Blue
}

// Publish the message
publisher_->publish(*msg);
}

// ✨ 추가: Semantic Segmentation 이미지 발행 함수
void FrontCameraPublisher::publishSemanticImage(const csd::Image &carla_image) {
// Semantic 카메라는 태그를 R 채널에 담아 보냅니다.
// mono8로 R 채널만 추출하여 퍼블리시합니다.
auto msg = std::make_unique<sensor_msgs::msg::Image>();


msg->header.stamp = this->now();
msg->header.frame_id = "1";
msg->height = carla_image.GetHeight();
msg->width  = carla_image.GetWidth();
msg->encoding = "mono8";
msg->is_bigendian = false;
msg->step = carla_image.GetWidth();

msg->data.resize(static_cast<size_t>(msg->step) * msg->height);

const auto* raw_data = reinterpret_cast<const uint8_t*>(carla_image.data());
// BGRA에서 R 채널은 +2 오프셋
const int pixels = static_cast<int>(msg->height * msg->width);
for (int i = 0; i < pixels; ++i) {
    msg->data[i] = raw_data[i * 4 + 2];
}

ss_publisher_->publish(*msg);
}
