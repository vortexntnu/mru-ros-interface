#include <cmath>
#include <mru_ros_interface/mru_ros_interface.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mru::ros_interface {

MruRosInterface::MruRosInterface(const rclcpp::NodeOptions& options)
    : Node("mru_ros_interface", options) {
    declare_ros_parameters();
    create_publishers();
    create_driver();
    setup_mru_connection();
    configure_mru();
    start_mru_stream();
}

void MruRosInterface::declare_ros_parameters() {
    declare_parameter<std::string>("imu_pub_topic");
    frame_id_ = declare_parameter<std::string>("frame_id");

    declare_parameter<std::string>("connection_params.remote_ip");
    declare_parameter<int>("connection_params.data_remote_port");
    declare_parameter<int>("connection_params.data_local_port");
    declare_parameter<int>("connection_params.control_local_port");

    declare_parameter<std::string>("mru_settings.channel");
    declare_parameter<int>("mru_settings.port");
    declare_parameter<std::string>("mru_settings.ip_addr");
    declare_parameter<std::string>("mru_settings.format");
    declare_parameter<int>("mru_settings.interval");
    declare_parameter<int>("mru_settings.token");
}

void MruRosInterface::create_publishers() {
    std::string imu_pub_topic = get_parameter("imu_pub_topic").as_string();
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        imu_pub_topic, rclcpp::QoS(1).reliable());
}

void MruRosInterface::create_driver() {
    mru_driver_ = std::make_unique<KongsbergMRUDriver>(
        io_, [this](const MrubinMessage& msg) { this->mrubin_callback(msg); });
}

void MruRosInterface::setup_mru_connection() {
    MruConnectionParams params;
    params.remote_ip = get_parameter("connection_params.remote_ip").as_string();
    params.data_remote_port = static_cast<uint16_t>(
        get_parameter("connection_params.data_remote_port").as_int());
    params.data_local_port = static_cast<uint16_t>(
        get_parameter("connection_params.data_local_port").as_int());
    params.control_local_port = static_cast<uint16_t>(
        get_parameter("connection_params.control_local_port").as_int());

    std::error_code ec = mru_driver_->open_udp_socket(params);
    if (ec) {
        RCLCPP_ERROR(get_logger(), "Failed to open MRU connection: %s",
                     ec.message().c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "MRU UDP connection opened.");
}

void MruRosInterface::configure_mru() {
    MruSettings settings;
    settings.channel = get_parameter("mru_settings.channel").as_string();
    settings.port =
        static_cast<uint32_t>(get_parameter("mru_settings.port").as_int());
    settings.ip_addr = get_parameter("mru_settings.ip_addr").as_string();
    settings.format = get_parameter("mru_settings.format").as_string();
    settings.interval =
        static_cast<uint32_t>(get_parameter("mru_settings.interval").as_int());
    settings.token =
        static_cast<uint8_t>(get_parameter("mru_settings.token").as_int());

    MruStatus status = mru_driver_->set_default_settings(settings);
    if (status != MruStatus::Ok) {
        RCLCPP_ERROR(get_logger(), "Failed to configure MRU: status %d",
                     static_cast<int>(status));
    } else {
        RCLCPP_INFO(get_logger(), "MRU configured successfully.");
    }
}

void MruRosInterface::start_mru_stream() {
    StatusCode sc = mru_driver_->start_read();
    if (sc != StatusCode::Ok) {
        RCLCPP_ERROR(get_logger(), "Failed to start MRU read: status %d",
                     static_cast<int>(sc));
        return;
    }
    io_thread_ = std::thread([this]() { io_.run(); });
    RCLCPP_INFO(get_logger(), "Started reading MRU data.");
}

void MruRosInterface::mrubin_callback(const MrubinMessage& msg) {
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.frame_id = frame_id_;
    imu_msg->header.stamp = this->get_clock()->now();

    // Convert euler angles (roll, pitch, yaw) to quaternion
    const double cr = std::cos(msg.roll * 0.5);
    const double sr = std::sin(msg.roll * 0.5);
    const double cp = std::cos(msg.pitch * 0.5);
    const double sp = std::sin(msg.pitch * 0.5);
    const double cy = std::cos(msg.yaw * 0.5);
    const double sy = std::sin(msg.yaw * 0.5);

    imu_msg->orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg->orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg->orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg->orientation.z = cr * cp * sy - sr * sp * cy;

    imu_msg->angular_velocity.x = msg.angle_rate_roll;
    imu_msg->angular_velocity.y = msg.angle_rate_pitch;
    imu_msg->angular_velocity.z = msg.angle_rate_yaw;

    imu_msg->linear_acceleration.x = msg.acceleration_roll_direction;
    imu_msg->linear_acceleration.y = msg.acceleration_pitch_direction;
    imu_msg->linear_acceleration.z = msg.acceleration_yaw_direction;

    imu_pub_->publish(std::move(imu_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MruRosInterface)

}  // namespace mru::ros_interface
