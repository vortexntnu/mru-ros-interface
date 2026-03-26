#ifndef MRU_ROS_INTERFACE__MRU_ROS_INTERFACE_HPP_
#define MRU_ROS_INTERFACE__MRU_ROS_INTERFACE_HPP_

#include <kongsberg_mru_driver.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <thread>

namespace mru::ros_interface {

class MruRosInterface : public rclcpp::Node {
   public:
    explicit MruRosInterface(const rclcpp::NodeOptions& options);

    ~MruRosInterface() {
        mru_driver_->stop_udp_communication();
        io_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

   private:
    void mrubin_callback(const MrubinMessage& msg);

    void declare_ros_parameters();

    void create_publishers();

    void create_driver();

    void setup_mru_connection();

    void configure_mru();

    void start_mru_stream();

    asio::io_context io_;
    std::thread io_thread_;

    std::unique_ptr<KongsbergMRUDriver> mru_driver_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::string frame_id_;
};

}  // namespace mru::ros_interface

#endif  // MRU_ROS_INTERFACE__MRU_ROS_INTERFACE_HPP_
