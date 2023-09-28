#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <sl/Camera.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ZedCameraDriver : public rclcpp::Node
{
  public:
    ZedCameraDriver(): Node("zed_camera_driver")
    {
      this->declare_parameter("frequency", 30);

      left_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/zed_camera/left/image", 10);
      right_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/zed_camera/right/image", 10);
      pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/zed_camera/pointcloud", 10);
      
      timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(1.0 / this->get_parameter("frequency").as_double()), \
      std::bind(&ZedCameraDriver::callback, this));

      zed_.open();
    }
    
    ~ZedCameraDriver()
    {
      zed_.close();
    }

  private:
    void callback()
    {
      sensor_msgs::msg::Image left_img_msg = sensor_msgs::msg::Image();
      sensor_msgs::msg::Image right_img_msg = sensor_msgs::msg::Image();
      sensor_msgs::msg::PointCloud2 pointcloud_msg = sensor_msgs::msg::PointCloud2();

      left_img_pub_->publish(left_img_msg);
      right_img_pub_->publish(right_img_msg);
      pointcloud_pub_->publish(pointcloud_msg);
    }

    sl::Camera zed_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedCameraDriver>());
  rclcpp::shutdown();
  return 0;
}