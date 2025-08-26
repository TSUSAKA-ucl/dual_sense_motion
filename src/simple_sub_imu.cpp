#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Geometry>

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
	const Eigen::Quaterniond q(msg->orientation.w,
				   msg->orientation.x,
				   msg->orientation.y,
				   msg->orientation.z);
	const Eigen::Vector3d x_axis(1, 0, 0);
	const Eigen::Vector3d x_rotated = q * x_axis;
	const Eigen::Vector3d y_axis(0, 1, 0);
	const Eigen::Vector3d y_rotated = q * y_axis;
	// RCLCPP_INFO(this->get_logger(), "I heard: orientation=(%3.2f, %3.2f, %3.2f, %3.2f), x_rotated=(%5.2f,%5.2f,%5.2f), y_rotated=(%5.2f,%5.2f,%5.2f)",
	// 	    msg->orientation.w,
	// 	    msg->orientation.x,
	// 	    msg->orientation.y,
	// 	    msg->orientation.z,
	// 	    x_rotated.x(), x_rotated.y(), x_rotated.z(),
	// 	    y_rotated.x(), y_rotated.y(), y_rotated.z());
	RCLCPP_INFO(this->get_logger(), "I heard: angular_velocity=(%5.2f,%5.2f,%5.2f), linear_acceleration=(%5.2f,%5.2f,%5.2f), x_rotated=(%5.2f,%5.2f,%5.2f), y_rotated=(%5.2f,%5.2f,%5.2f)",
		    	    msg->angular_velocity.x,
		    	    msg->angular_velocity.y,
		    	    msg->angular_velocity.z,
		    	    msg->linear_acceleration.x,
		    	    msg->linear_acceleration.y,
		    	    msg->linear_acceleration.z,
		    	    x_rotated.x(), x_rotated.y(), x_rotated.z(),
		    	    y_rotated.x(), y_rotated.y(), y_rotated.z());
      };
    subscription_ =
      this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, topic_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
