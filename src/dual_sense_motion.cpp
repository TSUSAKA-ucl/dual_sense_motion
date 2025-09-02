#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <Eigen/Dense>

using namespace std::chrono_literals;

const static unsigned ACC_INIT_SAMPLES = 500;

class ImuNode : public rclcpp::Node {
public:
  ImuNode(const std::string& device) 
    : Node("imu_node"), fd_(-1)
  {
    acc_init_count_ = 0;
    fix_offset_ = declare_parameter("fix_offset", true);
    openloop_ = declare_parameter("openloop", true);
    rotate_acc_ = declare_parameter("rotate_acc", true);
    scale_acc_ = declare_parameter("scale_acc", 4.0) * 9.81 / 32768.0;
    scale_gyro_ = declare_parameter("scale_gyro", 30.)*M_PI/180.0 / 32768.0;
    // 
    fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s", device.c_str());
      rclcpp::shutdown();
    }

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    reinitialize_acc_offset_ = this->create_service<std_srvs::srv::Empty>(
      "reinitialize_acc_offset",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
	     std::shared_ptr<std_srvs::srv::Empty::Response>)
      {
	this->acc_init_count_ = 0;
	this->acc_init_sum_ = Eigen::Vector3d::Zero();
      });
    reinitialize_orientation_ = this->create_service<std_srvs::srv::Empty>(
      "reinitialize_orientation",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
	     std::shared_ptr<std_srvs::srv::Empty::Response>)
      {
	x_.head<4>() << 1, 0, 0, 0;  // 単位四元数
      });
    // EKF 状態: [四元数 (4), 角速度バイアス(3)]
    x_.setZero();
    x_.head<4>() << 1, 0, 0, 0;  // 単位四元数

    timer_ = this->create_wall_timer(5ms, std::bind(&ImuNode::poll, this));
  }

private:
  void poll() {
    struct input_event ev;
    ssize_t n = read(fd_, &ev, sizeof(ev));
    while (n == sizeof(ev)) {
      if (ev.type == EV_ABS) {
	if (ev.code == ABS_X)  ax_ = ev.value * scale_acc_;
	if (ev.code == ABS_Y)  ay_ = ev.value * scale_acc_;
	if (ev.code == ABS_Z)  az_ = ev.value * scale_acc_;
	if (ev.code == ABS_RX) gx_ = ev.value * scale_gyro_;
	if (ev.code == ABS_RY) gy_ = ev.value * scale_gyro_;
	if (ev.code == ABS_RZ) gz_ = ev.value * scale_gyro_;
      }
      n = read(fd_, &ev, sizeof(ev));
    }
    if (fix_offset_) {
      if (acc_init_count_ < ACC_INIT_SAMPLES) {
	acc_init_count_++;
	acc_init_sum_ += Eigen::Vector3d(ax_, ay_, az_);
	// RCLCPP_INFO(this->get_logger(), "acc: %f, %f, %f",
	// 	    ax_, ay_, az_);
      } else if (acc_init_count_ == ACC_INIT_SAMPLES) {
	acc_offset_ = acc_init_sum_ / acc_init_count_;
	RCLCPP_INFO(this->get_logger(), "acc_init_count: %d", acc_init_count_);
	RCLCPP_INFO(this->get_logger(), "acc offset: %f, %f, %f",
		    acc_offset_.x(), acc_offset_.y(), acc_offset_.z());
	acc_init_count_++;  // 1回だけ実行するように
      }
    }
    // EKF predict/update
    update_filter();

    Eigen::Vector3d acc(ax_, ay_, az_);
    if (fix_offset_ && acc_init_count_ <= ACC_INIT_SAMPLES) {
      // 補正値がまだ確定していない場合は、とりあえず静止していると仮定
      acc = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    Eigen::Vector3d acc_world = Eigen::Quaterniond(x_(0), x_(1), x_(2), x_(3)) * acc;
    if (fix_offset_ && acc_init_count_ > ACC_INIT_SAMPLES) {
      acc_world -= acc_offset_;
    }
    // publish
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    if (rotate_acc_) {
      msg.linear_acceleration.x = acc_world.x();
      msg.linear_acceleration.y = acc_world.y();
      msg.linear_acceleration.z = acc_world.z();
    } else {
      msg.linear_acceleration.x = acc.x();
      msg.linear_acceleration.y = acc.y();
      msg.linear_acceleration.z = acc.z();
    }

    msg.angular_velocity.x = gx_;
    msg.angular_velocity.y = gy_;
    msg.angular_velocity.z = gz_;

    msg.orientation.w = x_(0);
    msg.orientation.x = x_(1);
    msg.orientation.y = x_(2);
    msg.orientation.z = x_(3);

    pub_->publish(msg);
  }

  void update_filter() {
    // 超簡易版: ジャイロ積分で四元数更新し、加速度で補正
    double dt = 0.005;  // 200Hz 仮定

    Eigen::Quaterniond q(x_(0), x_(1), x_(2), x_(3));
    Eigen::Vector3d omega(gx_, gy_, gz_);

    if (q.w() < 0) {
      q.coeffs() = -q.coeffs();  // 常に w >= 0 にする
    }
    // 四元数の微分
    Eigen::Quaterniond dq;
    dq.w() = 1.0;
    dq.vec() = 0.5 * omega * dt;
    q = (q * dq).normalized();

    if (! openloop_) {
      // 加速度（重力ベクトルに合わせて補正）
      Eigen::Vector3d acc(ax_, ay_, az_);
      acc.normalize();
      Eigen::Vector3d g_ref(0, 0, -1);
      Eigen::Quaterniond q_acc;
      Eigen::Vector3d v = acc.cross(g_ref);
      q_acc.w() = sqrt((acc.norm()*acc.norm())*(g_ref.norm()*g_ref.norm())) + acc.dot(g_ref);
      q_acc.vec() = v;
      q_acc.normalize();

      // フュージョン (単純に補間)
      q = q.slerp(0.02, q_acc);
    }

        x_(0) = q.w();
        x_(1) = q.x();
        x_(2) = q.y();
        x_(3) = q.z();
    }

    int fd_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reinitialize_acc_offset_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reinitialize_orientation_;
  bool openloop_ = false;
  bool fix_offset_ = false;
  bool rotate_acc_ = false;
    // raw data
    double ax_=0, ay_=0, az_=0;
    double gx_=0, gy_=0, gz_=0;

    // 状態ベクトル
    Eigen::VectorXd x_{7};

    // scale factors (適当に補正値を入れる)
    double scale_acc_ = 0.0;  // 例: ±4g レンジ
  double scale_gyro_ = 0.0; // 例: ±30 deg/s

  unsigned int acc_init_count_ = 0;
  Eigen::Vector3d acc_init_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_offset_ = Eigen::Vector3d::Zero();
};

int main(int argc, char *argv[]) {
  const char dual_sense_motion_dev[] = "/dev/input/by-id/usb-Sony_Interactive_Entertainment_DualSense_Wireless_Controller-event-if03";
  const char* dev_name = dual_sense_motion_dev;

  rclcpp::init(argc, argv);
  std::vector<std::string> args =
    rclcpp::remove_ros_arguments(argc, argv);

  if (args.size() >= 2) {
      dev_name = args[1].c_str();
  }
  auto node = std::make_shared<ImuNode>(dev_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
