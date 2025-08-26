#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node {
public:
    ImuNode(const std::string& device) 
    : Node("imu_node"), fd_(-1) {
        fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", device.c_str());
            rclcpp::shutdown();
        }

        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

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

        // EKF predict/update
        update_filter();

        // publish
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = ax_;
        msg.linear_acceleration.y = ay_;
        msg.linear_acceleration.z = az_;

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

        // 四元数の微分
        Eigen::Quaterniond dq;
        dq.w() = 1.0;
        dq.vec() = 0.5 * omega * dt;
        q = (q * dq).normalized();

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

        x_(0) = q.w();
        x_(1) = q.x();
        x_(2) = q.y();
        x_(3) = q.z();
    }

    int fd_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // raw data
    double ax_=0, ay_=0, az_=0;
    double gx_=0, gy_=0, gz_=0;

    // 状態ベクトル
    Eigen::VectorXd x_{7};

    // scale factors (適当に補正値を入れる)
    double scale_acc_ = 9.81 / 16384.0;  // 例: ±2g レンジ
    double scale_gyro_ = M_PI / 180.0 / 131.0; // 例: ±250 deg/s
};

int main(int argc, char *argv[]) {
  const char dual_sense_motion_dev[] = "/dev/input/by-id/usb-Sony_Interactive_Entertainment_DualSense_Wireless_Controller-event-if03";
  const char* dev_name = dual_sense_motion_dev;

  rclcpp::init(argc, argv);
  if (argc >= 2) {
      dev_name = argv[1];
  }
  auto node = std::make_shared<ImuNode>(dev_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
