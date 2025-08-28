#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
  // ROS用の引数を除去
  rclcpp::init(argc, argv);
  std::vector<std::string> args =
    rclcpp::remove_ros_arguments(argc, argv);
 
  // ここで argc, argv にはユーザ独自の引数だけが残っている
  std::cout << "残りの引数の数: " << args.size() << std::endl; // argv[0] は実行ファイル名

  for (unsigned i = 0; i < args.size(); ++i) {
    std::cout << "arg[" << i << "]: " << args[i] << std::endl;
  }

  auto node = std::make_shared<rclcpp::Node>("my_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
