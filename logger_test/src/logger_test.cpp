#include <chrono>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node1 = std::make_shared<rclcpp::Node>("LoggerNode");
  
  // This will output a warning if /rosout logging is enabled.
  // node2 will publish with the same publisher, as it is linked 
  // to the name and node1 and node2 have identical names.
  auto node2 = std::make_shared<rclcpp::Node>("LoggerNode");

  // childlogger will throw an exception if uncommented and /rosout logging is enabled.
  // It cannot configure the child logger to use the parent's publisher
  // if the parent is not a node logger.
  auto namedlogger = rclcpp::get_logger("NamedLogger");
  //auto childlogger = namedlogger.get_child("childname");

  RCLCPP_INFO(node1->get_logger(), "Log 1");
  RCLCPP_INFO(node2->get_logger(), "Log 2");
  RCLCPP_INFO(node1->get_logger().get_child("child"), "Log 3");  // This is published to /rosout
  RCLCPP_INFO(rclcpp::get_logger("LoggerNode.child"), "Log 4");  // This is not published to /rosout (no child logger in scope)
  auto childnode1 = node1->get_logger().get_child("child");       
  RCLCPP_INFO(rclcpp::get_logger("LoggerNode.child"), "Log 5");  // This is published to /rosout (child logger in scope)

  rclcpp::shutdown();
  return 0;
}
