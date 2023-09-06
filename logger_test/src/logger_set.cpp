#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"


using namespace std::chrono_literals;
using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;

class SetLoglevelNode : public rclcpp::Node
{
public:
    SetLoglevelNode(std::string name) : Node(name)
    {    
      srv_ = create_service<Empty>("setloglevel", std::bind(&SetLoglevelNode::loglevel_callback, this, _1, _2));
    }

private:
  void loglevel_callback(
    const std::shared_ptr<Empty::Request> request,
    const std::shared_ptr<Empty::Response> response
  )
  {

    (void) request;
    (void) response;

    // get_global_logging_mutex() is currently not being exported in the API
    // std::shared_ptr<std::recursive_mutex> logging_mutex;
    // logging_mutex = get_global_logging_mutex();
    {
      //std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
      rcutils_ret_t rcutils_ret = rcutils_logging_set_logger_level( "LoggingNode", RCUTILS_LOG_SEVERITY_DEBUG);

      (void) rcutils_ret;
    }
  }
    

  std::shared_ptr<rclcpp::Service<Empty>> srv_;

};  // class SetLoglevelNode


class LoggingNode : public rclcpp::Node
{
public:
    LoggingNode(std::string name) : Node(name)
    {
      timer_ = this->create_wall_timer(
      50ms, std::bind(&LoggingNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Logging debug messages at 20 Hz");
      RCLCPP_DEBUG(this->get_logger(), "Debug message");
    }

    rclcpp::TimerBase::SharedPtr timer_;

};  // class LoggingNode



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node1 = std::make_shared<SetLoglevelNode>("SetLoglevelNode");
  auto node2 = std::make_shared<LoggingNode>("LoggingNode");


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node1);
  executor.add_node(node2);
  executor.spin();


  rclcpp::shutdown();
  return 0;
}
