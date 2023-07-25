#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalTimer : public rclcpp::Node
{
public:
  MinimalTimer()
  : Node("minimal_timer")
  {
    counter_ = 0;


    // Comment one of both following lines to choose a reentrant or a mutually exclusive callback group:
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    //timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    timer_ = create_wall_timer(
      1s, std::bind(&MinimalTimer::timer_callback, this), timer_cb_group_);
  }

private:
  void timer_callback()
  {
    unsigned int localcounter;
    counter_++;
    localcounter = counter_;
    
    // Choose to always sleep, or to only sleep every 5th callback
    bool always_sleep = false;

    if (always_sleep)
    {
      sleep(10);
    }
    else
    {
      if (counter_ % 5 == 0) {
        sleep(3);
      }
    }


    RCLCPP_INFO(this->get_logger(), "Local counter: %u, Counter: %u", localcounter, counter_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::CallbackGroup> timer_cb_group_;
  unsigned int counter_;
};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Set to zero for the default number of threads, which is:
  //
  //     std::max(std::thread::hardware_concurrency(), 2U)
  //
  size_t number_of_threads = 5;
  
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), number_of_threads);
  
  auto node = std::make_shared<MinimalTimer>();
  executor.add_node(node);

  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
