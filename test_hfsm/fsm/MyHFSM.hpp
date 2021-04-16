// NOLINT (legal/copyright) 

#ifndef MYHFSM_H_
#define MYHFSM_H_

#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace cascade_hfsm
{
class MyHFSM : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  MyHFSM();
  virtual ~MyHFSM();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  virtual void Second_code_iterative() {}
  virtual void Second_code_once() {}
  virtual void First_code_iterative() {}
  virtual void First_code_once() {}
  virtual void Initial_code_iterative() {}
  virtual void Initial_code_once() {}

  virtual bool Second_2_Initial() {return false;}
  virtual bool Initial_2_First() {return false;}
  virtual bool First_2_Second() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void Second_activateDeps();
  void First_activateDeps();
  void Initial_activateDeps();


  static const int SECOND = 0;
  static const int FIRST = 1;
  static const int INITIAL = 2;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace cascade_hfsm

#endif  // MYHFSM_H_
