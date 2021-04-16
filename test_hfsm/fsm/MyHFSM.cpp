// NOLINT (legal/copyright)

#include "MyHFSM.hpp"

namespace cascade_hfsm
{
MyHFSM::MyHFSM()
: CascadeLifecycleNode("MyHFSM"), state_(INITIAL), myBaseId_("MyHFSM")
{
  declare_parameter("frequency");

  state_ts_ = now();
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
}

MyHFSM::~MyHFSM()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyHFSM::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  deactivateAllDeps();

  state_ = INITIAL;
  state_ts_ = now();

  Initial_activateDeps();
  Initial_code_once();


  double frequency = 5.0;
  get_parameter_or<double>("frequency", frequency, 5.0);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double, std::ratio<1>>(1.0 / frequency),
    std::bind(&MyHFSM::tick, this));

  state_pub_->on_activate();
  return CascadeLifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyHFSM::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  loop_timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(previous_state);
}

void MyHFSM::tick()
{
  std_msgs::msg::String msg;

  switch (state_) {
    case SECOND:
      Second_code_iterative();

      msg.data = "Second";
      state_pub_->publish(msg);

      if (Second_2_Initial()) {
        deactivateAllDeps();

        state_ = INITIAL;
        state_ts_ = now();

        Initial_activateDeps();
        Initial_code_once();
      }
      break;
    case FIRST:
      First_code_iterative();

      msg.data = "First";
      state_pub_->publish(msg);

      if (First_2_Second()) {
        deactivateAllDeps();

        state_ = SECOND;
        state_ts_ = now();

        Second_activateDeps();
        Second_code_once();
      }
      break;
    case INITIAL:
      Initial_code_iterative();

      msg.data = "Initial";
      state_pub_->publish(msg);

      if (Initial_2_First()) {
        deactivateAllDeps();

        state_ = FIRST;
        state_ts_ = now();

        First_activateDeps();
        First_code_once();
      }
      break;
  }
}

void
MyHFSM::deactivateAllDeps()
{
  remove_activation("TestComp");
}

void
MyHFSM::Second_activateDeps()
{
}
void
MyHFSM::First_activateDeps()
{
  add_activation("TestComp");
}
void
MyHFSM::Initial_activateDeps()
{
}


}  // namespace cascade_hfsm
