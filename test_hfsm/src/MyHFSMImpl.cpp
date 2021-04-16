
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "MyHFSM.hpp"

using std::placeholders::_1;

class MyHFSMImpl : public cascade_hfsm::MyHFSM
{
public:
  MyHFSMImpl()
  : MyHFSM()
  {
   
  }


  bool Second_2_Initial()
  {
    return (now() - state_ts_).seconds() > 5.0;
  }

  bool First_2_Second()
  {
    return (now() - state_ts_).seconds() > 5.0;
  }

  bool Initial_2_First()
  {
    return (now() - state_ts_).seconds() > 5.0;
  }

  void Second_code_iterative()
  {
    RCLCPP_INFO(get_logger(), "Second state");
  }
};

class TestComp : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
  public:
    TestComp() : CascadeLifecycleNode("TestComp")
    {

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state)
    {

      RCLCPP_INFO(get_logger(), "Activado TestComp");
      return CascadeLifecycleNode::on_activate(previous_state);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {

      RCLCPP_INFO(get_logger(), "Desactivado TestComp");
      return CascadeLifecycleNode::on_deactivate(previous_state);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto my_hfsm = std::make_shared<MyHFSMImpl>();
  auto test_node = std::make_shared<TestComp>();
 
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(my_hfsm->get_node_base_interface());
  executor.add_node(test_node->get_node_base_interface());


  my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();
  my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}