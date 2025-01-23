#include "hacka_analaura/mission_node.hpp"

namespace hacka_analaura
{
	/*MissionNode()*/
MissionNode::MissionNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("mission_node", "", options){
  RCLCPP_INFO(get_logger(), "Creating");

	declare_parameter("rate.state_machine", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.points", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.qty_points", rclcpp::ParameterValue(4));

	current_state_ = 0;
}

/*MissionNode()*/
MissionNode::~MissionNode() {

}

/* on_configure()*/
CallbackReturn MissionNode::on_configure(const rclcpp_lifecycle::State &) {
	RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}

/* on_activate()*/
CallbackReturn MissionNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
	RCLCPP_INFO(get_logger(), "Activating");

  pub_goto_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}

/*on_cleanup()*/
CallbackReturn MissionNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
	RCLCPP_INFO(get_logger(), "Cleaning up");

  pub_goto_.reset();

  sub_pose_.reset();

  return CallbackReturn::SUCCESS;
}

/* on_shutdown()*/
CallbackReturn MissionNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state){
	RCLCPP_INFO(get_logger(), "Shutting Down");
	
	return CallbackReturn::SUCCESS;
}

void MissionNode::getParameters() {
	get_parameter("rate.state_machine", _rate_state_machine_);
}  

void MissionNode::configPubSub(){
   RCLCPP_INFO(get_logger(), "initPubSub");

   pub_goto_ = create_publisher<geometry_msgs::msg::Pose>("/uav1/goto", 1);

   sub_pose_ = create_subscription<geometry_msgs::msg::Pose>("sub_pose", 1, std::bind(&MissionNode::subPose, this, std::placeholders::_1));

}

void MissionNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_pub_pose_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_state_machine_), std::bind(&MissionNode::tmrPubPose, this), nullptr);
}

void MissionNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  start_state_machine =
      create_service<std_srvs::srv::Trigger>("start_state_machine", std::bind(&MissionNode::start_state_machine, this, std::placeholders::_1, std::placeholders::_2));

}

void MissionNode::startStateMachine(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
   start_state_machine_ = true;
}

void MissionNode::tmrPubPose(){
   if (!is_active_) {
    return;
  }
  
  if(start_state_machine_ = true){
   have_a_goal_ = true;
   current_state_ = 1;
  }


  }
}


