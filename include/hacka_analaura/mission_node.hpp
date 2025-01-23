#ifndef HACKA_ANALAURA__MISSION_NODE_HPP
#define HACKA_ANALAURA__MISSION_NODE_HPP


#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace hacka_analaura

{
class MissionNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit MissionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	~MissionNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();

	rclcpp::Subscription<geometry_msgs::msg::Pose>::ConstSharedPtr sub_pose_;
	void subPose(const geometry_msgs::msg::Pose &msg);

	rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_goto_;

	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_state_machine;
   void startStateMachine(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
	
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr take_off_;

	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_;

	std::vector<geometry_msgs::msg::Pose> _coordenates_;

	rclcpp::TimerBase::SharedPtr tmr_pub_pose_;
   void tmrPubPose();

	std::vector<double> waypoints;

	int qty_points;

	double _rate_state_machine_;

	unsigned int current_state_;

	bool have_a_goal_{false};

	bool start_state_machine_{false};

	bool is_active_{false};

enum States
{
	/*takeoff*/
	BEGIN,
	/*goto first coordinates*/
	COORDINATES1,
	/*goto second coordinates*/
	COORDINATES2,
	/*goto third coordinates*/
	COORDINATES3,
	/*got fourth coordinates*/
	COORDINATES4,
	/*finish and land*/
	FINISH,

};
	
};
}

#endif
