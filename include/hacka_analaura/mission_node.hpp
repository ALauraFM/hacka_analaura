#ifndef HACKA_ANALAURA__MISSION_NODE_HPP
#define HACKA_ANALAURA__MISSION_NODE_HPP


#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>

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

	rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_position_;

	std::vector<geometry_msgs::msg::Pose> _coordenates_;

	double _rate_pub_;

	unsigned int current_state_;

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
