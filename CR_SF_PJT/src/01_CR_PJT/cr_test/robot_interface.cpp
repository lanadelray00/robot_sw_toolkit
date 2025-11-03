#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>

class RobotInterface : public rclcpp::Node
{
public:
  RobotInterface()
  : Node("robot_interface")
  {
    // === 1. MoveGroup 인터페이스 생성 ===
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "arm");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "gripper");

    RCLCPP_INFO(this->get_logger(), "✅ MoveGroup interfaces initialized (arm, gripper)");

    // === 2. Emergency Stop 서비스 서버 등록 ===
    emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "emergency_stop",
      std::bind(&RobotInterface::emergencyStopCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "✅ Emergency Stop service ready: /emergency_stop");
  }

  // ========== ① 목표 좌표 이동 ==========
  bool moveToPose(double x, double y, double z,
                  double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    arm_group_->setPoseTarget(target_pose);
    arm_group_->setGoalPositionTolerance(0.01);
    arm_group_->setGoalOrientationTolerance(0.01);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Planning success, executing...");
      arm_group_->execute(plan);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "❌ Planning failed");
    }
    return success;
  }

  // ========== ② 그리퍼 열기 ==========
  void openGripper()
  {
    gripper_group_->setNamedTarget("open");
    gripper_group_->move();
    RCLCPP_INFO(this->get_logger(), "🤖 Gripper opened");
  }

  // ========== ③ 그리퍼 닫기 ==========
  void closeGripper()
  {
    gripper_group_->setNamedTarget("close");
    gripper_group_->move();
    RCLCPP_INFO(this->get_logger(), "✊ Gripper closed");
  }

  // ========== ④ 홈 포즈 이동 ==========
  void moveHome()
  {
    arm_group_->setNamedTarget("init");
    arm_group_->move();
    RCLCPP_INFO(this->get_logger(), "🏠 Moved to home pose");
  }

  // ========== ⑤ 임의 포즈 이동 ==========
  void moveToNamedPose(const std::string &pose_name)
  {
    arm_group_->setNamedTarget(pose_name);
    arm_group_->move();
    RCLCPP_INFO(this->get_logger(), "📍 Moved to named pose: %s", pose_name.c_str());
  }

  // ========== ⑥ 비상정지 ==========
  void emergencyStopCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                             std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    arm_group_->stop();
    gripper_group_->stop();
    response->success = true;
    response->message = "Emergency stop activated — all motion halted.";
    RCLCPP_WARN(this->get_logger(), "🛑 EMERGENCY STOP TRIGGERED!");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotInterface>();

  // === 테스트 시퀀스 ===
  node->moveHome();
  node->openGripper();
  node->moveToPose(0.2, 0.0, 0.25);
  node->closeGripper();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
