#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct GoalPose {
  double x;
  double y;
  double theta;
};

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher()
  : Node("goal_publisher"), current_goal_idx_(0), started_(false)
  {
    // Optional override for goals file path
    this->declare_parameter<std::string>("goals_file", "");

    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // Resolve CSV path: parameter override or package default
    std::string csv_path = this->get_parameter("goals_file").as_string();
    if (csv_path.empty()) {
      std::string pkg_share =
        ament_index_cpp::get_package_share_directory("second_project");
      csv_path = pkg_share + "/csv/goals.csv";
    }

    loadGoals(csv_path);

    if (goals_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No goals loaded – shutting down.");
      rclcpp::shutdown();
      return;
    }

    // Poll for action server availability
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GoalPublisher::tryStart, this));
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::vector<GoalPose> goals_;
  size_t current_goal_idx_;
  bool started_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── Load CSV ──────────────────────────────────────────────────────────────
  void loadGoals(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open goals file: %s", path.c_str());
      return;
    }

    std::string line;
    // Skip header row
    std::getline(file, line);

    while (std::getline(file, line)) {
      if (line.empty()) {continue;}

      std::istringstream ss(line);
      std::string token;
      GoalPose g;

      try {
        std::getline(ss, token, ','); g.x     = std::stod(token);
        std::getline(ss, token, ','); g.y     = std::stod(token);
        std::getline(ss, token, ','); g.theta = std::stod(token);
        goals_.push_back(g);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Skipping malformed line: %s (%s)",
          line.c_str(), e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu goals from %s",
      goals_.size(), path.c_str());
  }

  // ── Wait for server, then start ───────────────────────────────────────────
  void tryStart()
  {
    if (started_) {return;}
    if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_INFO(this->get_logger(),
        "Waiting for navigate_to_pose action server...");
      return;
    }
    started_ = true;
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Action server found. Starting goal sequence.");
    sendNextGoal();
  }

  // ── Send the next goal in the sequence ────────────────────────────────────
  void sendNextGoal()
  {
    if (current_goal_idx_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(), "All %zu goals completed!", goals_.size());
      return;
    }

    const GoalPose & g = goals_[current_goal_idx_];
    RCLCPP_INFO(this->get_logger(),
      "Sending goal [%zu/%zu]: x=%.2f  y=%.2f  theta=%.3f",
      current_goal_idx_ + 1, goals_.size(), g.x, g.y, g.theta);

    // Build the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.pose.pose.position.x = g.x;
    goal_msg.pose.pose.position.y = g.y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g.theta);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // Goal accepted / rejected callback
    send_goal_options.goal_response_callback =
      [this](const GoalHandleNavigate::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(),
            "Goal [%zu] rejected by server – skipping.", current_goal_idx_);
          current_goal_idx_++;
          sendNextGoal();
        } else {
          RCLCPP_INFO(this->get_logger(),
            "Goal [%zu] accepted.", current_goal_idx_);
        }
      };

    // Feedback callback (optional, logs navigation distance remaining)
    send_goal_options.feedback_callback =
      [this](GoalHandleNavigate::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_DEBUG(this->get_logger(),
          "Distance remaining: %.2f m",
          feedback->distance_remaining);
      };

    // Result callback
    send_goal_options.result_callback =
      [this](const GoalHandleNavigate::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
              "Goal [%zu] SUCCEEDED.", current_goal_idx_);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(),
              "Goal [%zu] ABORTED – continuing to next.", current_goal_idx_);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),
              "Goal [%zu] CANCELED – continuing to next.", current_goal_idx_);
            break;
          default:
            RCLCPP_ERROR(this->get_logger(),
              "Goal [%zu] unknown result code.", current_goal_idx_);
            break;
        }
        current_goal_idx_++;
        sendNextGoal();
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
