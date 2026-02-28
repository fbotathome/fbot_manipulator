#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

// class GetArmPose : public rclcpp::Node
// {
//   public:
//     GetArmPose()
//     : Node("getarmpose")
//     {
//       subscription_ = this->create_subscription<std_msgs::msg::String>(
//       "xarm", 10, std::bind(&GetArmPose::topic_callback, this, _1));
//     }

//   private:
//     void topic_callback(const std_msgs::msg::String & msg) const
//     {
//       RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
//     }
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };

class GetArmPose : public rclcpp::Node {
  public:
    GetArmPose() : Node("getarmpose") {
      subscription_ = this->create_subscription<std_msgs::msg::Strings>("joint_states" 10, std::bin(&GetArmPose::jointStates_callback, this, _1));
    }

    private: 
      void jointStates_callback(const std_msgs::msg::String & msg) const {
        RCLCPP_INFO(this->get_logger(), "I Heard: %s", msg.data.c_str());
      }
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetArmPose>());
  rclcpp::shutdown();
  return 0;
}

