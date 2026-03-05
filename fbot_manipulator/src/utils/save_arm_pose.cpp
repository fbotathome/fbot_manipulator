#include <chrono>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using sensor_msgs::msg::JointState;

class SaveArmPose : public rclcpp::Node
{
public:
    SaveArmPose()
    : Node("save_arm_pose")
    {
        this->declare_parameter<std::string>(
            "xacro_path",
            "/home/insider/fbot_ws/src/xarm_ros2/xarm_moveit_config/srdf/_xarm6_macro.srdf.xacro");
            //TODO: Change that value in robot
        xacro_path_ = this->get_parameter("xacro_path").as_string();

        group_name_ = "${prefix}xarm6";
        default_arm_joint_order_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        arm_joint_order_ = loadArmJointOrder();

        subscription_ = this->create_subscription<JointState>(
            "/joint_states", 10,
            std::bind(&SaveArmPose::jointStateCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<JointState>("/saved_arm_pose", 10);
    }

private:
    // ── helpers ──────────────────────────────────────────────────────────

    std::string readFile(const std::string & path)
    {
        std::ifstream ifs(path);
        if (!ifs.is_open()) return {};
        std::ostringstream ss;
        ss << ifs.rdbuf();
        return ss.str();
    }

    bool writeFile(const std::string & path, const std::string & content)
    {
        std::ofstream ofs(path);
        if (!ofs.is_open()) return false;
        ofs << content;
        return ofs.good();
    }

    // ── load joint order from existing xacro ─────────────────────────────

    std::vector<std::string> loadArmJointOrder()
    {
        if (!std::ifstream(xacro_path_).good()) {
            RCLCPP_WARN(get_logger(), "%s not found. Using default joint order.", xacro_path_.c_str());
            return default_arm_joint_order_;
        }

        std::string content = readFile(xacro_path_);
        if (content.empty()) {
            RCLCPP_WARN(get_logger(), "Could not read %s. Using default joint order.", xacro_path_.c_str());
            return default_arm_joint_order_;
        }

        // Find first <group_state ... group="${prefix}xarm6" ...> ... </group_state>
        std::regex group_re(
            R"(<group_state[^>]*group="\$\{prefix\}xarm6"[^>]*>([\s\S]*?)</group_state>)");
        std::smatch group_match;
        if (!std::regex_search(content, group_match, group_re)) {
            RCLCPP_WARN(get_logger(), "No group_state for xarm6 found. Using default joint order.");
            return default_arm_joint_order_;
        }

        std::string block = group_match[1].str();
        std::regex joint_re(R"(<joint[^>]*name="\$\{prefix\}([^"]+)"[^>]*/?>)");
        std::vector<std::string> names;
        auto it = std::sregex_iterator(block.begin(), block.end(), joint_re);
        for (; it != std::sregex_iterator(); ++it) {
            names.push_back((*it)[1].str());
        }

        if (names.empty()) {
            RCLCPP_WARN(get_logger(), "No joints parsed from group_state. Using default joint order.");
            return default_arm_joint_order_;
        }

        std::ostringstream oss;
        for (size_t i = 0; i < names.size(); ++i) {
            if (i) oss << ", ";
            oss << names[i];
        }
        RCLCPP_INFO(get_logger(), "Detected arm joint order: [%s]", oss.str().c_str());
        return names;
    }

    // ── subscription callback ────────────────────────────────────────────

    void jointStateCallback(const JointState::SharedPtr msg)
    {
        if (done_saving_) return;

        publisher_->publish(*msg);

        if (!save_loop_started_) {
            save_loop_started_ = true;
            RCLCPP_INFO(get_logger(), "Starting pose save session...");
            savePose();
            done_saving_ = true;
            RCLCPP_INFO(get_logger(), "Pose saving complete. Shutting down...");
            rclcpp::shutdown();
        }
    }

    // ── wait for a single JointState message ─────────────────────────────

    JointState::SharedPtr waitForJointState(double timeout_sec)
    {
        JointState::SharedPtr received;
        auto sub = this->create_subscription<JointState>(
            "/joint_states", 10,
            [&received](const JointState::SharedPtr msg) {
                if (!received) received = msg;
            });

        auto start = std::chrono::steady_clock::now();
        while (!received && rclcpp::ok()) {
            rclcpp::spin_some(this->shared_from_this());
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration<double>(elapsed).count() >= timeout_sec) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return received;
    }

    // ── find a joint value (exact or suffix match) ───────────────────────

    bool findJointValue(
        const std::map<std::string, double> & joint_map,
        const std::string & joint_name,
        double & out)
    {
        auto it = joint_map.find(joint_name);
        if (it != joint_map.end()) { out = it->second; return true; }

        std::vector<double> suffix_matches;
        for (const auto & [name, value] : joint_map) {
            if (name.size() >= joint_name.size() &&
                name.compare(name.size() - joint_name.size(), joint_name.size(), joint_name) == 0)
            {
                suffix_matches.push_back(value);
            }
        }
        if (suffix_matches.size() == 1) { out = suffix_matches[0]; return true; }
        return false;
    }

    // ── interactive save loop ────────────────────────────────────────────

    void savePose()
    {
        std::regex name_re(R"(^[A-Za-z0-9_.\-]+$)");

        while (rclcpp::ok()) {
            std::cout << "Move the arm to the desired pose and enter its name "
                         "(e.g., 'PrePickup', 'LookToGarbage'): " << std::flush;
            std::string arm_name;
            if (!std::getline(std::cin, arm_name)) break;

            // trim
            arm_name.erase(0, arm_name.find_first_not_of(" \t\r\n"));
            arm_name.erase(arm_name.find_last_not_of(" \t\r\n") + 1);

            if (arm_name.empty()) {
                RCLCPP_WARN(get_logger(), "Pose name cannot be empty. Pose not saved.");
                continue;
            }
            if (!std::regex_match(arm_name, name_re)) {
                RCLCPP_WARN(get_logger(),
                    "Invalid pose name. Use only letters, numbers, '-', '_' or '.'. Pose not saved.");
                continue;
            }

            auto msg = waitForJointState(2.0);
            if (!msg) {
                RCLCPP_WARN(get_logger(), "Timed out waiting for /joint_states. Pose not saved.");
                continue;
            }

            std::map<std::string, double> joint_map;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                joint_map[msg->name[i]] = msg->position[i];
            }

            std::vector<double> ordered_pose;
            std::vector<std::string> missing;
            for (const auto & jn : arm_joint_order_) {
                double val;
                if (findJointValue(joint_map, jn, val)) {
                    ordered_pose.push_back(val);
                } else {
                    missing.push_back(jn);
                }
            }

            if (!missing.empty()) {
                std::ostringstream oss;
                for (size_t i = 0; i < missing.size(); ++i) {
                    if (i) oss << ", ";
                    oss << missing[i];
                }
                RCLCPP_WARN(get_logger(),
                    "Missing joints in /joint_states: [%s]. Pose not saved.", oss.str().c_str());
                continue;
            }

            if (writeToXacro(arm_name, ordered_pose)) {
                RCLCPP_INFO(get_logger(), "Saved '%s' to %s", arm_name.c_str(), xacro_path_.c_str());
            }

            std::cout << "Do you want to add more poses? (y/n): " << std::flush;
            std::string answer;
            if (!std::getline(std::cin, answer)) break;
            answer.erase(0, answer.find_first_not_of(" \t\r\n"));
            answer.erase(answer.find_last_not_of(" \t\r\n") + 1);
            std::transform(answer.begin(), answer.end(), answer.begin(), ::tolower);

            if (answer == "n" || answer == "no") {
                break;
            } else if (answer == "y" || answer == "yes") {
                continue;
            } else {
                RCLCPP_WARN(get_logger(), "Invalid input. Please enter 'y' or 'n'. Continuing by default.");
            }
        }
    }

    // ── format a <group_state> block ─────────────────────────────────────

    std::string formatGroupState(const std::string & pose_name,
                                 const std::vector<double> & pose_values)
    {
        std::ostringstream oss;
        oss << "    <group_state name=\"" << pose_name
            << "\" group=\"" << group_name_ << "\">\n";

        for (size_t i = 0; i < arm_joint_order_.size(); ++i) {
            // Use full precision like the Python ".17g"
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.17g", pose_values[i]);
            oss << "      <joint name=\"${prefix}" << arm_joint_order_[i]
                << "\" value=\"" << buf << "\" />\n";
        }
        oss << "    </group_state>";
        return oss.str();
    }

    // ── write / update the xacro file ────────────────────────────────────

    bool writeToXacro(const std::string & pose_name,
                      const std::vector<double> & pose_values)
    {
        if (!std::ifstream(xacro_path_).good()) {
            RCLCPP_ERROR(get_logger(), "%s does not exist. Pose not saved.", xacro_path_.c_str());
            return false;
        }

        std::string content = readFile(xacro_path_);
        if (content.empty()) {
            RCLCPP_ERROR(get_logger(), "Error reading %s.", xacro_path_.c_str());
            return false;
        }

        std::string block = formatGroupState(pose_name, pose_values);

        // Try to replace an existing group_state with the same name
        std::string escaped_name = std::regex_replace(pose_name, std::regex(R"([.^$|()\\*+?{}\[\]])"), R"(\$&)");
        std::regex existing_re(
            R"([ \t]*<group_state\s+name=")" + escaped_name +
            R"("\s+group="\$\{prefix\}xarm6">[\s\S]*?</group_state>\s*\n?)");

        std::string updated;
        if (std::regex_search(content, existing_re)) {
            updated = std::regex_replace(content, existing_re, block + "\n",
                                         std::regex_constants::format_first_only);
        } else {
            // Insert before <!-- gripper --> comment
            std::regex gripper_re(R"([ \t]*<!--\s*gripper\s*-->)");
            std::smatch gm;
            if (std::regex_search(content, gm, gripper_re)) {
                auto pos = static_cast<size_t>(gm.position(0));
                updated = content.substr(0, pos) + block + "\n\n" + content.substr(pos);
            } else {
                // Fallback: insert before </xacro:macro>
                std::string close_tag = "  </xacro:macro>";
                auto pos = content.rfind(close_tag);
                if (pos == std::string::npos) {
                    RCLCPP_ERROR(get_logger(), "Could not find insertion point in xacro file.");
                    return false;
                }
                updated = content.substr(0, pos) + block + "\n" + content.substr(pos);
            }
        }

        if (!writeFile(xacro_path_, updated)) {
            RCLCPP_ERROR(get_logger(), "Error writing %s.", xacro_path_.c_str());
            return false;
        }
        return true;
    }

    // ── members ──────────────────────────────────────────────────────────

    std::string xacro_path_;
    std::string group_name_;
    std::vector<std::string> default_arm_joint_order_;
    std::vector<std::string> arm_joint_order_;

    rclcpp::Subscription<JointState>::SharedPtr subscription_;
    rclcpp::Publisher<JointState>::SharedPtr publisher_;

    bool save_loop_started_ = false;
    bool done_saving_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SaveArmPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}