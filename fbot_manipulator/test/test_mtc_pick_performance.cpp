#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include "fbot_manipulator/mtc/mtc_pick_task.hpp"

namespace fbot_manipulator
{

class PickTaskPerformanceTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_pick_performance");
    }

    void TearDown() override
    {
        if (rclcpp::ok())
            rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(PickTaskPerformanceTest, PlanningTimeBenchmark)
{
    auto task = std::make_shared<MtcPickTask>(node_, "test_object");
    
    // buildTask requires robot model to be loaded
    if (!task->buildTask()) {
        GTEST_SKIP() << "Could not build task - robot model may not be available";
    }

    // Add collision object (simulating pick scenario)
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.3;
    pose.position.y = 0.2;
    pose.position.z = 0.05;
    pose.orientation.w = 1.0;

    geometry_msgs::msg::Vector3 size;
    size.x = 0.05;
    size.y = 0.05;
    size.z = 0.1;

    task->addCollisionObject("test_object", pose, size);

    // Measure planning time
    auto start = std::chrono::high_resolution_clock::now();
    bool success = task->plan();
    auto end = std::chrono::high_resolution_clock::now();

    auto duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    auto duration_s = duration_ms / 1000.0;

    std::cout << "\n=== PICK Task Planning Performance ===" << std::endl;
    std::cout << "Planning time: " << duration_s << " seconds (" << duration_ms << " ms)" << std::endl;
    std::cout << "Status: " << (success ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================================\n" << std::endl;

    EXPECT_TRUE(success);
    EXPECT_LT(duration_s, 60.0);
}

} // namespace fbot_manipulator

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
