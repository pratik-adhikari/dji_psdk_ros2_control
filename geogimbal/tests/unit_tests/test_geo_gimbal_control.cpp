// tests/unit_tests/test_geo_gimbal_control.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "geogimbal/geo_gimbal_control.hpp"

/**
 * @class GeoGimbalControlTest
 * @brief Minimal test fixture for the public API of GeoGimbalControl.
 */
class GeoGimbalControlTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<GeoGimbalControl>();
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    std::shared_ptr<GeoGimbalControl> node_;
};

/**
 * @test Verify we can set a target without crashing.
 */
TEST_F(GeoGimbalControlTest, TestSetTarget)
{
    TargetLocation loc{100.0, 200.0, 50.0};
    EXPECT_NO_THROW(node_->set_target(loc));
    // We do not verify the internal storage, since that is private.
    // For deeper checks, we would either add a getter or rely on integration tests.
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
