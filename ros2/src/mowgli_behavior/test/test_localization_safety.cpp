// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::IsLocalizationUnsafe;
using mowgli_interfaces::msg::GnssStatus;
using namespace std::chrono_literals;

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

class LocalizationSafetyTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_localization_safety");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<IsLocalizationUnsafe>("IsLocalizationUnsafe");
  }

  void setHealthyStatus(bool fixed = true)
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    auto& status = ctx->latest_gnss_status;
    status.fix_valid = true;
    status.fix_type = fixed ? GnssStatus::FIX_TYPE_RTK_FIXED : GnssStatus::FIX_TYPE_RTK_FLOAT;
    status.rtk_mode = fixed ? GnssStatus::RTK_MODE_FIXED : GnssStatus::RTK_MODE_FLOAT;
    status.differential_corrections = true;
    status.corrections_active = true;
    status.correction_stream_status = GnssStatus::CORRECTION_STREAM_STATUS_ACTIVE;
    status.msm_summary_seen = true;
    status.msm_summary_age_s = 0.1f;
    ctx->has_gnss_status = true;
    ctx->last_gnss_status_time = std::chrono::steady_clock::now();
    ctx->gps_is_fixed = fixed;
  }

  void setWheelOdom(double x, double y)
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->wheel_odom_x = x;
    ctx->wheel_odom_y = y;
    ctx->last_wheel_odom_time = std::chrono::steady_clock::now();
    ctx->has_wheel_odom = true;
  }

  BT::Tree makeTree(const std::string& attrs)
  {
    const std::string xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsLocalizationUnsafe )" +
                            attrs + R"( />
        </BehaviorTree>
      </root>
    )";
    return factory.createTreeFromText(xml, blackboard);
  }
};

TEST_F(LocalizationSafetyTest, DisabledDoesNotBlockWithoutStatus)
{
  auto tree = makeTree(R"(enabled="false")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(LocalizationSafetyTest, ChargingDoesNotBlockWithoutStatus)
{
  ctx->latest_power.charger_enabled = true;
  auto tree = makeTree(R"(enabled="true")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(LocalizationSafetyTest, UndockBackupDoesNotBlockWithoutStatus)
{
  ctx->undock_backup_active = true;
  auto tree = makeTree(R"(enabled="true")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(LocalizationSafetyTest, PostUndockGraceDoesNotBlockWithoutStatus)
{
  ctx->undock_localization_grace_until = std::chrono::steady_clock::now() + 1s;
  auto tree = makeTree(R"(enabled="true")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(LocalizationSafetyTest, MissingStatusBlocksMotion)
{
  auto tree = makeTree(R"(enabled="true")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(LocalizationSafetyTest, StaleStatusBlocksMotion)
{
  setHealthyStatus();
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_gnss_status_time = std::chrono::steady_clock::now() - 5s;
  }
  auto tree = makeTree(R"(max_gnss_status_age_sec="0.1")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(LocalizationSafetyTest, MissingCorrectionsBlockAfterGraceWindow)
{
  setHealthyStatus();
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->latest_gnss_status.corrections_active = false;
  }
  auto tree = makeTree(R"(max_corrections_missing_sec="0.0")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(2ms);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(LocalizationSafetyTest, RtkFloatBlocksAfterGraceWindowWhenRequired)
{
  setHealthyStatus(false);
  auto tree = makeTree(R"(require_rtk_fixed="true" max_rtk_float_age_sec="0.0")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(2ms);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(LocalizationSafetyTest, RtkFloatBlocksAfterDriftBudgetWhenRequired)
{
  setHealthyStatus(false);
  setWheelOdom(0.0, 0.0);
  auto tree = makeTree(
      R"(require_rtk_fixed="true" max_rtk_float_age_sec="60.0" max_degraded_drift_m="0.5")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  setWheelOdom(0.6, 0.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(LocalizationSafetyTest, RtkFloatAllowedWhenFixedIsNotRequired)
{
  setHealthyStatus(false);
  auto tree = makeTree(R"(require_rtk_fixed="false" max_rtk_float_age_sec="0.0")");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

}  // namespace
