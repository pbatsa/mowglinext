// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_behavior::BTContext;
using mowgli_behavior::IsLocalizationDegraded;
using mowgli_behavior::IsUndocking;

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

class LocalizationGuardTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory localization_factory;
  BT::BehaviorTreeFactory undocking_factory;
  BT::Tree localization_tree;
  BT::Tree undocking_tree;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_localization_guard");
    ctx->last_gnss_status_time = std::chrono::steady_clock::now();
    ctx->gnss_status_timeout_s = 3.0;

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    localization_factory.registerNodeType<IsLocalizationDegraded>("IsLocalizationDegraded");
    undocking_factory.registerNodeType<IsUndocking>("IsUndocking");

    localization_tree = localization_factory.createTreeFromText(R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsLocalizationDegraded/>
        </BehaviorTree>
      </root>
    )",
                                                                blackboard);
    undocking_tree = undocking_factory.createTreeFromText(R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsUndocking/>
        </BehaviorTree>
      </root>
    )",
                                                          blackboard);
  }
};

TEST_F(LocalizationGuardTest, FreshRtkFixedPasses)
{
  ctx->localization_degraded = false;
  ctx->rtk_degraded = false;
  EXPECT_EQ(localization_tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(LocalizationGuardTest, RtkFloatStopsEvenWithLowCovariance)
{
  ctx->localization_degraded = false;
  ctx->rtk_degraded = true;
  EXPECT_EQ(localization_tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(LocalizationGuardTest, StaleGnssStatusStopsLatchedFixedState)
{
  ctx->localization_degraded = false;
  ctx->rtk_degraded = false;
  ctx->last_gnss_status_time = std::chrono::steady_clock::now() - std::chrono::seconds(4);
  EXPECT_EQ(localization_tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(LocalizationGuardTest, UndockingExemptionRequiresActiveStartManeuver)
{
  ctx->current_command = 1;
  ctx->undock_start_recorded = true;
  ctx->has_high_level_status = true;
  ctx->last_high_level_status.state_name = "UNDOCKING";
  EXPECT_EQ(undocking_tree.tickOnce(), BT::NodeStatus::SUCCESS);

  ctx->last_high_level_status.state_name = "TRANSIT";
  EXPECT_EQ(undocking_tree.tickOnce(), BT::NodeStatus::FAILURE);
}

}  // namespace
