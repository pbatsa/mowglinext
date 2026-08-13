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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_battery_critical_resume.cpp
 * @brief Regression for "robot stuck after a critical-battery charge".
 *
 * The CriticalBatteryDock branch used to END the session unconditionally
 * (EndSession + ClearCommand) once it left the charge-hold, so after a
 * critical-battery event the robot docked and wiped the coverage resume cursor.
 * A later fix auto-undocked immediately after recharge, which was too eager for
 * real docks: once charging is detected, the safe state is "parked and paused".
 * The current contract preserves the resume cursor with PauseCommand and waits
 * for an explicit START/schedule to resume.
 *
 * These tests exercise the exact control flow of the tail of CriticalBatteryDock
 * (from the CriticalChargeOrAbort Fallback onward) using the real EndSession,
 * ClearCommand, PauseCommand and IsBatteryAbove nodes, with a stand-in for
 * IsChargingProgressing (controllable). The IsBatteryLow entry gate is unchanged
 * by the fix and is omitted here (it cannot coexist with the resume gate in a
 * single tick — entry needs battery < 10 %, resume needs battery >= 95 %).
 */

#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::ClearCommand;
using mowgli_behavior::EndSession;
using mowgli_behavior::IsBatteryAbove;
using mowgli_behavior::NeedsDocking;
using mowgli_behavior::PauseCommand;

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Fixture — mirrors the CriticalBatteryDock navigation fallback.
//
// Charging pins are a physical dock signal. If they are already engaged, the
// critical-battery branch must skip DockRobot and move straight into the charge
// hold instead of trying to navigate while sitting on the dock.
// ---------------------------------------------------------------------------

class CriticalBatteryDockNavTest : public ::testing::Test
{
protected:
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;

  bool is_charging = false;
  int dock_attempts = 0;

  void SetUp() override
  {
    blackboard = BT::Blackboard::create();

    factory.registerSimpleCondition("IsCharging",
                                    [this](BT::TreeNode&)
                                    {
                                      return is_charging ? BT::NodeStatus::SUCCESS
                                                         : BT::NodeStatus::FAILURE;
                                    });
    factory.registerSimpleAction("DockRobot",
                                 [this](BT::TreeNode&)
                                 {
                                   ++dock_attempts;
                                   return BT::NodeStatus::SUCCESS;
                                 });
    factory.registerSimpleAction("CriticalBatteryNavFailed",
                                 [](BT::TreeNode&)
                                 {
                                   return BT::NodeStatus::SUCCESS;
                                 });
  }

  BT::Tree makeTree()
  {
    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <Fallback name="CriticalNavOrStop">
            <IsCharging/>
            <DockRobot/>
            <CriticalBatteryNavFailed/>
          </Fallback>
        </BehaviorTree>
      </root>
    )";
    return factory.createTreeFromText(xml, blackboard);
  }
};

TEST_F(CriticalBatteryDockNavTest, AlreadyChargingSkipsDockRobot)
{
  is_charging = true;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(dock_attempts, 0);
}

TEST_F(CriticalBatteryDockNavTest, NotChargingAttemptsDockRobot)
{
  is_charging = false;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(dock_attempts, 1);
}

// ---------------------------------------------------------------------------
// Fixture — mirrors the tail of CriticalBatteryDock (charge-hold + pause).
//
// A returned SUCCESS means the recovery pause ran; FAILURE means the
// charger-failed abort ran. ChargingProgress is the only stand-in we control;
// every other node is real.
// ---------------------------------------------------------------------------

class CriticalBatteryResumeTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  bool charging_ok = true;  // stand-in for IsChargingProgressing

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_battery_critical_resume");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    // Resume level pulled by {battery_full_pct} — the same knob MowingSequence
    // uses; matches the mowgli_robot.yaml default.
    blackboard->set("battery_full_pct", 95.0f);

    factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove");
    factory.registerNodeType<EndSession>("EndSession");
    factory.registerNodeType<ClearCommand>("ClearCommand");
    factory.registerNodeType<PauseCommand>("PauseCommand");

    factory.registerSimpleCondition("ChargingProgress",
                                    [this](BT::TreeNode&)
                                    {
                                      return charging_ok ? BT::NodeStatus::SUCCESS
                                                         : BT::NodeStatus::FAILURE;
                                    });
  }

  BT::Tree makeTree()
  {
    // The inner 30 s WaitForDuration is collapsed to a bare AlwaysFailure here:
    // in the recovery case IsBatteryAbove short-circuits it, and in the
    // charger-failed case ChargingProgress fails first, so the wait is never
    // reached. The RetryUntilSuccessful cap mirrors the real tree.
    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <Sequence name="CriticalBatteryDockTail">
            <Fallback name="CriticalChargeOrAbort">
              <RetryUntilSuccessful num_attempts="960">
                <Sequence>
                  <ChargingProgress/>
                  <Fallback>
                    <IsBatteryAbove threshold="{battery_full_pct}"/>
                    <AlwaysFailure/>
                  </Fallback>
                </Sequence>
              </RetryUntilSuccessful>
              <Sequence name="CriticalChargerFailed">
                <EndSession/>
                <ClearCommand/>
                <AlwaysFailure/>
              </Sequence>
            </Fallback>
            <PauseCommand/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    return factory.createTreeFromText(xml, blackboard);
  }
};

// Recovery: charged past battery_full_pct with a healthy charger MUST pause on
// the dock — clear only the active command and keep the resume cursor, so the
// next explicit START resumes from where it left off.
TEST_F(CriticalBatteryResumeTest, RecoveryPausesOnDockAndPreservesResumeCursor)
{
  ctx->current_command = 1;  // COMMAND_START in flight
  ctx->area_resume_pose_index[0] = 42;  // saved coverage cursor
  ctx->battery_percent = 100.0f;  // fully charged
  charging_ok = true;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // PauseCommand ran, but EndSession did not.
  EXPECT_EQ(ctx->current_command, 0);
  ASSERT_EQ(ctx->area_resume_pose_index.count(0), 1u);
  EXPECT_EQ(ctx->area_resume_pose_index[0], 42u);
}

// Dead charger: no charge progress MUST end the session (EndSession +
// ClearCommand) and abort the branch with FAILURE so the undock tail is
// SKIPPED — never resume mowing on a critical pack.
TEST_F(CriticalBatteryResumeTest, DeadChargerEndsSessionAndSkipsUndock)
{
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  ctx->battery_percent = 50.0f;  // charge stalled below resume level
  charging_ok = false;  // charger not progressing

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Session ended: command cleared and resume cursor wiped.
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_TRUE(ctx->area_resume_pose_index.empty());
}

TEST_F(CriticalBatteryResumeTest, LowBatteryDockLatchSurvivesThresholdBounceUntilPause)
{
  factory.registerNodeType<NeedsDocking>("NeedsDocking");

  static const char* pause_xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <Sequence name="LowBatteryPauseTail">
          <NeedsDocking threshold="20.0"/>
          <PauseCommand/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";
  static const char* latch_xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <NeedsDocking threshold="20.0"/>
      </BehaviorTree>
    </root>
  )";

  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  ctx->battery_percent = 19.0f;

  {
    auto tree = factory.createTreeFromText(pause_xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  }

  EXPECT_EQ(ctx->current_command, 0);
  ASSERT_EQ(ctx->area_resume_pose_index.count(0), 1u);
  EXPECT_EQ(ctx->area_resume_pose_index[0], 42u);
  EXPECT_FALSE(ctx->battery_docking_active);

  ctx->current_command = 1;
  ctx->battery_percent = 19.0f;
  auto latch_tree = factory.createTreeFromText(latch_xml, blackboard);
  EXPECT_EQ(latch_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->battery_docking_active);

  ctx->battery_percent = 25.0f;
  EXPECT_EQ(latch_tree.tickOnce(), BT::NodeStatus::SUCCESS);
}
