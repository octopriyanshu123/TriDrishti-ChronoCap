#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <utility>
#include <vector>

#include "i2w/impl.hpp"
// #include "logger_types.hpp"
#include "logger_config.hpp"

using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
LoggerConfig LoggerConfig_;

namespace {

void on_pose(const i2w::Sample<Pose2D>& sample) {
  std::printf("[pose]    seq=%llu stamp=%lldns x=%.2f y=%.2f yaw=%.2f\n",
              static_cast<unsigned long long>(sample.header.seq),
              static_cast<long long>(sample.header.stamp_ns),
              sample.value.x,
              sample.value.y,
              sample.value.yaw);
  std::fflush(stdout);
}

void on_axis(const i2w::Sample<Axis>& sample) {
  std::printf("[axis]    seq=%llu stamp=%lldns axes_count=%d\n",
              static_cast<unsigned long long>(sample.header.seq),
              static_cast<long long>(sample.header.stamp_ns),
              sample.value.axes_count);
  std::fflush(stdout);
}

void on_buttons(const i2w::Sample<Buttons>& sample) {
  std::printf("[buttons] seq=%llu stamp=%lldns buttons_count=%d\n",
              static_cast<unsigned long long>(sample.header.seq),
              static_cast<long long>(sample.header.stamp_ns),
              sample.value.buttons_count);
  std::fflush(stdout);
}

} // namespace

enum class SubscriberError : std::uint32_t {
  SubscribeFailed = 100,
};

// ---------------------------------------------------------------------------
// Pose2D subscriber
// ---------------------------------------------------------------------------
class PoseSubscriberSystem final : public i2w::SystemBase {
 public:
  PoseSubscriberSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

 private:
  i2w::LifecycleResult OnSetup() noexcept override {
    i2w::SubscriptionOptions opts;
    opts.plane = plane_;
    opts.reliability = i2w::Reliability::BestEffort;
    opts.queue_depth = 64;
    opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto sub = runtime().subscribe<Pose2D>("pose", &on_pose, opts);
    if (!sub) {
      return i2w::Fail();
    }

    sub_ = std::move(sub.value());
    return i2w::Ok();
  }

  i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

  i2w::Subscription<Pose2D> sub_{};
  i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
};

// ---------------------------------------------------------------------------
// Axis subscriber
// ---------------------------------------------------------------------------
class AxisSubscriberSystem final : public i2w::SystemBase {
 public:
  AxisSubscriberSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

 private:
  i2w::LifecycleResult OnSetup() noexcept override {
    i2w::SubscriptionOptions opts;
    opts.plane = plane_;
    opts.reliability = i2w::Reliability::BestEffort;
    opts.queue_depth = 64;
    opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto sub = runtime().subscribe<Axis>("axis", &on_axis, opts);
    if (!sub) {
      return i2w::Fail();
    }

    sub_ = std::move(sub.value());
    return i2w::Ok();
  }

  i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

  i2w::Subscription<Axis> sub_{};
  i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
};

// ---------------------------------------------------------------------------
// Buttons subscriber
// ---------------------------------------------------------------------------
class ButtonsSubscriberSystem final : public i2w::SystemBase {
 public:
  ButtonsSubscriberSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

 private:
  i2w::LifecycleResult OnSetup() noexcept override {
    
      if (!LoggerConfig_.LoadFromFile("../config/logger_config.json"))
    {
        return i2w::Fail();
    }

    if (LoggerConfig_.IsEnabled<logger_msgs::Pose2D>())
    {
        auto sub = runtime().subscribe<logger_msgs::Pose2D>(
            TopicTraits<logger_msgs::Pose2D>::name, &on_pose, opts);
        if (!sub) return i2w::Fail();
        pose_sub_ = std::move(sub.value());
        std::printf("[recorder] pose: ENABLED\n");
    }
    else
    {
        std::printf("[recorder] pose: disabled by config\n");
    }
    
    i2w::SubscriptionOptions opts;
    opts.plane = plane_;
    opts.reliability = i2w::Reliability::BestEffort;
    opts.queue_depth = 64;
    opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto sub = runtime().subscribe<Buttons>("buttons", &on_buttons, opts);
    if (!sub) {
      return i2w::Fail();
    }

    sub_ = std::move(sub.value());
    return i2w::Ok();
  }

  i2w::LifecycleResult OnTick() noexcept override { return i2w::Ok(); }

  i2w::Subscription<Buttons> sub_{};
  i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
};

// ---------------------------------------------------------------------------
// Runner: each system gets its own thread; OnTick is a no-op poll, callbacks
// fire asynchronously as samples arrive
// ---------------------------------------------------------------------------
template <typename SystemT>
void RunSubscriber(i2w::Config config, const char* name) {
  SystemT system(std::move(config));
  if (!system.Setup().ok) {
    std::printf("[%s] setup failed\n", name);
    return;
  }

  while (true) {
    if (!system.Tick().ok) {
      std::printf("[%s] tick failed\n", name);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

int main(int argc, char** argv) {
  i2w::Config pose_cfg;
  pose_cfg.node_name = "pose_sub";
  pose_cfg.ns = "/demo";

  i2w::Config axis_cfg;
  axis_cfg.node_name = "axis_sub";
  axis_cfg.ns = "/demo";

  i2w::Config buttons_cfg;
  buttons_cfg.node_name = "buttons_sub";
  buttons_cfg.ns = "/demo";

  std::vector<std::thread> threads;
  threads.emplace_back(RunSubscriber<PoseSubscriberSystem>, std::move(pose_cfg), "pose");
  threads.emplace_back(RunSubscriber<AxisSubscriberSystem>, std::move(axis_cfg), "axis");
  threads.emplace_back(RunSubscriber<ButtonsSubscriberSystem>, std::move(buttons_cfg), "buttons");

  for (auto& t : threads) {
    t.join();
  }
  return 0;
}