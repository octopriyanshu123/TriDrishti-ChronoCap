#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <utility>
#include <vector>

#include "i2w/impl.hpp"
#include "logger_config.hpp"
#include "topic_registry.hpp"

using logger_msgs::Axis;
using logger_msgs::Buttons;
using logger_msgs::Pose2D;
using logger_msgs::TopicTraits;

// Loaded once in main(), before any subscriber thread starts.
// After that point every system only reads from it, so concurrent
// reads from multiple threads are safe (no concurrent writes).
logger_cfg::LoggerConfig g_logger_config;

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
    if (!g_logger_config.IsEnabled<Pose2D>()) {
      std::printf("[pose]    disabled by config\n");
      return i2w::Ok();
    }

    i2w::SubscriptionOptions opts;
    opts.plane = plane_;
    opts.reliability = i2w::Reliability::BestEffort;
    opts.queue_depth = 64;
    opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto sub = runtime().subscribe<Pose2D>(TopicTraits<Pose2D>::name, &on_pose, opts);
    if (!sub) {
      return i2w::Fail();
    }

    sub_ = std::move(sub.value());
    std::printf("[pose]    ENABLED\n");
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
    if (!g_logger_config.IsEnabled<Axis>()) {
      std::printf("[axis]    disabled by config\n");
      return i2w::Ok();
    }

    i2w::SubscriptionOptions opts;
    opts.plane = plane_;
    opts.reliability = i2w::Reliability::BestEffort;
    opts.queue_depth = 64;
    opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto sub = runtime().subscribe<Axis>(TopicTraits<Axis>::name, &on_axis, opts);
    if (!sub) {
      return i2w::Fail();
    }

    sub_ = std::move(sub.value());
    std::printf("[axis]    ENABLED\n");
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
    if (!g_logger_config.IsEnabled<Buttons>()) {
      std::printf("[buttons] disabled by config\n");
      return i2w::Ok();
    }

    i2w::SubscriptionOptions opts;
    opts.plane = plane_;
    opts.reliability = i2w::Reliability::BestEffort;
    opts.queue_depth = 64;
    opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

    auto sub = runtime().subscribe<Buttons>(TopicTraits<Buttons>::name, &on_buttons, opts);
    if (!sub) {
      return i2w::Fail();
    }

    sub_ = std::move(sub.value());
    std::printf("[buttons] ENABLED\n");
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
  // Load config ONCE, before any thread starts. This is what makes the
  // later concurrent IsEnabled<T>() reads across threads safe.
  if (!g_logger_config.LoadFromFile("/home/octobot/Github/TriDrishti-ws/src/TriDrishti-ChronoCap/config/logger_config.json")) {
    std::printf("failed to load logger config\n");
    return 1;
  }

  i2w::Config logger_sub_cfg;
  logger_sub_cfg.node_name = "logger_sub";
  logger_sub_cfg.ns = "/demo";

  std::vector<std::thread> threads;
  threads.emplace_back(RunSubscriber<PoseSubscriberSystem>, std::move(logger_sub_cfg), "pose");
  threads.emplace_back(RunSubscriber<AxisSubscriberSystem>, std::move(logger_sub_cfg), "axis");
  threads.emplace_back(RunSubscriber<ButtonsSubscriberSystem>, std::move(logger_sub_cfg), "buttons");

  for (auto& t : threads) {
    t.join();
  }
  return 0;
}