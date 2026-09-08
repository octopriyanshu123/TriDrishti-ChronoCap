#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <utility>
#include <vector>

#include "i2w/impl.hpp"
#include "chrono_cap/chrono_cap.hpp"

// ---------------------------------------------------------------------------
// Pose2D publisher — 10 Hz
// ---------------------------------------------------------------------------
class PosePublisherSystem final : public i2w::SystemBase
{
public:
  PosePublisherSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

  static constexpr std::chrono::milliseconds kPeriod{100}; // 10 Hz

private:
  i2w::LifecycleResult OnSetup() noexcept override
  {
    i2w::PublisherOptions opts;
    opts.plane = plane_;
    auto pub = runtime().advertise<logger_msgs::Pose2D>("pose", opts);
    if (!pub)
    {
      return i2w::Fail();
    }

    pub_ = std::move(pub.value());
    return i2w::Ok();
  }

  i2w::LifecycleResult OnTick() noexcept override
  {
    logger_msgs::Pose2D pose{1.0f + static_cast<float>(seq_), 2.0f, 0.1f};
    auto sent = pub_.publish(pose, runtime().clock().now().ns);
    if (!sent)
    {
      return i2w::Ok();
    }
    std::printf("[pose]    published seq=%llu x=%.2f\n",
                static_cast<unsigned long long>(seq_), pose.x);
    std::fflush(stdout);
    ++seq_;
    return i2w::Ok();
  }

  i2w::Publisher<logger_msgs::Pose2D> pub_{};
  i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
  std::uint64_t seq_{0};
};

// ---------------------------------------------------------------------------
// Axis publisher — 100 Hz
// ---------------------------------------------------------------------------
class AxisPublisherSystem final : public i2w::SystemBase
{
public:
  AxisPublisherSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

  static constexpr std::chrono::milliseconds kPeriod{10}; // 100 Hz

private:
  i2w::LifecycleResult OnSetup() noexcept override
  {
    i2w::PublisherOptions opts;
    opts.plane = plane_;
    auto pub = runtime().advertise<logger_msgs::Axis>("axis", opts);
    if (!pub)
    {
      return i2w::Fail();
    }

    pub_ = std::move(pub.value());
    return i2w::Ok();
  }

  i2w::LifecycleResult OnTick() noexcept override
  {
    logger_msgs::Axis axis{};
    axis.sequence = seq_;
    axis.timestamp_ns = runtime().clock().now().ns;
    axis.axes_count = 4; // e.g. 2 sticks * 2 axes

    auto sent = pub_.publish(axis, axis.timestamp_ns);
    if (!sent)
    {
      return i2w::Ok();
    }
    std::printf("[axis]    published seq=%llu axes_count=%d\n",
                static_cast<unsigned long long>(seq_), axis.axes_count);
    std::fflush(stdout);
    ++seq_;
    return i2w::Ok();
  }

  i2w::Publisher<logger_msgs::Axis> pub_{};
  i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
  std::uint64_t seq_{0};
};

// ---------------------------------------------------------------------------
// Buttons publisher — 20 Hz
// ---------------------------------------------------------------------------
class ButtonsPublisherSystem final : public i2w::SystemBase
{
public:
  ButtonsPublisherSystem(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

  static constexpr std::chrono::milliseconds kPeriod{50}; // 20 Hz

private:
  i2w::LifecycleResult OnSetup() noexcept override
  {
    i2w::PublisherOptions opts;
    opts.plane = plane_;
    auto pub = runtime().advertise<logger_msgs::Buttons>("buttons", opts);
    if (!pub)
    {
      return i2w::Fail();
    }

    pub_ = std::move(pub.value());
    return i2w::Ok();
  }

  i2w::LifecycleResult OnTick() noexcept override
  {
    logger_msgs::Buttons buttons{};
    buttons.sequence = seq_;
    buttons.timestamp_ns = runtime().clock().now().ns;
    buttons.buttons_count = 8;

    auto sent = pub_.publish(buttons, buttons.timestamp_ns);
    if (!sent)
    {
      return i2w::Ok();
    }
    std::printf("[buttons] published seq=%llu buttons_count=%d\n",
                static_cast<unsigned long long>(seq_), buttons.buttons_count);
    std::fflush(stdout);
    ++seq_;
    return i2w::Ok();
  }

  i2w::Publisher<logger_msgs::Buttons> pub_{};
  i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
  std::uint64_t seq_{0};
};

// ---------------------------------------------------------------------------
// Runner: each system gets its own thread, ticking at its own period
// ---------------------------------------------------------------------------
template <typename SystemT>
void RunPublisher(i2w::Config config, const char *name)
{
  SystemT system(std::move(config));
  if (!system.Setup().ok)
  {
    std::printf("[%s] setup failed\n", name);
    return;
  }

  while (true)
  {
    if (!system.Tick().ok)
    {
      std::printf("[%s] tick failed\n", name);
      return;
    }
    std::this_thread::sleep_for(SystemT::kPeriod);
  }
}

int main()
{
  i2w::Config pose_cfg;
  pose_cfg.node_name = "pose_pub";
  pose_cfg.ns = "/demo";

  i2w::Config axis_cfg;
  axis_cfg.node_name = "axis_pub";
  axis_cfg.ns = "/demo";

  i2w::Config buttons_cfg;
  buttons_cfg.node_name = "buttons_pub";
  buttons_cfg.ns = "/demo";

  std::vector<std::thread> threads;
  threads.emplace_back(RunPublisher<PosePublisherSystem>, std::move(pose_cfg), "pose");
  threads.emplace_back(RunPublisher<AxisPublisherSystem>, std::move(axis_cfg), "axis");
  threads.emplace_back(RunPublisher<ButtonsPublisherSystem>, std::move(buttons_cfg), "buttons");

  for (auto &t : threads)
  {
    t.join();
  }
  return 0;
}
