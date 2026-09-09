#pragma once

#include <cstdint>
#include <cstdio> // fixed: needed for std::fprintf/stderr, was relying on a transitive include
#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>

#include "topic_registry.hpp" // for TopicId, TopicTraits<T>

namespace logger_cfg
{

    struct TopicConfig final
    {
        std::string name;
        std::uint16_t struct_id{0};
        bool enabled{true};
    };

    class LoggerConfig final
    {
    public:
        // Load and parse the JSON file. Returns false on parse failure.
        // Intended to be called exactly once, from main(), before any
        // subscriber/recorder thread starts -- IsEnabled<T>() below only reads,
        // so concurrent reads from multiple threads after that point are safe.
        bool LoadFromFile(const std::string &path)
        {
            std::ifstream in(path);
            if (!in.is_open())
            {
                std::fprintf(stderr, "[logger_config] failed to open %s\n", path.c_str());
                return false;
            }

            nlohmann::json j;
            try
            {
                in >> j;
            }
            catch (const std::exception &e)
            {
                std::fprintf(stderr, "[logger_config] JSON parse error: %s\n", e.what());
                return false;
            }

            by_name_.clear();
            if (!j.contains("topics") || !j["topics"].is_array())
            {
                std::fprintf(stderr, "[logger_config] missing 'topics' array\n");
                return false;
            }

            for (const auto &entry : j["topics"])
            {
                TopicConfig cfg;
                cfg.name = entry.value("name", std::string{});
                cfg.struct_id = entry.value("struct_id", std::uint16_t{0});
                cfg.enabled = entry.value("enabled", true);
                if (cfg.name.empty())
                {
                    continue;
                }
                by_name_[cfg.name] = cfg;
            }

            return true;
        }

        void PrintSummary() const
        {
            std::printf("[logger_config] loaded %zu topics:\n", by_name_.size());
            for (const auto &[name, cfg] : by_name_)
            {
                std::printf("  %-16s struct_id=%u enabled=%s\n", name.c_str(), cfg.struct_id,
                            cfg.enabled ? "true" : "false");
            }
        }

        // Generic check: is topic T enabled, and does the JSON struct_id match
        // the compile-time registry? Logs a warning on mismatch but does not
        // crash -- mismatch just means "treat as disabled" for safety.
        template <typename T>
        bool IsEnabled() const
        {
            const char *name = logger_msgs::TopicTraits<T>::topicName;
            auto it = by_name_.find(name);
            if (it == by_name_.end())
            {
                // Not present in config at all -> default to enabled.
                return true;
            }

            const TopicConfig &cfg = it->second;
            const auto expected_id = static_cast<std::uint16_t>(logger_msgs::TopicTraits<T>::id);
            if (cfg.struct_id != expected_id)
            {
                std::fprintf(stderr,
                             "[logger_config] WARNING: topic '%s' struct_id mismatch "
                             "(json=%u, registry=%u) -- treating as disabled\n",
                             name, cfg.struct_id, expected_id);
                return false;
            }
            return cfg.enabled;
        }

    private:
        std::unordered_map<std::string, TopicConfig> by_name_;
    };

} // namespace logger_cfg
