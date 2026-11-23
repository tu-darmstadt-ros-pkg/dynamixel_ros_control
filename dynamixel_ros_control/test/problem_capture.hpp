#ifndef DYNAMIXEL_ROS_CONTROL_TEST_PROBLEM_CAPTURE_HPP
#define DYNAMIXEL_ROS_CONTROL_TEST_PROBLEM_CAPTURE_HPP

#include <rcutils/logging.h>

#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace dynamixel_ros_control::test {

inline std::mutex messages_mutex;
inline std::vector<std::string> messages;
inline std::vector<std::string> logger_names;

inline void captureProblems(const rcutils_log_location_t* /*location*/, int severity, const char* name,
                            rcutils_time_point_value_t /*timestamp*/, const char* format, va_list* args)
{
  if (severity < RCUTILS_LOG_SEVERITY_WARN) {
    return;
  }
  char message[1024];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(message, sizeof(message), format, copy);
  va_end(copy);
  const std::lock_guard<std::mutex> lock(messages_mutex);
  messages.emplace_back(message);
  logger_names.emplace_back(name);
}

/// Collects warnings and errors while it lives
struct ProblemCapture
{
  ProblemCapture()
  {
    {
      const std::lock_guard<std::mutex> lock(messages_mutex);
      messages.clear();
      logger_names.clear();
    }
    rcutils_logging_set_output_handler(captureProblems);
  }
  ~ProblemCapture()
  {
    rcutils_logging_set_output_handler(previous);
  }
  ProblemCapture(const ProblemCapture&) = delete;
  ProblemCapture& operator=(const ProblemCapture&) = delete;

  [[nodiscard]] size_t count() const
  {
    const std::lock_guard<std::mutex> lock(messages_mutex);
    return messages.size();
  }

  /// The logger of the captured message containing @p text, empty if there is none
  [[nodiscard]] std::string loggerOf(const std::string& text) const
  {
    const std::lock_guard<std::mutex> lock(messages_mutex);
    for (size_t i = 0; i < messages.size(); ++i) {
      if (messages[i].find(text) != std::string::npos) {
        return logger_names[i];
      }
    }
    return "";
  }

  rcutils_logging_output_handler_t previous{rcutils_logging_get_output_handler()};
};

}  // namespace dynamixel_ros_control::test

#endif
