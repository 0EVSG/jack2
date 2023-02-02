#ifndef SOSSO_LOGGING_HPP
#define SOSSO_LOGGING_HPP

#include <cstdint>
#include <cstdio>

namespace sosso {

/*!
 * \brief Store the source location for logging.
 *
 * Keep implementation close to C++20 std::source_location, replace with that
 * when it is widely available.
 */
struct SourceLocation {
  std::uint_least32_t line() const { return _line; }
  std::uint_least32_t column() const { return _column; }
  const char *file_name() const { return _file_name; }
  const char *function_name() const { return _function_name; }

  std::uint_least32_t _line;
  std::uint_least32_t _column;
  const char *_file_name;
  const char *_function_name;
};

#define SOSSO_LOC                                                              \
  SourceLocation { __LINE__, 0, __FILE__, __func__ }

#define SOSSO_INFO(...) Log::info(SOURCE_LOC, __VA_ARGS__)

#define SOSSO_WARN(...) Log::warn(SOURCE_LOC, __VA_ARGS__)

class Log {
public:
  static void info(SourceLocation location, const char *message);

  template <typename... Args>
  static void info(SourceLocation location, const char *message, Args... args) {
    char formatted[256];
    std::snprintf(formatted, 256, message, args...);
    info(location, formatted);
  }

  static void warn(SourceLocation location, const char *message);

  template <typename... Args>
  static void warn(SourceLocation location, const char *message, Args... args) {
    char formatted[256];
    std::snprintf(formatted, 256, message, args...);
    warn(location, formatted);
  }
};

} // namespace sosso

#endif // SOSSO_LOGGING_HPP
