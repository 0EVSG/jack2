#ifndef SOSSO_FRAMECLOCK_HPP
#define SOSSO_FRAMECLOCK_HPP

#include "sosso/Logging.hpp"
#include <time.h>

namespace sosso {

class FrameClock {
public:
  bool init_clock(unsigned sample_rate) {
    return set_sample_rate(sample_rate) && init_zero_time();
  }

  bool now(std::int64_t &result) const {
    std::int64_t time_ns = 0;
    if (get_time_offset(time_ns)) {
      result = time_to_frames(time_ns);
      return true;
    }
    return false;
  }

  bool sleep(std::int64_t wakeup_frame) const {
    std::int64_t time_ns = frames_to_time(wakeup_frame);
    return sleep_until(time_ns);
  }

  std::int64_t frames_to_time(std::int64_t frames) const {
    return (frames * 1000000000) / _sample_rate;
  }

  std::int64_t time_to_frames(std::int64_t time_ns) const {
    return (time_ns * _sample_rate) / 1000000000;
  }

  std::int64_t frames_to_absolute_us(std::int64_t frames) const {
    return _zero.tv_sec * 1000000 + _zero.tv_nsec / 1000 +
           frames_to_time(frames);
  }

  unsigned sample_rate() const { return _sample_rate; }

  bool set_sample_rate(unsigned sample_rate) {
    if (sample_rate > 0) {
      _sample_rate = sample_rate;
      return true;
    }
    return false;
  }

  unsigned stepping() const { return 16U * (1U + (_sample_rate / 50000)); }

private:
  bool init_zero_time() { return gettime(_zero); }

  bool get_time_offset(std::int64_t &result) const {
    timespec now;
    if (gettime(now)) {
      result = ((now.tv_sec - _zero.tv_sec) * 1000000000) + now.tv_nsec -
               _zero.tv_nsec;
      return true;
    }
    return false;
  }

  bool sleep_until(std::int64_t offset_ns) const {
    timespec wakeup = {_zero.tv_sec + (_zero.tv_nsec + offset_ns) / 1000000000,
                       (_zero.tv_nsec + offset_ns) % 1000000000};
    if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL) != 0) {
      Log::warn(SOSSO_LOC, "Sleep failed with error %d.", errno);
      return false;
    }
    return true;
  }

  bool gettime(timespec &result) const {
    if (clock_gettime(CLOCK_MONOTONIC, &result) != 0) {
      Log::warn(SOSSO_LOC, "Get time failed with error %d.", errno);
      return false;
    }
    return true;
  }

  timespec _zero = {0, 0};
  unsigned _sample_rate = 48000;
};

} // namespace sosso

#endif // SOSSO_FRAMECLOCK_HPP
