#ifndef SOSSO_PROTOTYPE_HPP
#define SOSSO_PROTOTYPE_HPP

#include "sosso/Logging.hpp"
#include "sosso/ReadChannel.hpp"
#include "sosso/WriteChannel.hpp"
#include <easy/profiler.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/soundcard.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace sosso {

class Prototype {
public:
  ~Prototype() { close(); }

  WriteChannel &out() { return _out; }

  ReadChannel &in() { return _in; }

  void close() {
    _out.close();
    _in.close();
  }

  bool write_loop(std::size_t frames, unsigned repetitions) {
    const long period = (frames * 1000000) / 48;
    size_t map_len = _out.buffer_size();
    Log::info(SOSSO_LOC, "Map length is %lu bytes.", map_len);
    Log::info(SOSSO_LOC, "Period is %ld ns.", period);
    void *map =
        mmap(NULL, map_len, PROT_WRITE, MAP_SHARED, _out.file_descriptor(), 0);
    if (map == MAP_FAILED) {
      Log::warn(SOSSO_LOC, "Memory map failed with error %d.", errno);
      return false;
    }
    // Get current time.
    timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
      Log::warn(SOSSO_LOC, "Get time failed with error %d.", errno);
      return false;
    }
    // Repeated wait and jitter evaluation.
    long long jitter_total = 0;
    long long jitter_max = 0;
    for (unsigned i = 0; i < repetitions; ++i) {
      timespec next = {now.tv_sec + (now.tv_nsec + period) / 1000000000,
                       (now.tv_nsec + period) % 1000000000};
      // Sleep for one period.
      if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) != 0) {
        Log::warn(SOSSO_LOC, "Sleep failed with error %d.", errno);
        return false;
      }
      // Get current time again and compare.
      if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        Log::warn(SOSSO_LOC, "Get time failed with error %d.", errno);
        return false;
      }
      long long jitter = (now.tv_sec - next.tv_sec) * 1000000000LL +
                         now.tv_nsec - next.tv_nsec;
      jitter_total += jitter;
      jitter_max = std::max(jitter_max, jitter);
    }

    if (repetitions > 0) {
      Log::info(SOSSO_LOC, "Jitter average is %lld ns, maximum %lld ns.",
                jitter_total / repetitions, jitter_max);
    }
    if (munmap(map, map_len) != 0) {
      Log::warn(SOSSO_LOC, "Memory unmap failed with error %d.", errno);
      return false;
    }
    return true;
  }

  void test_buffer_size() {
    if (_out.playback()) {
      for (unsigned period : {96, 128, 192, 256, 384, 512, 768, 1024}) {
        unsigned required = 2 * period * _out.frame_size();
        Log::info(SOSSO_LOC,
                  "Period of %u samples requires OSS buffer of %u bytes.",
                  period, required);
        _out.set_buffer_size(required);
        Log::info(SOSSO_LOC, "Actual OSS buffer size %lu vs %u required.",
                  _out.buffer_size(), required);
      }
    }
  }

private:
  WriteChannel _out;
  ReadChannel _in;
};

} // namespace sosso

#endif // SOSSO_PROTOTYPE_HPP
