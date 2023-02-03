#ifndef SOSSO_READCHANNEL_HPP
#define SOSSO_READCHANNEL_HPP

#include "sosso/Buffer.hpp"
#include "sosso/Channel.hpp"
#include "sosso/Logging.hpp"
#include <easy/profiler.h>
#include <fcntl.h>
#include <unistd.h>

namespace sosso {

class ReadChannel : public Channel {
public:
  bool open(const char *device, bool exclusive = true) {
    int mode = O_RDONLY | O_NONBLOCK;
    if (exclusive) {
      mode |= O_EXCL;
    }
    return Device::open(device, mode);
  }

  void set_target_latency(std::int64_t latency = 0) {
    latency = std::max(latency, max_progress());
    if (latency > _target_latency) {
      _target_latency = latency;
      Log::info(SOSSO_LOC, "Recording target latency extended to %lld.",
                _target_latency);
    }
  }

  bool process(Buffer &buffer, std::int64_t end, std::int64_t now) {
    std::int64_t offset = buffer_offset(buffer.remaining(), end);
    if (offset < 0) {
      // Overlapping buffers, skip the overlapping part.
      char *position = buffer.position();
      std::size_t advance = buffer.advance((-offset) * frame_size());
      std::memset(position, 0, advance);
      Log::info(SOSSO_LOC, "Read buffer overlap %lld, advance by %lu.", offset,
                advance / frame_size());
      offset += advance / frame_size();
    }
    // Read as much as currently available and fits into the buffer.
    std::size_t bytes_read = 0;
    if (!non_blocking_read(buffer.position(), buffer.remaining(), bytes_read)) {
      return false;
    }
    buffer.advance(bytes_read);
    // Assume all OSS data was read if buffer is not full yet.
    std::int64_t available = 0;
    if (buffer.remaining() == 0) {
      // Buffer size was the limit, query queued OSS buffer content.
      available = queued_samples();
    }
    std::int64_t processed = bytes_read / frame_size();
    std::int64_t progress = oss_progress(processed, available);
    // Check for OSS buffer overruns.
    if (progress > 0 && processed + available == buffer_frames()) {
      if (get_rec_overruns() > 0) {
        std::int64_t loss = mark_loss(progress, now);
        Log::warn(SOSSO_LOC, "OSS recording buffer overrun, %lld lost.", loss);
      }
    }
    if (!mark_progress(progress, now)) {
      return false;
    }
    if (offset > 0) {
      // Gap between buffers, erase early frames not mapped to buffer.
      std::size_t erased = buffer.erase(0, offset * frame_size());
      Log::info(SOSSO_LOC, "Read buffer gap %lld, erased %lu.", offset,
                erased / frame_size());
      offset -= erased / frame_size();
    }
    set_target_latency();
    return true;
  }

private:
  std::int64_t buffer_offset(std::size_t remaining, std::int64_t end) {
    std::int64_t position = end - (remaining / frame_size());
    std::int64_t processed = _last_progress - oss_available();
    std::int64_t offset = position - _target_latency - processed;
    return offset;
  }

  bool non_blocking_read(char *buffer, std::size_t limit,
                         std::size_t &progress) {
    if (buffer && limit > 0) {
      ssize_t result = ::read(file_descriptor(), buffer, limit);
      if (result >= 0) {
        progress += result;
      } else if (errno == EAGAIN) {
        progress += 0;
      } else {
        Log::warn(SOSSO_LOC, "Data read failed with %d.", errno);
        return false;
      }
    }
    return true;
  }

  std::int64_t _target_latency = 0;
};

} // namespace sosso

#endif // SOSSO_READCHANNEL_HPP
