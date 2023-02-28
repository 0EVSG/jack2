#ifndef SOSSO_WRITECHANNEL_HPP
#define SOSSO_WRITECHANNEL_HPP

#include "sosso/Buffer.hpp"
#include "sosso/Channel.hpp"
#include "sosso/Logging.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <vector>

namespace sosso {

class WriteChannel : public Channel {
public:
  bool open(const char *device, bool exclusive = true) {
    int mode = O_WRONLY | O_NONBLOCK;
    if (exclusive) {
      mode |= O_EXCL;
    }
    return Channel::open(device, mode);
  }

  void set_target_latency(std::int64_t latency = 0) {
    if (latency != _target_latency) {
      _target_latency = latency;
      Log::info(SOSSO_LOC, "Playback target latency changed to %lld.",
                _target_latency);
    }
  }

  bool process(Buffer &buffer, std::int64_t end, std::int64_t now) {
    // Check for OSS buffer underruns.
    std::int64_t overdue = now - estimated_dropout();
    if ((overdue > 0 && get_play_underruns() > 0) || overdue > max_progress()) {
      std::int64_t progress = oss_progress(0, buffer_frames());
      std::int64_t loss = mark_loss(progress, now);
      Log::warn(SOSSO_LOC, "OSS playback buffer underrun, %lld lost.", loss);
      if (!mark_progress(progress + loss, now)) {
        return false;
      }
    }
    std::size_t write_limit = buffer.remaining();
    std::int64_t offset = buffer_offset(buffer.remaining(), end);
    if (offset > 0) {
      write_limit = std::min(write_limit, offset * frame_size());
      // Gap between buffers, replay parts to fill it up.
      std::int64_t rewind = buffer.rewind(offset * frame_size()) / frame_size();
      if (rewind > 0) {
        Log::info(SOSSO_LOC, "Write buffer gap %lld, rewind %lld.", offset,
                  rewind);
      }
      offset -= rewind;
    } else if (offset < 0) {
      // Overlapping buffers, skip the overlapping part.
      std::int64_t advance =
          buffer.advance((-offset) * frame_size()) / frame_size();
      Log::info(SOSSO_LOC, "Write buffer overlap %lld, advance by %lld.",
                offset, advance);
      offset += advance;
    }
    // Write as much as currently possible.
    std::size_t bytes_written = 0;
    if (!non_blocking_write(buffer.position(), write_limit, bytes_written)) {
      return false;
    }
    buffer.advance(bytes_written);
    // Assume OSS buffer is full if only part of the data was written.
    std::int64_t available = 0;
    if (bytes_written == write_limit) {
      // All data was written, query queued OSS buffer content.
      available = buffer_frames() - queued_samples();
    }
    std::int64_t processed = bytes_written / frame_size();
    std::int64_t progress = oss_progress(processed, available);
    if (!mark_progress(progress, now)) {
      return false;
    }
    if (offset > 0) {
      // Rewind the remaining buffer gap fill up parts.
      std::int64_t rewind = buffer.rewind(offset * frame_size()) / frame_size();
      Log::info(SOSSO_LOC, "Write buffer gap %lld, fill write %lld.", offset,
                rewind);
    }
    if (full_resync() && now >= end) {
      buffer.advance(buffer.remaining());
    }
    return true;
  }

private:
  std::int64_t buffer_offset(std::size_t remaining, std::int64_t end) {
    std::int64_t position = end - (remaining / frame_size());
    std::int64_t processed = _last_progress + buffer_frames() - oss_available();
    std::int64_t offset = position + _target_latency - processed;
    return offset;
  }

  bool non_blocking_write(char *buffer, std::size_t limit,
                          std::size_t &progress) {
    if (buffer && limit > 0) {
      ssize_t result = ::write(file_descriptor(), buffer, limit);
      if (result >= 0) {
        progress += result;
      } else if (errno == EAGAIN) {
        progress += 0;
      } else {
        Log::warn(SOSSO_LOC, "Data write failed with %d.", errno);
        return false;
      }
    }
    return true;
  }

  std::int64_t _target_latency = 0;
};

} // namespace sosso

#endif // SOSSO_WRITECHANNEL_HPP
