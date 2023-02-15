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
    latency = std::max(latency, max_progress());
    if (latency > _target_latency) {
      _target_latency = latency;
      Log::info(SOSSO_LOC, "Playback target latency extended to %lld.",
                _target_latency);
    }
  }

  bool process(Buffer &buffer, std::int64_t end, std::int64_t now) {
    std::size_t write_limit = buffer.remaining();
    std::int64_t offset = buffer_offset(buffer.remaining(), end);
    if (offset > 0) {
      // Gap between buffers, replay parts to fill it up.
      std::int64_t rewind = buffer.rewind(offset * frame_size()) / frame_size();
      if (rewind > 0) {
        Log::info(SOSSO_LOC, "Write buffer gap %lld, rewind %lld.", offset,
                  rewind);
      }
      offset -= rewind;
      if (offset > 0) {
        write_limit = (offset + rewind) * frame_size();
      }
    } else if (offset < 0) {
      // Overlapping buffers, skip the overlapping part.
      std::int64_t advance =
          buffer.advance((-offset) * frame_size()) / frame_size();
      Log::info(SOSSO_LOC, "Write buffer overlap %lld, advance by %lld.",
                offset, advance);
      offset += advance;
    }
    // Write as much as currently possible.
    if (!write_buffer(buffer, write_limit, now)) {
      return false;
    }
    if (offset > 0) {
      // Rewind the remaining buffer gap fill up parts.
      std::int64_t rewind = buffer.rewind(offset * frame_size()) / frame_size();
      Log::info(SOSSO_LOC, "Write buffer gap %lld, fill write %lld.", offset,
                rewind);
    }
    if (_ignore > 0 && now >= end) {
      buffer.advance(buffer.remaining());
    }
    set_target_latency();
    return true;
  }

private:
  std::int64_t buffer_offset(std::size_t remaining, std::int64_t end) {
    std::int64_t position = end - (remaining / frame_size());
    std::int64_t processed = _last_progress + buffer_frames() - oss_available();
    std::int64_t offset = position + _target_latency - processed;
    return offset;
  }

  std::size_t restrict_write(std::size_t limit) {
    // To handle irregular initial progress, restrict write to latency target.
    if (false) {
      std::int64_t queued = buffer_frames() - _oss_available;
      if (queued < _target_latency) {
        // Write at most latency target frames to OSS queue.
        limit = std::min(limit, (_target_latency - queued) * frame_size());
      } else {
        // Skip write.
        limit = 0;
      }
    }
    return limit;
  }

  bool write_buffer(Buffer &buffer, std::size_t limit, std::int64_t now) {
    limit = restrict_write(std::min(limit, buffer.remaining()));
    // Write as much as currently possible.
    std::size_t bytes_written = 0;
    if (!non_blocking_write(buffer.position(), limit, bytes_written)) {
      return false;
    }
    buffer.advance(bytes_written);
    // Assume OSS buffer is full if only part of the data was written.
    std::int64_t available = 0;
    if (bytes_written == limit) {
      // All data was written, query queued OSS buffer content.
      available = buffer_frames() - queued_samples();
    }
    std::int64_t processed = bytes_written / frame_size();
    std::int64_t progress = oss_progress(processed, available);
    // Check for OSS buffer underruns.
    if (progress > 0 && processed + available == buffer_frames()) {
      if (get_play_underruns() > 0) {
        std::int64_t loss = mark_loss(progress, now);
        Log::warn(SOSSO_LOC, "OSS playback buffer underrun, %lld lost.", loss);
      }
    }
    return mark_progress(progress, now);
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
