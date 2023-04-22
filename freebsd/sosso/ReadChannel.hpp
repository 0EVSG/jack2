#ifndef SOSSO_READCHANNEL_HPP
#define SOSSO_READCHANNEL_HPP

#include "sosso/Buffer.hpp"
#include "sosso/Channel.hpp"
#include "sosso/Logging.hpp"
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
    _oss_available = 0;
    return Channel::open(device, mode);
  }

  std::int64_t oss_available() const { return _oss_available; }

  bool needs_processing(const Buffer &buffer, std::int64_t end) const {
    return buffer.remaining() > 0 || end - _target_latency > oss_position();
  }

  std::int64_t wakeup_time(std::int64_t sync_frames) const {
    return Channel::wakeup_time(sync_frames, oss_available());
  }

  void set_target_latency(std::int64_t latency = 0) {
    latency = std::max(latency, max_progress());
    if (latency > _target_latency) {
      _target_latency = latency;
      Log::info(SOSSO_LOC, "Recording target latency extended to %lld.",
                _target_latency);
    }
  }

  bool process_mapped(Buffer &buffer, std::int64_t end, std::int64_t now) {
    end -= _target_latency;
    // Get OSS progress through map pointer.
    if (get_rec_pointer()) {
      std::int64_t progress = map_progress() - _oss_progress;
      _oss_progress += progress;
      std::int64_t available = progress + oss_available();
      std::int64_t loss = mark_loss(available - buffer_frames());
      if (loss > 0) {
        Log::warn(SOSSO_LOC, "OSS recording buffer overrun, %lld lost.", loss);
      }
      available -= loss;
      _oss_available = available;
      mark_progress(progress, now);
      set_target_latency();
    }
    // Only read what is available until OSS captured its complete buffer.
    std::int64_t available = buffer_frames();
    if (_oss_progress < available) {
      available = _oss_progress;
    }
    // Calculate offset of read buffer position to available OSS data.
    std::int64_t offset = end - (buffer.remaining() / frame_size());
    offset -= (last_progress() - available);
    if (offset < 0) {
      // First part of the read buffer already passed, fill it up.
      std::size_t fill = buffer.remaining((-offset) * frame_size());
      std::memset(buffer.position(), 0, fill);
      buffer.advance(fill);
      Log::info(SOSSO_LOC,
                "@%lld - %lld Read buffer overlap %lld, fill by %lu.", now, end,
                offset, fill / frame_size());
      offset += fill / frame_size();
    }
    if (offset >= 0 && offset < available && buffer.remaining() > 0) {
      // Read from offset up to current position, if read buffer can hold it.
      available -= offset;
      std::size_t remaining = buffer.remaining(available * frame_size());
      unsigned pointer = (_oss_progress - available) % buffer_frames();
      remaining =
          map_read(buffer.position(), pointer * frame_size(), remaining);
      buffer.advance(remaining);
      available -= (remaining / frame_size());
      _oss_available = available;
    } else if (freewheel() && now >= end + balance()) {
      // Buffer is overdue in freewheel sync mode, finish immediately.
      std::size_t fill = buffer.remaining();
      std::memset(buffer.position(), 0, fill);
      buffer.advance(fill);
      Log::info(SOSSO_LOC, "@%lld - %lld Read buffer overdue, fill by %lu.",
                now, end, fill / frame_size());
    }
    return true;
  }

  bool process(Buffer &buffer, std::int64_t end, std::int64_t now) {
    if (map()) {
      return process_mapped(buffer, end, now);
    }
    end -= _target_latency;
    // Check for OSS buffer overruns.
    std::int64_t overdue = now - estimated_dropout(_oss_available);
    if ((overdue > 0 && get_rec_overruns() > 0) || overdue > max_progress()) {
      std::int64_t progress = buffer_frames() - _oss_available;
      _oss_available = buffer_frames();
      std::int64_t loss = mark_loss(progress, now);
      Log::warn(SOSSO_LOC, "OSS recording buffer overrun, %lld lost.", loss);
      mark_progress(progress + loss, now);
    }
    std::int64_t offset = buffer_offset(buffer.remaining(), end);
    if (offset < 0) {
      // Overlapping buffers, skip the overlapping part.
      char *position = buffer.position();
      std::size_t advance = buffer.advance((-offset) * frame_size());
      std::memset(position, 0, advance);
      Log::info(SOSSO_LOC,
                "@%lld - %lld Read buffer overlap %lld, advance by %lu.", now,
                end, offset, advance / frame_size());
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
    std::int64_t progress = processed + available - _oss_available;
    _oss_available = available;
    mark_progress(progress, now);
    if (offset > 0) {
      // Gap between buffers, erase early frames not mapped to buffer.
      std::size_t erased = buffer.erase(0, offset * frame_size());
      Log::info(SOSSO_LOC, "@%lld - %lld Read buffer gap %lld, erased %lu.",
                now, end, offset, erased / frame_size());
      offset -= erased / frame_size();
    }
    set_target_latency();
    return true;
  }

private:
  std::int64_t buffer_offset(std::size_t remaining, std::int64_t end) const {
    std::int64_t position = end - (remaining / frame_size());
    return position - oss_position();
  }

  std::int64_t oss_position() const {
    return last_progress() - oss_available();
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

  std::size_t map_read(char *dest, std::size_t pointer, std::size_t length) {
    std::size_t bytes_read = 0;
    if (length > 0) {
      // Sanitize pointer and length parameters.
      pointer = pointer % buffer_size();
      if (length > buffer_size()) {
        length = buffer_size();
      }
      if (pointer + length > buffer_size()) {
        // Read across buffer cycle boundary, write until buffer end first.
        bytes_read = map_read(dest, pointer, buffer_size() - pointer);
        length -= bytes_read;
        dest += bytes_read;
        pointer = 0;
      }
      // Read remaining data.
      std::memcpy(dest, map() + pointer, length);
      bytes_read += length;
    }
    return bytes_read;
  }

  std::int64_t _target_latency = 0;
  std::int64_t _oss_progress = 0;
  std::int64_t _oss_available = 0;
};

} // namespace sosso

#endif // SOSSO_READCHANNEL_HPP
