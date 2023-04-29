#ifndef SOSSO_READCHANNEL_HPP
#define SOSSO_READCHANNEL_HPP

#include "sosso/Buffer.hpp"
#include "sosso/Channel.hpp"
#include "sosso/Logging.hpp"
#include <algorithm>
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
    return Channel::open(device, mode);
  }

  std::int64_t oss_available() const {
    std::int64_t result = last_progress() - _read_position;
    result = std::max<std::int64_t>(result, 0);
    result = std::min<std::int64_t>(result, buffer_frames());
    return result;
  }

  std::int64_t pro_position() const { return oss_position() + _target_latency; }

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

  bool process(Buffer &buffer, std::int64_t end, std::int64_t now) {
    if (map()) {
      return process_mapped(buffer, end, now);
    } else {
      return process_read(buffer, end, now);
    }
  }

  bool process_mapped(Buffer &buffer, std::int64_t end, std::int64_t now) {
    end -= _target_latency;
    // Get OSS progress through map pointer.
    if (get_rec_pointer()) {
      std::int64_t progress = map_progress() - _oss_progress;
      _oss_progress += progress;
      std::int64_t available = last_progress() + progress - _read_position;
      std::int64_t loss = mark_loss(available - buffer_frames());
      if (loss > 0) {
        Log::warn(SOSSO_LOC, "OSS recording buffer overrun, %lld lost.", loss);
        _read_position = last_progress() - buffer_frames();
      }
      mark_progress(progress, now);
      set_target_latency();
    }
    // Only read what is available until OSS captured its complete buffer.
    std::int64_t oldest = last_progress() - buffer_frames();
    if (_oss_progress < buffer_frames()) {
      oldest = last_progress() - _oss_progress;
    }
    // Calculate offset of read buffer position to available OSS data.
    std::int64_t position = buffer_position(buffer.remaining(), end);
    if (position < oldest) {
      // First part of the read buffer already passed, fill it up.
      std::size_t fill = buffer.remaining((oldest - position) * frame_size());
      std::memset(buffer.position(), 0, fill);
      buffer.advance(fill);
      Log::info(SOSSO_LOC,
                "@%lld - %lld Read buffer overlap %lld, fill by %lu.", now, end,
                oldest - position, fill / frame_size());
      position += fill / frame_size();
    }
    if (position >= oldest && position < last_progress() &&
        buffer.remaining() > 0) {
      // Read from offset up to current position, if read buffer can hold it.
      std::int64_t offset = last_progress() - position;
      std::size_t length = buffer.remaining(offset * frame_size());
      unsigned pointer = (_oss_progress - offset) % buffer_frames();
      length = map_read(buffer.position(), pointer * frame_size(), length);
      buffer.advance(length);
    }
    freewheel_finish(buffer, end, now);
    _read_position = buffer_position(buffer.remaining(), end);
    return true;
  }

  bool process_read(Buffer &buffer, std::int64_t end, std::int64_t now) {
    end -= _target_latency;
    // Check for OSS buffer overruns.
    std::int64_t overdue = now - estimated_dropout(oss_available());
    if ((overdue > 0 && get_rec_overruns() > 0) || overdue > max_progress()) {
      std::int64_t progress = buffer_frames() - oss_available();
      std::int64_t loss = mark_loss(progress, now);
      Log::warn(SOSSO_LOC, "OSS recording buffer overrun, %lld lost.", loss);
      mark_progress(progress + loss, now);
      _read_position = last_progress() - buffer_frames();
    }
    std::int64_t position = buffer_position(buffer.remaining(), end);
    if (position < _read_position) {
      // Overlapping buffers, skip the overlapping part.
      char *data = buffer.position();
      std::size_t skip = (_read_position - position) * frame_size();
      skip = buffer.advance(skip);
      std::memset(data, 0, skip);
      Log::info(SOSSO_LOC,
                "@%lld - %lld Read buffer overlap %lld, advance by %lu.", now,
                end, _read_position - position, skip / frame_size());
      position += skip / frame_size();
    }
    // Read as much as currently available and fits into the buffer.
    std::size_t bytes_read = 0;
    if (!non_blocking_read(buffer.position(), buffer.remaining(), bytes_read)) {
      return false;
    }
    buffer.advance(bytes_read);
    // Assume all OSS data was read if buffer is not full yet.
    std::int64_t queued = 0;
    if (buffer.remaining() == 0) {
      // Buffer size was the limit, query queued OSS buffer content.
      queued = queued_samples();
    }
    std::int64_t progress = queued - (last_progress() - _read_position);
    progress += bytes_read / frame_size();
    mark_progress(progress, now);
    _read_position = last_progress() - queued;
    position = buffer_position(buffer.remaining(), end);
    if (position > _read_position) {
      // Gap between buffers, erase early frames not mapped to buffer.
      std::size_t erase = (position - _read_position) * frame_size();
      erase = buffer.erase(0, erase);
      Log::info(SOSSO_LOC, "@%lld - %lld Read buffer gap %lld, erased %lu.",
                now, end, position - _read_position, erase / frame_size());
    }
    set_target_latency();
    freewheel_finish(buffer, end, now);
    return true;
  }

private:
  std::int64_t buffer_position(std::size_t remaining, std::int64_t end) const {
    return end - (remaining / frame_size());
  }

  std::int64_t buffer_offset(std::size_t remaining, std::int64_t end) const {
    std::int64_t position = end - (remaining / frame_size());
    return position - oss_position();
  }

  std::int64_t oss_position() const { return _read_position; }

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

  void freewheel_finish(Buffer &buffer, std::int64_t end, std::int64_t now) {
    if (freewheel() && now >= end + balance()) {
      // Buffer is overdue in freewheel sync mode, finish immediately.
      std::size_t fill = buffer.remaining();
      std::memset(buffer.position(), 0, fill);
      buffer.advance(fill);
      Log::info(SOSSO_LOC, "@%lld - %lld Read buffer overdue, fill by %lu.",
                now, end, fill / frame_size());
    }
  }

  std::int64_t _target_latency = 0;
  std::int64_t _oss_progress = 0;
  std::int64_t _read_position = 0;
};

} // namespace sosso

#endif // SOSSO_READCHANNEL_HPP
