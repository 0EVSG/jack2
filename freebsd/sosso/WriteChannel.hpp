#ifndef SOSSO_WRITECHANNEL_HPP
#define SOSSO_WRITECHANNEL_HPP

#include "sosso/Buffer.hpp"
#include "sosso/Channel.hpp"
#include "sosso/Logging.hpp"
#include <cstring>
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

  bool process_mapped(Buffer &buffer, std::int64_t end, std::int64_t now) {
    // Get OSS progress through map pointer.
    if (get_play_pointer()) {
      std::int64_t progress = map_progress() - _oss_progress;
      if (progress > 0) {
        // Sometimes OSS playback starts with a bogus extra buffer cycle.
        if (progress > buffer_frames() &&
            now - last_processing() < buffer_frames() / 2) {
          Log::warn(SOSSO_LOC,
                    "OSS playback bogus buffer cycle, %lld frames in %lld.",
                    progress, now - last_processing());
          progress = progress % buffer_frames();
        }
        // Clear obsolete audio data in the buffer.
        map_write(nullptr, (_oss_progress % buffer_frames()) * frame_size(),
                  progress * frame_size());
        _oss_progress = map_progress();
      }
      std::int64_t available = progress + oss_available();
      std::int64_t loss = mark_loss(available - buffer_frames());
      if (loss > 0) {
        Log::warn(SOSSO_LOC, "OSS playback buffer underrun, %lld lost.", loss);
      }
      available -= loss;
      oss_progress(0, available);
      if (!mark_progress(progress, now)) {
        return false;
      }
    }
    // Treat the whole OSS buffer as available for writing.
    std::int64_t available = buffer_frames();
    // Calculate offset of write buffer position to available OSS window.
    std::int64_t offset = end - (buffer.remaining() / frame_size());
    offset -= _last_progress - _target_latency;
    if (offset < 0) {
      // First part of the write buffer already passed, skip it.
      std::size_t skip = buffer.advance((-offset) * frame_size());
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer overlap %lld, skip %lu.",
                now, end, offset, skip / frame_size());
      offset += skip / frame_size();
    }
    if (offset != buffer_frames() - oss_available()) {
      Log::info(SOSSO_LOC, "@%lld - %lld Write offset %lld vs previous %lld.",
                now, end, offset, buffer_frames() - oss_available());
    }
    if (offset >= 0 && offset < available && buffer.remaining() > 0) {
      // Write from offset up to either OSS or write buffer end.
      available -= offset;
      std::size_t remaining = buffer.remaining(available * frame_size());
      unsigned pointer = (_oss_progress + offset) % buffer_frames();
      // Write remaining data to OSS buffer.
      std::size_t written =
          map_write(buffer.position(), pointer * frame_size(), remaining);
      buffer.advance(written);
      available -= (written / frame_size());
      oss_progress(0, available);
    } else if (freewheel() && now >= end + balance() + _target_latency) {
      // Buffer is overdue in freewheel sync mode, finish immediately.
      std::size_t skip = buffer.advance(buffer.remaining());
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer overdue, skip all %lu.",
                now, end, skip / frame_size());
    }
    return true;
  }

  bool process(Buffer &buffer, std::int64_t end, std::int64_t now) {
    if (map()) {
      return process_mapped(buffer, end, now);
    }
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
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer gap %lld, rewind %lld.",
                now, end, offset, rewind);
      offset -= rewind;
    } else if (offset < 0) {
      // Overlapping buffers, skip the overlapping part.
      std::int64_t advance =
          buffer.advance((-offset) * frame_size()) / frame_size();
      Log::info(SOSSO_LOC,
                "@%lld - %lld Write buffer overlap %lld, advance by %lld.", now,
                end, offset, advance);
      offset += advance;
      write_limit = buffer.remaining();
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
      Log::info(SOSSO_LOC,
                "@%lld - %lld Write buffer gap %lld, fill write %lld.", now,
                end, offset, rewind);
    }
    if (freewheel() && now >= end) {
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

  std::size_t map_write(const char *source, std::size_t pointer,
                        std::size_t length) {
    std::size_t bytes_written = 0;
    if (length > 0) {
      // Sanitize pointer and length parameters.
      pointer = pointer % buffer_size();
      if (length > buffer_size()) {
        length = buffer_size();
      }
      if (pointer + length > buffer_size()) {
        // Write across buffer cycle boundary, write until buffer end first.
        bytes_written += map_write(source, pointer, buffer_size() - pointer);
        length -= bytes_written;
        if (source) {
          source += bytes_written;
        }
        pointer = 0;
      }
      // Write source if available, otherwise clear the buffer.
      if (source) {
        std::memcpy(map() + pointer, source, length);
      } else {
        std::memset(map() + pointer, 0, length);
      }
      bytes_written += length;
    }
    return bytes_written;
  }

  std::int64_t _target_latency = 0;
  std::int64_t _oss_progress = 0;
};

} // namespace sosso

#endif // SOSSO_WRITECHANNEL_HPP
