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

  std::int64_t oss_available() const {
    std::int64_t result = last_progress() + buffer_frames() - _write_position;
    result = std::max<std::int64_t>(result, 0);
    result = std::min<std::int64_t>(result, buffer_frames());
    return result;
  }

  std::int64_t pro_position() const { return _write_position; }

  std::int64_t wakeup_time(std::int64_t sync_frames) const {
    return Channel::wakeup_time(sync_frames, oss_available());
  }

  bool process(Buffer &buffer, std::int64_t end, std::int64_t now) {
    if (map()) {
      return process_mapped(buffer, end, now);
    } else {
      return process_write(buffer, end, now);
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
      std::int64_t loss =
          mark_loss(last_progress() + progress - _write_position);
      if (loss > 0) {
        Log::warn(SOSSO_LOC, "OSS playback buffer underrun, %lld lost.", loss);
        _write_position = last_progress();
      }
      mark_progress(progress, now);
    }
    // Buffer position should be between OSS progress and last write position.
    std::int64_t position =
        adjust_position(buffer, end, last_progress(), _write_position, now);
    // The writable window is the whole buffer, starting from OSS progress.
    if (buffer.remaining() > 0 && position >= last_progress() &&
        position < last_progress() + buffer_frames()) {
      // Write from buffer offset up to either OSS or write buffer end.
      std::int64_t offset = position - last_progress();
      unsigned pointer = (_oss_progress + offset) % buffer_frames();
      std::size_t length = (buffer_frames() - offset) * frame_size();
      length = buffer.remaining(length * frame_size());
      std::size_t written =
          map_write(buffer.position(), pointer * frame_size(), length);
      buffer.advance(written);
    }
    freewheel_finish(buffer, end, now);
    _write_position = buffer_position(buffer.remaining(), end);
    return true;
  }

  bool process_write(Buffer &buffer, std::int64_t end, std::int64_t now) {
    // Check for OSS buffer underruns.
    std::int64_t overdue = now - estimated_dropout(oss_available());
    if ((overdue > 0 && get_play_underruns() > 0) || overdue > max_progress()) {
      std::int64_t progress = _write_position - last_progress();
      std::int64_t loss = mark_loss(progress, now);
      Log::warn(SOSSO_LOC, "OSS playback buffer underrun, %lld lost.", loss);
      mark_progress(progress + loss, now);
      _write_position = last_progress();
    }
    // Adjust buffer position to OSS write position, if possible.
    std::int64_t position =
        adjust_position(buffer, end, _write_position, _write_position, now);
    std::size_t write_limit = buffer.remaining();
    if (position > _write_position) {
      // Replay to fill remaining gap, limit the write to just fill the gap.
      std::int64_t gap = position - _write_position;
      write_limit = std::min(write_limit, gap * frame_size());
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer gap %lld, fill %lld.",
                now, end, gap, write_limit / frame_size());
    }
    // Write as much as currently possible.
    std::size_t bytes_written = 0;
    if (!non_blocking_write(buffer.position(), write_limit, bytes_written)) {
      return false;
    }
    // Assume OSS buffer is full if only part of the data was written.
    std::int64_t queued = buffer_frames();
    if (bytes_written == write_limit) {
      // All data was written, query queued OSS buffer content.
      queued = queued_samples();
    }
    // Infer progress from OSS queue changes and newly written data.
    std::int64_t progress = (_write_position - last_progress()) - queued;
    progress += bytes_written / frame_size();
    mark_progress(progress, now);
    _write_position = last_progress() + queued;
    // Advance buffer position by written data, unless we filled a gap.
    if (position < _write_position) {
      buffer.advance((_write_position - position) * frame_size());
    }
    // Make sure buffers finish in time, despite irregular progress (freewheel).
    freewheel_finish(buffer, end, now);
    return true;
  }

private:
  std::int64_t buffer_position(std::size_t remaining, std::int64_t end) const {
    return end - (remaining / frame_size());
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

  void freewheel_finish(Buffer &buffer, std::int64_t end, std::int64_t now) {
    // Make sure buffers finish in time, despite irregular progress (freewheel).
    if (freewheel() && now >= end + balance()) {
      std::int64_t advance = buffer.advance(buffer.remaining()) / frame_size();
      Log::info(SOSSO_LOC,
                "@%lld - %lld Write freewheel finish remaining buffer %lld.",
                now, end, advance);
    }
  }

  std::int64_t adjust_position(Buffer &buffer, std::int64_t end,
                               std::int64_t min, std::int64_t max,
                               std::int64_t now) {
    std::int64_t position = buffer_position(buffer.remaining(), end);
    if (position > max) {
      // Gap between buffers, replay parts to fill it up.
      std::int64_t gap = position - max;
      std::int64_t rewind = buffer.rewind(gap * frame_size()) / frame_size();
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer gap %lld, rewind %lld.",
                now, end, gap, rewind);
      position -= rewind;
    } else if (position < min) {
      // Overlapping buffers, skip the overlapping part.
      std::int64_t overlap = min - position;
      std::int64_t skip = buffer.advance(overlap * frame_size()) / frame_size();
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer overlap %lld, skip %lld.",
                now, end, overlap, skip);
      position += skip;
    }
    return position;
  }

  std::int64_t _oss_progress = 0;
  std::int64_t _write_position = 0;
};

} // namespace sosso

#endif // SOSSO_WRITECHANNEL_HPP
