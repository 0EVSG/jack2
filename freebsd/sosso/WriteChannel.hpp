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
      return (progress_done(now) || check_map_progress(now)) &&
             (buffer_done(buffer, end) || process_mapped(buffer, end, now));
    } else {
      return (progress_done(now) || check_write_progress(now)) &&
             (buffer_done(buffer, end) || process_write(buffer, end, now));
    }
  }

protected:
  bool progress_done(std::int64_t now) { return (last_processing() == now); }

  bool check_map_progress(std::int64_t now) {
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
      mark_progress(progress, now);
      if (loss > 0) {
        Log::warn(SOSSO_LOC, "OSS playback buffer underrun, %lld lost.", loss);
        _write_position = last_progress();
      }
    }
    return progress_done(now);
  }

  bool process_mapped(Buffer &buffer, std::int64_t end, std::int64_t now) {
    // Buffer position should be between OSS progress and last write position.
    std::int64_t position = buffer_position(buffer.remaining(), end);
    if (std::int64_t skip =
            buffer_advance(buffer, last_progress() - position)) {
      // First part of the buffer already played, skip it.
      Log::info(SOSSO_LOC, "@%lld - %lld Write %lld already played, skip %lld.",
                now, end, last_progress() - position, skip);
      position += skip;
    } else if (position != _write_position) {
      // Position mismatch, rewrite as much as possible.
      if (std::int64_t rewind =
              buffer_rewind(buffer, position - last_progress())) {
        Log::info(SOSSO_LOC,
                  "@%lld - %lld Write position mismatch, rewrite %lld.", now,
                  end, rewind);
        position -= rewind;
      }
    }
    // The writable window is the whole buffer, starting from OSS progress.
    if (buffer.remaining() > 0 && position >= last_progress() &&
        position < last_progress() + buffer_frames()) {
      if (_write_position < position && _write_position + 8 >= position) {
        // Small remaining gap between writes, fill in a replay patch.
        std::int64_t offset = _write_position - last_progress();
        unsigned pointer = (_oss_progress + offset) % buffer_frames();
        std::size_t length = (position - _write_position) * frame_size();
        length = buffer.remaining(length);
        std::size_t written =
            map_write(buffer.position(), pointer * frame_size(), length);
        Log::info(SOSSO_LOC, "@%lld - %lld Write small gap %lld, replay %lld.",
                  now, end, position - _write_position, written / frame_size());
      }
      // Write from buffer offset up to either OSS or write buffer end.
      std::int64_t offset = position - last_progress();
      unsigned pointer = (_oss_progress + offset) % buffer_frames();
      std::size_t length = (buffer_frames() - offset) * frame_size();
      length = buffer.remaining(length);
      std::size_t written =
          map_write(buffer.position(), pointer * frame_size(), length);
      buffer.advance(written);
      _write_position = buffer_position(buffer.remaining(), end);
    }
    _write_position += freewheel_finish(buffer, end, now);
    return true;
  }

  bool check_write_progress(std::int64_t now) {
    // Check for OSS buffer underruns.
    std::int64_t overdue = now - estimated_dropout(oss_available());
    if ((overdue > 0 && get_play_underruns() > 0) || overdue > max_progress()) {
      // OSS buffer underrun, estimate loss and progress from time.
      std::int64_t progress = _write_position - last_progress();
      std::int64_t loss = mark_loss(progress, now);
      Log::warn(SOSSO_LOC, "OSS playback buffer underrun, %lld lost.", loss);
      mark_progress(progress + loss, now);
      _write_position = last_progress();
    } else {
      // Infer progress from OSS queue changes.
      std::int64_t queued = queued_samples();
      std::int64_t progress = (_write_position - last_progress()) - queued;
      mark_progress(progress, now);
      _write_position = last_progress() + queued;
    }
    return progress_done(now);
  }

  bool process_write(Buffer &buffer, std::int64_t end, std::int64_t now) {
    bool ok = true;
    // Adjust buffer position to OSS write position, if possible.
    std::int64_t position = buffer_position(buffer.remaining(), end);
    if (std::int64_t rewind =
            buffer_rewind(buffer, position - _write_position)) {
      // Gap between buffers, replay parts to fill it up.
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer gap %lld, replay %lld.",
                now, end, position - _write_position, rewind);
      position -= rewind;
    } else if (std::int64_t skip =
                   buffer_advance(buffer, _write_position - position)) {
      // Overlapping buffers, skip the overlapping part.
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer overlap %lld, skip %lld.",
                now, end, _write_position - position, skip);
      position += skip;
    }
    if (oss_available() == 0) {
      // OSS buffer is full, nothing to do.
    } else if (position > _write_position) {
      // Replay to fill remaining gap, limit the write to just fill the gap.
      std::int64_t gap = position - _write_position;
      std::size_t write_limit = buffer.remaining(gap * frame_size());
      std::size_t bytes_written = 0;
      ok = non_blocking_write(buffer.position(), write_limit, bytes_written);
      Log::info(SOSSO_LOC, "@%lld - %lld Write buffer gap %lld, fill %lld.",
                now, end, gap, bytes_written / frame_size());
      _write_position += bytes_written / frame_size();
    } else if (position == _write_position) {
      // Write as much as currently possible.
      std::size_t write_limit = buffer.remaining();
      std::size_t bytes_written = 0;
      ok = non_blocking_write(buffer.position(), write_limit, bytes_written);
      _write_position += bytes_written / frame_size();
      buffer.advance(bytes_written);
    }
    // Make sure buffers finish in time, despite irregular progress (freewheel).
    freewheel_finish(buffer, end, now);
    return ok;
  }

private:
  std::int64_t buffer_position(std::size_t remaining, std::int64_t end) const {
    return end - (remaining / frame_size());
  }

  bool buffer_done(const Buffer &buffer, std::int64_t end) const {
    return buffer.remaining() == 0 && end <= _write_position;
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

  std::int64_t freewheel_finish(Buffer &buffer, std::int64_t end,
                                std::int64_t now) {
    std::int64_t advance = 0;
    // Make sure buffers finish in time, despite irregular progress (freewheel).
    if (freewheel() && now >= end + balance() && !buffer.done()) {
      advance = buffer.advance(buffer.remaining()) / frame_size();
      Log::info(SOSSO_LOC,
                "@%lld - %lld Write freewheel finish remaining buffer %lld.",
                now, end, advance);
    }
    return advance;
  }

  std::int64_t buffer_advance(Buffer &buffer, std::int64_t frames) {
    if (frames > 0) {
      return buffer.advance(frames * frame_size()) / frame_size();
    }
    return 0;
  }

  std::int64_t buffer_rewind(Buffer &buffer, std::int64_t frames) {
    if (frames > 0) {
      return buffer.rewind(frames * frame_size()) / frame_size();
    }
    return 0;
  }

  std::int64_t _oss_progress = 0;
  std::int64_t _write_position = 0;
};

} // namespace sosso

#endif // SOSSO_WRITECHANNEL_HPP
