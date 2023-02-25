#ifndef SOSSO_DOUBLEBUFFER_HPP
#define SOSSO_DOUBLEBUFFER_HPP

#include "sosso/Buffer.hpp"
#include "sosso/Logging.hpp"
#include <algorithm>
#include <limits>

namespace sosso {

template <class Channel> class DoubleBuffer : public Channel {
  struct BufferRecord {
    Buffer buffer;
    std::int64_t end_frames = 0;
  };

public:
  ~DoubleBuffer() { Channel::close(); }

  bool ready() const { return _buffer_a.buffer.valid(); }

  bool set_buffer(Buffer &&buffer, std::int64_t end_frames) {
    // Set secondary buffer if available.
    if (!_buffer_b.buffer.valid()) {
      _buffer_b.buffer = std::move(buffer);
      _buffer_b.end_frames = end_frames;
      // Promote secondary buffer to primary if primary is not set.
      if (!_buffer_a.buffer.valid()) {
        std::swap(_buffer_b, _buffer_a);
      }
      return ready();
    }
    return false;
  }

  bool reset_buffers(std::int64_t end_frames) {
    // Reset primary buffer.
    if (_buffer_a.buffer.valid()) {
      std::memset(_buffer_a.buffer.data(), 0, _buffer_a.buffer.length());
      _buffer_a.buffer.reset();
      Log::info(SOSSO_LOC, "Primary buffer reset from %lld to %lld.", _buffer_a.end_frames, end_frames);
      _buffer_a.end_frames = end_frames;
    }
    // Reset secondary buffer.
    if (_buffer_b.buffer.valid()) {
      std::memset(_buffer_b.buffer.data(), 0, _buffer_b.buffer.length());
      _buffer_b.buffer.reset();
      end_frames += _buffer_b.buffer.length() / Channel::frame_size();
      Log::info(SOSSO_LOC, "Secondary buffer reset from %lld to %lld.", _buffer_a.end_frames, end_frames);
      _buffer_b.end_frames = end_frames;
    }
  }

  Buffer &&take_buffer() {
    std::swap(_buffer_a, _buffer_b);
    return std::move(_buffer_b.buffer);
  }

  bool process(std::int64_t now) {
    bool ok = ready();
    // Process primary buffer while not done, or if there is no secondary.
    if (_buffer_a.buffer.remaining() > 0 || !_buffer_b.buffer.valid()) {
      ok = ok && Channel::process(_buffer_a.buffer, _buffer_a.end_frames, now);
    }
    // Process secondary buffer if primary is done.
    if (_buffer_a.buffer.remaining() == 0 && _buffer_b.buffer.valid()) {
      ok = ok && Channel::process(_buffer_b.buffer, _buffer_b.end_frames, now);
    }
    return ok;
  }

  std::int64_t end_frames() const {
    if (ready()) {
      return _buffer_a.end_frames;
    }
    return 0;
  }

  std::int64_t period_left(std::int64_t now) const {
    return period_end() - now;
  }

  std::int64_t period_end() const {
    if (ready()) {
      return end_frames() + Channel::balance();
    }
    return 0;
  }

  std::int64_t wakeup_time(std::int64_t now) const {
    // No need to wake up if channel is not running.
    if (!Channel::is_open()) {
      return std::numeric_limits<std::int64_t>::max();
    }
    // Wakeup immediately if there's more work to do now.
    if (Channel::oss_available() > 0 && (_buffer_a.buffer.remaining() > 0 ||
                                         _buffer_b.buffer.remaining() > 0)) {
      Log::log(SOSSO_LOC, "Immediate wakeup at %lld for more work.", now);
      return now;
    }
    // Get upcoming buffer end and compute next channel wakeup time.
    std::int64_t sync_frames = now;
    if (!finished(now)) {
      sync_frames = period_end();
    } else if (_buffer_b.buffer.valid()) {
      sync_frames = _buffer_b.end_frames + Channel::balance();
    }
    return Channel::wakeup_time(now, sync_frames);
  }

  std::int64_t buffer_progress() const {
    return _buffer_a.buffer.progress() / Channel::frame_size();
  }

  bool finished(std::int64_t now) const {
    return period_end() <= now && _buffer_a.buffer.remaining() == 0;
  }

  void log_state(std::int64_t now) const {
    const char *direction = Channel::playback() ? "Out" : "In";
    const char *sync = (Channel::last_sync() == now) ? "sync" : "frame";
    std::int64_t buf_a = _buffer_a.buffer.progress() / Channel::frame_size();
    std::int64_t buf_b = _buffer_b.buffer.progress() / Channel::frame_size();
    Log::log(SOSSO_LOC,
             "%s %s, %lld bal %lld, buf A %lld B %lld OSS %lld, %lld left, "
             "req %u min %lld",
             direction, sync, now, Channel::balance(), buf_a, buf_b,
             Channel::oss_available(), period_left(now),
             Channel::sync_requested(), Channel::min_progress());
  }

private:
  BufferRecord _buffer_a;
  BufferRecord _buffer_b;
};

} // namespace sosso

#endif // SOSSO_DOUBLEBUFFER_HPP
