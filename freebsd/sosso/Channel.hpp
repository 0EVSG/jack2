#ifndef SOSSO_CHANNEL_HPP
#define SOSSO_CHANNEL_HPP

#include "sosso/Device.hpp"
#include <algorithm>

namespace sosso {

class Channel : public Device {
public:
  bool open(const char *device, int mode) {
    // Reset all internal statistics from last run.
    _last_processing = 0;
    _last_sync = 0;
    _last_progress = 0;
    _balance = 0;
    _min_progress = 0;
    _max_progress = 0;
    _oss_available = 0;
    _total_loss = 0;
    _sync_required = 0;
    _ignore = 4;
    bool ok = Device::open(device, mode);
    if (playback()) {
      _oss_available = buffer_frames();
    }
    return ok;
  }

  std::int64_t last_sync() const { return _last_sync; }

  std::int64_t last_processing() const { return _last_processing; }

  std::int64_t balance() const { return _balance; }

  std::int64_t next_min_progress() const {
    return _last_progress + _min_progress + _balance;
  }

  std::int64_t max_progress() const { return _max_progress; }

  std::int64_t min_progress() const { return _min_progress; }

  bool initial_fill() const { return _max_progress == 0; }

  void request_sync(unsigned count = 1U) { _sync_required += count; }

  unsigned sync_required() const { return _sync_required; }

  bool full_resync() const { return _ignore > 0; }

  std::int64_t oss_available() const { return _oss_available; }

  std::int64_t total_loss() const { return _total_loss; }

  std::int64_t safe_wakeup() const {
    return next_min_progress() + buffer_frames() - _oss_available -
           max_progress();
  }

  std::int64_t estimated_dropout() const {
    return _last_progress + _balance + buffer_frames() - _oss_available;
  }

  std::int64_t wakeup_time(std::int64_t now, std::int64_t sync_target) const {
    // Use one sync step by default.
    std::int64_t wakeup = now + Device::stepping();
    if (initial_fill() || full_resync()) {
      // Small steps when doing a full resync.
    } else if (sync_required() > 0 || wakeup + max_progress() > sync_target) {
      // Sync required, wake up prior to next progress if possible.
      if (next_min_progress() > wakeup) {
        wakeup = next_min_progress() - Device::stepping();
      } else if (next_min_progress() > now) {
        wakeup = next_min_progress();
      }
    } else {
      // Sleep until prior to sync target, then sync again.
      wakeup = sync_target - max_progress();
    }
    // Make sure we wake up at sync target.
    if (sync_target > now && sync_target < wakeup) {
      wakeup = sync_target;
    }
    // Make sure we don't sleep into an OSS under- or overrun.
    if (now < safe_wakeup() && safe_wakeup() < wakeup) {
      wakeup = std::max(safe_wakeup(), now + Device::stepping());
    }
    return wakeup;
  }

protected:
  std::int64_t oss_progress(std::int64_t processed,
                            std::int64_t oss_available) {
    // Compute OSS progress from read / write and buffer content.
    std::int64_t progress = processed + oss_available - _oss_available;
    _oss_available = oss_available;
    return progress;
  }

  bool mark_progress(std::int64_t progress, std::int64_t now) {
    if (progress > 0) {
      if (full_resync()) {
        // Some cards show irregular progress at the beginning, correct that.
        // Also correct loss after under- and overruns, assume same balance.
        _last_progress = now - progress;
        // Require a sync before transition back to normal processing.
        if (_ignore > 1 || now <= _last_processing + stepping()) {
          _ignore -= 1;
        }
      } else if (now < next_min_progress() ||
                 now <= _last_processing + stepping()) {
        // Sync on progress if early or after small processing steps.
        _balance = now - (_last_progress + progress);
        _last_sync = now;
        if (_sync_required > 0) {
          _sync_required -= 1;
        }
        if (progress < _min_progress || _min_progress == 0) {
          _min_progress = progress;
        }
        if (progress > _max_progress) {
          _max_progress = progress;
        }
      } else {
        // Big step with progress but no sync, requires a resync.
        _sync_required += 1;
      }
      _last_progress += progress;
    }
    _last_processing = now;
    return true;
  }

  std::int64_t mark_loss(std::int64_t progress, std::int64_t now) {
    // Approximate frames lost due to over- or underrun.
    std::int64_t loss = (now - _balance) - (_last_progress + progress);
    if (loss > 0) {
      _total_loss += loss;
      // Resync OSS progress to frame time (now) to recover from loss.
      _ignore = std::max(_ignore, 2U);
      _sync_required = std::max(_sync_required, 1U);
    } else {
      loss = 0;
    }
    return loss;
  }

  std::int64_t _last_processing = 0;
  std::int64_t _last_sync = 0;
  std::int64_t _last_progress = 0;
  std::int64_t _balance = 0;
  std::int64_t _min_progress = 0;
  std::int64_t _max_progress = 0;
  std::int64_t _oss_available = 0;
  std::int64_t _total_loss = 0;
  unsigned _sync_required = 0;
  unsigned _ignore = 4;
};

} // namespace sosso

#endif // SOSSO_CHANNEL_HPP
