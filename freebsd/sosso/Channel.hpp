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
    _total_loss = 0;
    _sync_level = 8;
    return Device::open(device, mode);
  }

  std::int64_t last_progress() const { return _last_progress; }

  std::int64_t balance() const { return _balance; }

  std::int64_t last_sync() const { return _last_sync; }

  std::int64_t last_processing() const { return _last_processing; }

  std::int64_t next_min_progress() const {
    return _last_progress + _min_progress + _balance;
  }

  std::int64_t max_progress() const { return _max_progress; }

  std::int64_t min_progress() const { return _min_progress; }

  unsigned sync_level() const { return _sync_level; }

  bool freewheel() const { return _sync_level > 4; }

  bool full_resync() const { return _sync_level > 2; }

  bool resync() const { return _sync_level > 0; }

  std::int64_t total_loss() const { return _total_loss; }

  std::int64_t safe_wakeup(std::int64_t oss_available) const {
    return next_min_progress() + buffer_frames() - oss_available -
           max_progress();
  }

  std::int64_t estimated_dropout(std::int64_t oss_available) const {
    return _last_progress + _balance + buffer_frames() - oss_available;
  }

  std::int64_t wakeup_time(std::int64_t sync_target,
                           std::int64_t oss_available) const {
    // Use one sync step by default.
    std::int64_t wakeup = _last_processing + Device::stepping();
    if (freewheel() || full_resync()) {
      // Small steps when doing a full resync.
    } else if (resync() || wakeup + max_progress() > sync_target) {
      // Sync required, wake up prior to next progress if possible.
      if (next_min_progress() > wakeup) {
        wakeup = next_min_progress() - Device::stepping();
      } else if (next_min_progress() > _last_processing) {
        wakeup = next_min_progress();
      }
    } else {
      // Sleep until prior to sync target, then sync again.
      wakeup = sync_target - max_progress();
    }
    // Make sure we wake up at sync target.
    if (sync_target > _last_processing && sync_target < wakeup) {
      wakeup = sync_target;
    }
    // Make sure we don't sleep into an OSS under- or overrun.
    if (_last_processing < safe_wakeup(oss_available) &&
        safe_wakeup(oss_available) < wakeup) {
      wakeup = std::max(safe_wakeup(oss_available),
                        _last_processing + Device::stepping());
    }
    return wakeup;
  }

protected:
  void mark_progress(std::int64_t progress, std::int64_t now) {
    if (progress > 0) {
      if (freewheel()) {
        // Some cards show irregular progress at the beginning, correct that.
        // Also correct loss after under- and overruns, assume same balance.
        _last_progress = now - progress - _balance;
        // Require a sync before transition back to normal processing.
        if (now <= _last_processing + stepping()) {
          _sync_level -= 1;
        }
      } else if (now <= _last_processing + stepping()) {
        // Successful sync on progress within small processing steps.
        _balance = now - (_last_progress + progress);
        _last_sync = now;
        if (_sync_level > 0) {
          _sync_level -= 1;
        }
        if (progress < _min_progress || _min_progress == 0) {
          _min_progress = progress;
        }
        if (progress > _max_progress) {
          _max_progress = progress;
        }
      } else {
        // Big step with progress but no sync, requires a resync.
        _sync_level += 1;
      }
      _last_progress += progress;
    }
    _last_processing = now;
  }

  std::int64_t mark_loss(std::int64_t progress, std::int64_t now) {
    // Approximate frames lost due to over- or underrun.
    std::int64_t loss = (now - _balance) - (_last_progress + progress);
    return mark_loss(loss);
  }

  std::int64_t mark_loss(std::int64_t loss) {
    if (loss > 0) {
      _total_loss += loss;
      // Resync OSS progress to frame time (now) to recover from loss.
      _sync_level = std::max(_sync_level, 6U);
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
  std::int64_t _total_loss = 0;
  unsigned _sync_level = 0;
};

} // namespace sosso

#endif // SOSSO_CHANNEL_HPP
