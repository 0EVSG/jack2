#ifndef SOSSO_DEVICE_HPP
#define SOSSO_DEVICE_HPP

#include "sosso/Logging.hpp"
#include <easy/profiler.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>
#include <unistd.h>

namespace sosso {

class Device {
public:
  static std::size_t bytes_per_sample(int format) {
    switch (format) {
    case AFMT_S16_NE:
      return 2;
    case AFMT_S24_NE:
      return 3;
    case AFMT_S32_NE:
      return 4;
    default:
      return 0;
    }
  }

  ~Device() { close(); }

  std::size_t bytes_per_sample() const {
    return bytes_per_sample(_sample_format);
  }

  bool playback() const { return _fd >= 0 && (_file_mode & O_WRONLY); }

  bool recording() const { return _fd >= 0 && !playback(); }

  int file_descriptor() const { return _fd; }

  unsigned channels() const { return _channels; }

  std::size_t frame_size() const { return _channels * bytes_per_sample(); }

  std::size_t buffer_size() const { return _fragments * _fragment_size; }

  unsigned buffer_frames() const { return buffer_size() / frame_size(); }

  unsigned sample_rate() const { return _sample_rate; }

  unsigned stepping() const { return 16U * (1U + (_sample_rate / 50000)); }

  std::int64_t frames_to_time(std::int64_t frames) const {
    return (frames * 1000000000) / _sample_rate;
  }

  std::int64_t time_to_frames(std::int64_t time_ns) const {
    return (time_ns * _sample_rate) / 1000000000;
  }

  bool set_parameters(int format, int rate, int channels) {
    if (bytes_per_sample(format) && channels > 0) {
      _sample_format = format;
      _sample_rate = rate;
      _channels = channels;
      return true;
    }
    return false;
  }

  bool open(const char *device, int mode) {
    if (mode & O_RDWR) {
      Log::warn(SOSSO_LOC, "Only one direction allowed, open %s in read mode.",
                device);
      mode = O_RDONLY | (mode & O_EXCL) | (mode & O_NONBLOCK);
    }
    _fd = ::open(device, mode);
    if (_fd >= 0) {
      _file_mode = mode;
      if (bitperfect_mode(_fd) && set_sample_format(_fd) && set_channels(_fd) &&
          set_sample_rate(_fd) && get_buffer_info()) {
        return true;
      }
    }
    Log::warn(SOSSO_LOC, "Unable to open device %s, errno %d.", device, errno);
    close();
    return false;
  }

  bool set_buffer_size(unsigned fragments, unsigned fragment_size) {
    int frg = 0;
    while ((1U << frg) < fragment_size) {
      ++frg;
    }
    frg |= (fragments << 16);
    Log::info(SOSSO_LOC, "Request %d fragments of %u bytes.", (frg >> 16),
              (1U << (frg & 0xffff)));
    if (ioctl(_fd, SNDCTL_DSP_SETFRAGMENT, &frg) != 0) {
      Log::warn(SOSSO_LOC, "Set fragments failed with %d.", errno);
      return false;
    }
    return get_buffer_info();
  }

  bool set_buffer_size(unsigned total_size) {
    if (_fragment_size > 0) {
      unsigned fragments = (total_size + _fragment_size - 1) / _fragment_size;
      return set_buffer_size(fragments, _fragment_size);
    }
    return false;
  }

  bool read_to_buffer(char *buffer, std::size_t length, std::size_t &pos) {
    if (buffer && pos < length && recording()) {
      ssize_t result = ::read(_fd, buffer + pos, length - pos);
      if (result >= 0) {
        pos += result;
        return true;
      } else if (errno == EAGAIN) {
        return true;
      } else {
        Log::warn(SOSSO_LOC, "Data read failed with %d.", errno);
      }
    }
    return false;
  }

  int queued_samples() {
    unsigned long request =
        playback() ? SNDCTL_DSP_CURRENT_OPTR : SNDCTL_DSP_CURRENT_IPTR;
    oss_count_t ptr;
    if (ioctl(_fd, request, &ptr) == 0) {
      return ptr.fifo_samples;
    }
    return 0;
  }

  void close() {
    if (_fd >= 0) {
      ::close(_fd);
      _fd = -1;
    }
  }

  bool start() const {
    int trigger = recording() ? PCM_ENABLE_INPUT : PCM_ENABLE_OUTPUT;
    if (ioctl(file_descriptor(), SNDCTL_DSP_SETTRIGGER, &trigger) != 0) {
      const char *direction = recording() ? "recording" : "playback";
      Log::warn(SOSSO_LOC, "Starting %s channel failed with error %d.",
                direction, errno);
      return false;
    }
    return true;
  }

  bool add_to_sync_group(int &id) {
    oss_syncgroup sync_group = {0, 0, {0}};
    sync_group.id = id;
    sync_group.mode |= (recording() ? PCM_ENABLE_INPUT : PCM_ENABLE_OUTPUT);
    if (ioctl(file_descriptor(), SNDCTL_DSP_SYNCGROUP, &sync_group) == 0 &&
        (id == 0 || sync_group.id == id)) {
      id = sync_group.id;
      return true;
    }
    Log::warn(SOSSO_LOC, "Sync grouping channel failed with error %d.", errno);
    return false;
  }

  bool start_sync_group(int id) {
    if (ioctl(file_descriptor(), SNDCTL_DSP_SYNCSTART, &id) == 0) {
      return true;
    }
    Log::warn(SOSSO_LOC, "Start of sync group failed with error %d.", errno);
    return false;
  }

  bool get_errors(int &play_underruns, int &rec_overruns) {
    audio_errinfo error_info = {};
    if (ioctl(file_descriptor(), SNDCTL_DSP_GETERROR, &error_info) == 0) {
      play_underruns = error_info.play_underruns;
      rec_overruns = error_info.rec_overruns;
      return true;
    }
    return false;
  }

  int get_play_underruns() {
    int play_underruns = 0;
    int rec_overruns = 0;
    get_errors(play_underruns, rec_overruns);
    return play_underruns;
  }

  int get_rec_overruns() {
    int play_underruns = 0;
    int rec_overruns = 0;
    get_errors(play_underruns, rec_overruns);
    return rec_overruns;
  }

private:
  bool bitperfect_mode(int fd) {
    if (_file_mode & O_EXCL) {
      int flags = 0;
      int result = ioctl(fd, SNDCTL_DSP_COOKEDMODE, &flags);
      if (result < 0) {
        Log::warn(SOSSO_LOC, "Unable to set cooked mode.");
      }
      return result >= 0;
    }
    return true;
  }

  bool set_sample_format(int fd) {
    int format = _sample_format;
    int result = ioctl(fd, SNDCTL_DSP_SETFMT, &format);
    if (result != 0) {
      Log::warn(SOSSO_LOC, "Unable to set sample format, error %d.", errno);
      return false;
    } else if (bytes_per_sample(format) == 0) {
      Log::warn(SOSSO_LOC, "Unsupported sample format %d.", format);
      return false;
    } else if (format != _sample_format) {
      Log::warn(
          SOSSO_LOC, "Driver changed the sample format, %lu bit vs %lu bit.",
          bytes_per_sample(format) * 8, bytes_per_sample(_sample_format) * 8);
    }
    _sample_format = format;
    return true;
  }

  bool set_sample_rate(int fd) {
    int rate = _sample_rate;
    if (ioctl(fd, SNDCTL_DSP_SPEED, &rate) == 0) {
      if (rate != _sample_rate) {
        Log::warn(SOSSO_LOC, "Driver changed the sample rate, %d vs %d.", rate,
                  _sample_rate);
        _sample_rate = rate;
      }
      return true;
    }
    Log::warn(SOSSO_LOC, "Unable to set sample rate, error %d.", errno);
    return false;
  }

  bool set_channels(int fd) {
    int channels = _channels;
    if (ioctl(fd, SNDCTL_DSP_CHANNELS, &channels) == 0) {
      if (channels != _channels) {
        Log::warn(SOSSO_LOC, "Driver changed number of channels, %d vs %d.",
                  channels, _channels);
        _channels = channels;
      }
      return true;
    }
    Log::warn(SOSSO_LOC, "Unable to set channels, error %d.", errno);
    return false;
  }

  bool get_buffer_info() {
    audio_buf_info info = {0, 0, 0, 0};
    unsigned long request =
        playback() ? SNDCTL_DSP_GETOSPACE : SNDCTL_DSP_GETISPACE;
    if (ioctl(_fd, request, &info) >= 0) {
      _fragments = info.fragstotal;
      _fragment_size = info.fragsize;
      Log::info(SOSSO_LOC, "Buffer is %u fragments of size %u, %u frames.",
                _fragments, _fragment_size, buffer_frames());
      return true;
    } else {
      Log::warn(SOSSO_LOC, "Unable to get buffer info.");
      return false;
    }
  }

private:
  int _fd = -1;
  int _file_mode = O_RDONLY;
  int _channels = 2;
  int _sample_format = AFMT_S32_NE;
  int _sample_rate = 48000;
  unsigned _fragments = 0;
  unsigned _fragment_size = 0;
};

} // namespace sosso

#endif // SOSSO_DEVICE_HPP
