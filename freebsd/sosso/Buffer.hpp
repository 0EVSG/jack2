#ifndef SOSSO_BUFFER_HPP
#define SOSSO_BUFFER_HPP

#include <cstddef>
#include <cstring>

namespace sosso {

class Buffer {
public:
  Buffer() = default;

  Buffer(Buffer &&other)
      : _data(other._data), _position(other._position), _length(other._length) {
    other._data = nullptr;
    other._position = 0;
    other._length = 0;
  }

  Buffer &operator=(Buffer &&other) {
    _data = other._data;
    _position = other._position;
    _length = other._length;
    other._data = nullptr;
    other._position = 0;
    other._length = 0;
    return *this;
  }

  Buffer(char *buffer, std::size_t length)
      : _data(buffer), _position(0), _length(length) {}

  bool valid() const { return (_data != nullptr) && (_length > 0); }

  char *data() const { return _data; }

  std::size_t length() const { return _length; }

  char *position() const { return _data + _position; }

  std::size_t progress() const { return _position; }

  std::size_t remaining() const { return _length - _position; }

  std::size_t advance(std::size_t progress) {
    if (progress > remaining()) {
      progress = remaining();
    }
    _position += progress;
    return progress;
  }

  std::size_t rewind(std::size_t progress) {
    if (progress > _position) {
      progress = _position;
    }
    _position -= progress;
    return progress;
  }

  std::size_t erase(std::size_t begin, std::size_t end) {
    if (begin < _position && begin < end) {
      if (end > _position) {
        end = _position;
      }
      std::size_t copy = _position - end;
      if (copy > 0) {
        std::memmove(_data + begin, _data + end, copy);
      }
      _position -= (end - begin);
      return (end - begin);
    }
    return 0;
  }

  void reset() { _position = 0; }

private:
  char *_data = nullptr;
  std::size_t _position = 0;
  std::size_t _length = 0;
};

} // namespace sosso

#endif // SOSSO_BUFFER_HPP
