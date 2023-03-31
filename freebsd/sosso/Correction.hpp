#ifndef SOSSO_CORRECTION_HPP
#define SOSSO_CORRECTION_HPP

#include <cstddef>

namespace sosso {

class Correction {
public:
  Correction() = default;

  void set_drift_limits(std::int64_t drift_min, std::int64_t drift_max) {
    if (drift_min < drift_max) {
      _drift_min = drift_min;
      _drift_max = drift_max;
    } else {
      _drift_min = drift_max;
      _drift_max = drift_min;
    }
  }

  void set_loss_limits(std::int64_t loss_min, std::int64_t loss_max) {
    if (loss_min < loss_max) {
      _loss_min = loss_min;
      _loss_max = loss_max;
    } else {
      _loss_min = loss_max;
      _loss_max = loss_min;
    }
  }

  std::int64_t correction() const { return _correction; }

  std::int64_t correct(std::int64_t balance, std::int64_t target = 0) {
    std::int64_t corrected_balance = balance - target + _correction;
    if (corrected_balance > _loss_max) {
      _correction -= corrected_balance - _loss_max;
    } else if (corrected_balance < _loss_min) {
      _correction += _loss_min - corrected_balance;
    } else if (corrected_balance > _drift_max) {
      _correction -= 1;
    } else if (corrected_balance < _drift_min) {
      _correction += 1;
    }
    return _correction;
  }

  void clear() {
    _correction = 0;
  }

private:
  std::int64_t _loss_min = -128;
  std::int64_t _loss_max = 128;
  std::int64_t _drift_min = -64;
  std::int64_t _drift_max = 64;
  std::int64_t _correction = 0;
};

} // namespace sosso

#endif // SOSSO_CORRECTION_HPP
