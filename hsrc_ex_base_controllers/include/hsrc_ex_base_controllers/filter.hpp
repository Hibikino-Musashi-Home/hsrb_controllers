/// @file filter.hpp
/// @brief (速度指令用）フィルタ
/// @copyright Copyright (C) 2018 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_FILTER_HPP_
#define HSRC_EX_BASE_CONTROLLERS_FILTER_HPP_

#include <cstddef>
#include <algorithm>
#include <vector>

namespace hsrc_ex_base_controllers {

// matlabの下記関数と仕様を同じにしてある
// https://jp.mathworks.com/help/matlab/ref/filter.html
// Y(z)={b[1]+b[2]z^-1 +...+b[nb+1]z^−nb}/{1+a[2]z^−1+...+a[na-1]z^-na} * X(z)
// y[n]=b[1]x[n]+b[2]x[n−1]+...+b[nb+1]x[n-nb]−a[2]y[n−1]−...−a[na-1]y[n-na]
//  ただしb[1](matlab)->b[0](c++), a[1](matlab)->a[0](c++), a[0] = 1.0
// Tは入出力の型, Vは内部の型(基本double)
template<typename T = double, typename V = double>
class Filter {
 public:
  Filter() {
    a_.resize(1);
    b_.resize(1);
    a_[0] = 1.0;
    b_[0] = 1.0;
    reset(0.0);
  }
  Filter(const std::vector<V>& a, const std::vector<V>& b)
      : a_(a), b_(b) {
    // パラメータチェックは基本上でやる
    assert(a_.size() > 0);
    assert(b_.size() > 0);
    reset(0.0);
  }
  virtual ~Filter() {}

  // 内部状態を一定値リセットする
  void reset(const T& value) {
    x_.resize(b_.size());
    y_.resize(a_.size());
    std::fill(x_.begin(), x_.end(), static_cast<V>(value));
    std::fill(y_.begin(), y_.end(), static_cast<V>(value));
  }

  // フィルタをかける
  T update(const T& x) {
    V y = 0.0;
    // a[0]は使わない
    for (size_t i = a_.size() - 1; i > 0; --i) {
      y_[i] = y_[i - 1];
      y -= a_[i] * y_[i];
    }
    for (size_t i = b_.size() - 1; i > 0; --i) {
      x_[i] = x_[i - 1];
      y += b_[i] * x_[i];
    }
    y += b_[0] * x;
    x_[0] = static_cast<V>(x);
    y_[0] = static_cast<V>(y);
    return static_cast<T>(y);
  }

 private:
  std::vector<V> a_;
  std::vector<V> b_;
  std::vector<V> y_;
  std::vector<V> x_;
};

}  // namespace hsrc_ex_base_controllers

#endif
