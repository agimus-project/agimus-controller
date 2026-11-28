#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

namespace agimus_controller {

/// String with inline storage of at most N characters. Never allocates.
template <std::size_t N>
class FixedString {
 public:
  FixedString() = default;

  /// Setup constructor: throws std::invalid_argument if `s` is too long.
  explicit FixedString(std::string_view s) {
    if (!assign(s)) {
      throw std::invalid_argument("FixedString: '" + std::string(s) +
                                  "' is longer than " + std::to_string(N) +
                                  " characters.");
    }
  }

  /// Copies `s`. Returns false and leaves the string unchanged if too long.
  /// @rt FixedString::assign
  bool assign(std::string_view s) noexcept {
    if (s.size() > N) {
      return false;
    }
    std::memcpy(data_.data(), s.data(), s.size());
    size_ = s.size();
    data_[size_] = '\0';
    return true;
  }

  /// @rt FixedString::view
  std::string_view view() const noexcept { return {data_.data(), size_}; }
  const char* c_str() const noexcept { return data_.data(); }
  std::size_t size() const noexcept { return size_; }
  bool empty() const noexcept { return size_ == 0; }
  static constexpr std::size_t capacity() noexcept { return N; }

  /// @rt FixedString::operator==
  friend bool operator==(const FixedString& a, const FixedString& b) noexcept {
    return a.view() == b.view();
  }
  friend bool operator==(const FixedString& a, std::string_view b) noexcept {
    return a.view() == b;
  }
  friend bool operator!=(const FixedString& a, const FixedString& b) noexcept {
    return !(a == b);
  }
  friend bool operator!=(const FixedString& a, std::string_view b) noexcept {
    return !(a == b);
  }

 private:
  std::array<char, N + 1> data_{};
  std::size_t size_ = 0;
};

/// Vector with inline storage of at most N elements. All N elements are
/// constructed up front, so elements keep their own storage (e.g. Eigen
/// dynamic vectors) across clear() and push_back(). Never allocates as long
/// as copying a T does not.
template <typename T, std::size_t N>
class FixedVector {
 public:
  /// Copies `value` into the next slot. Returns false when full.
  /// @rt FixedVector::push_back
  bool push_back(const T& value) {
    if (size_ == N) {
      return false;
    }
    data_[size_] = value;
    ++size_;
    return true;
  }

  /// @rt FixedVector::clear
  void clear() noexcept { size_ = 0; }

  /// @rt FixedVector::operator[]
  T& operator[](std::size_t i) noexcept {
    assert(i < size_);
    return data_[i];
  }
  const T& operator[](std::size_t i) const noexcept {
    assert(i < size_);
    return data_[i];
  }

  T* begin() noexcept { return data_.data(); }
  T* end() noexcept { return data_.data() + size_; }
  const T* begin() const noexcept { return data_.data(); }
  const T* end() const noexcept { return data_.data() + size_; }

  std::size_t size() const noexcept { return size_; }
  bool empty() const noexcept { return size_ == 0; }
  bool full() const noexcept { return size_ == N; }
  static constexpr std::size_t capacity() noexcept { return N; }

  friend bool operator==(const FixedVector& a, const FixedVector& b) {
    if (a.size_ != b.size_) {
      return false;
    }
    for (std::size_t i = 0; i < a.size_; ++i) {
      if (!(a.data_[i] == b.data_[i])) {
        return false;
      }
    }
    return true;
  }

 private:
  std::array<T, N> data_{};
  std::size_t size_ = 0;
};

}  // namespace agimus_controller
