/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <cstddef>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>

namespace mt_rrt {
namespace detail {
template <typename T> struct ToString {
  // 1. Fixed braces to parentheses for std::to_string call
  static std::string to_string(const T &ref) { return std::to_string(ref); }
};

template <> struct ToString<std::string> {
  static const std::string &to_string(const std::string &ref) { return ref; }
};

template <> struct ToString<std::string_view> {
  static std::string_view to_string(const std::string_view &ref) { return ref; }
};

// 2. Fixed syntax for passing a C-style array by reference
template <std::size_t Len> struct ToString<char[Len]> {
  static std::string_view to_string(const char (&ref)[Len]) {
    // 3. Prevent including the null-terminator '\0' in string_view
    return {ref, Len > 0 ? Len - 1 : 0};
  }
};
} // namespace detail

/**
 * replacement class for compilers that does not support std::format
 */
template <typename... Args> struct Format {
  Format(std::string_view format, const Args &...args)
      : format_{format}, args_{args...} {}

  std::string to_string() const {
    std::string res;
    add_<0>(res, format_);
    return res;
  }

  void print(std::ostream &recipient) const { add_<0>(recipient, format_); }

private:
  template <typename Recipient, typename ToAdd>
  static void add__(Recipient &recipient, const ToAdd &to_add) {
    if constexpr (std::is_same_v<Recipient, std::string>) {
      recipient += detail::ToString<ToAdd>::to_string(to_add);
    } else {
      recipient << to_add;
    }
  }

  template <std::size_t Idx, typename Recipient>
  void add_(Recipient &recipient, std::string_view format_rest) const {
    if constexpr (Idx < std::tuple_size_v<decltype(args_)>) {
      auto next_pos = format_rest.find("{}");
      if (next_pos == std::string::npos) {
        throw std::runtime_error{"Invalid format!"};
      }
      if (0 < next_pos) {
        add__(recipient, std::string_view{format_rest.data(), next_pos});
      }

      using TypeAtIdx = std::tuple_element_t<Idx, decltype(args_)>;
      const TypeAtIdx &ref = std::get<Idx>(args_);
      add__(recipient, ref);
      format_rest = {format_rest.data() + next_pos + 2,
                     format_rest.size() - next_pos - 2};

      add_<Idx + 1>(recipient, format_rest);
    } else {
      add__(recipient, format_rest);
    }
  }

  std::string_view format_;
  std::tuple<const Args &...> args_;
};
} // namespace mt_rrt

namespace std {
template <typename... Args>
std::ofstream &operator<<(std::ofstream &strm,
                          const mt_rrt::Format<Args...> &fmrt) {
  fmrt.print(strm);
  return strm;
}
} // namespace std
