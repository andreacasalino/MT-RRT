/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>

namespace mt_rrt {
/**
 * replacement class for compilers that does not support std::format
 */
template <typename... Args> struct Format {
  Format(std::string_view format, const Args &...args)
      : format_{format}, args_{args} {}

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
      if constexpr (std::is_same_v<ToAdd, std::string> ||
                    std::is_same_v<ToAdd, std::string_view>) {
        recipient += to_add;
      } else {
        recipient += std::to_string(to_add);
      }
    } else {
      recipient << to_add;
    }
  }

  template <std::size_t Idx, typename Recipient>
  static void add_(Recipient &recipient, std::string_view format_rest) {
    auto next_pos = format_rest.find("{}");
    if (next_pos == std::string::npos) {
      throw std::runtime_error{"Invalid format!"};
    }
    if (0 < next_pos) {
      add__(recipient, std::string_view{format_rest.data(), next_pos});
    }
    add__(recipient, std::get<Idx>(args_));
    format_rest = {format_rest.data() + next_pos + 2,
                   format_rest.size() - next_pos - 2};
    if constexpr (Idx < std::tuple_size_v<decltype(args_)>) {
      add_<Idx + 1>(recipient, format_rest);
    } else {
      add__(recipient, format_rest);
    }
  }

  std::string_view format_;
  std::tie<Args...> args_;
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
