#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#if _WIN64 || _WIN32
#include <windows.h>
#endif

void setUpEnv() {
  std::stringstream ss(ENV);
  std::string token;
  while (ss >> token) {
    std::size_t sep = token.find('=');
    std::string key = std::string{token, 0, sep};
    std::string val = std::string{token, sep + 1};
#if _WIN64 || _WIN32
    SetEnvironmentVariable(key.c_str(), val.c_str());
#elif __linux__
    setenv(key.c_str(), val.c_str(), 1);
#endif
  }
}

struct Command {
  template <typename Arg>
  static void addArg_(std::stringstream &buff, Arg &&arg) {
    buff << arg;
  }

  template <typename... Args>
  Command(const std::string &executable, Args &&...args)
      : executable_{executable} {
    std::stringstream buff;
    (this->addArg_<Args>(buff, std::forward<Args>(args)), ...);
    args_ = buff.str();
  }

  std::string executable_;
  std::string args_;

  void run() const {
    static const std::size_t BUFFER_SIZE = 128;

    std::string cmd = executable_ + " " + args_;

    std::cout << "running `" << cmd << '`' << std::endl << std::endl;

    auto throw_exc = [&cmd]() {
      std::stringstream msg;
      msg << "Something went wrong running `" << cmd << '`';
      throw std::runtime_error{msg.str()};
    };

    std::string buffer_str;
    buffer_str.resize(BUFFER_SIZE);
    FILE *fp =
#if _WIN64 || _WIN32
        _popen(cmd.c_str(), "r")
#elif __linux__
        popen(cmd.c_str(), "r")
#endif
        ;
    if (fp == NULL) {
      throw_exc();
    }

    while (fgets(buffer_str.data(), BUFFER_SIZE, fp) != NULL) {
      //std::cout << buffer_str;
    }

#if _WIN64 || _WIN32
    feof(fp);
#endif

    int return_code =
#if _WIN64 || _WIN32
        _pclose(fp)
#elif __linux__
        pclose(fp)
#endif
        ;
    if (return_code != 0) {
      throw_exc();
    }
  }
};

int main() {
  setUpEnv();

  Command{BIN_PATH, ARGS}.run();
  Command{PYTHON_CMD, SCRIPT, ARGS_SCRIPT}.run();

  return EXIT_SUCCESS;
}
