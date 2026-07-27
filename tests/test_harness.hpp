#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace test {

using Case = std::pair<std::string, std::function<void()>>;

inline std::vector<Case> &cases() {
  static std::vector<Case> value;
  return value;
}

struct Register {
  Register(std::string name, std::function<void()> function) {
    cases().emplace_back(std::move(name), std::move(function));
  }
};

inline void require(bool condition, const char *expression, const char *file,
                    int line) {
  if (!condition) {
    throw std::runtime_error(std::string(file) + ":" +
                             std::to_string(line) + " REQUIRE(" + expression +
                             ") failed");
  }
}

inline int run() {
  int failed = 0;
  for (const auto &[name, function] : cases()) {
    try {
      function();
      std::cout << "[PASS] " << name << '\n';
    } catch (const std::exception &e) {
      ++failed;
      std::cerr << "[FAIL] " << name << ": " << e.what() << '\n';
    }
  }
  std::cout << cases().size() - static_cast<std::size_t>(failed) << "/"
            << cases().size() << " tests passed\n";
  return failed == 0 ? 0 : 1;
}

} // namespace test

#define OVRS_JOIN_INNER(a, b) a##b
#define OVRS_JOIN(a, b) OVRS_JOIN_INNER(a, b)
#define TEST_CASE(name)                                                         \
  static void OVRS_JOIN(test_function_, __LINE__)();                           \
  static test::Register OVRS_JOIN(test_register_, __LINE__)(                   \
      name, OVRS_JOIN(test_function_, __LINE__));                              \
  static void OVRS_JOIN(test_function_, __LINE__)()
#define REQUIRE(expression)                                                     \
  test::require(static_cast<bool>(expression), #expression, __FILE__, __LINE__)
#define REQUIRE_NEAR(a, b, tolerance)                                           \
  test::require(std::abs((a) - (b)) <= (tolerance), #a " near " #b, __FILE__,  \
                __LINE__)
