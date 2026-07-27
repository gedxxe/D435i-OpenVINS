#pragma once

#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ovrs {

inline std::vector<std::string>
simple_yaml_scalars(const std::string &yaml, const std::string &key) {
  std::istringstream input(yaml);
  std::string line;
  const std::string prefix = key + ":";
  std::vector<std::string> values;
  while (std::getline(input, line)) {
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos ||
        line.compare(first, prefix.size(), prefix) != 0) {
      continue;
    }
    const auto value_position = first + prefix.size();
    if (value_position < line.size() &&
        line[value_position] != ' ' && line[value_position] != '\t' &&
        line[value_position] != '\r') {
      continue;
    }
    std::string value = line.substr(value_position);
    const auto comment = value.find('#');
    if (comment != std::string::npos) {
      value.resize(comment);
    }
    const auto begin = value.find_first_not_of(" \t\"'");
    const auto end = value.find_last_not_of(" \t\"'\r");
    values.push_back(begin == std::string::npos
                         ? std::string{}
                         : value.substr(begin, end - begin + 1));
  }
  return values;
}

inline std::string simple_yaml_scalar(const std::string &yaml,
                                      const std::string &key) {
  const auto values = simple_yaml_scalars(yaml, key);
  return values.empty() ? std::string{} : values.front();
}

inline std::size_t simple_yaml_key_count(const std::string &yaml,
                                         const std::string &key) {
  std::istringstream input(yaml);
  std::string line;
  const std::string prefix = key + ":";
  std::size_t count = 0;
  while (std::getline(input, line)) {
    const auto first = line.find_first_not_of(" \t");
    if (first != std::string::npos &&
        line.compare(first, prefix.size(), prefix) == 0) {
      const auto value_position = first + prefix.size();
      if (value_position < line.size() &&
          line[value_position] != ' ' && line[value_position] != '\t' &&
          line[value_position] != '\r') {
        continue;
      }
      ++count;
    }
  }
  return count;
}

inline int parse_int_strict(const std::string &text,
                            const std::string &field) {
  std::size_t consumed = 0;
  const int value = std::stoi(text, &consumed);
  if (text.empty() || consumed != text.size()) {
    throw std::invalid_argument(field + " must be an integer");
  }
  return value;
}

inline std::uint64_t parse_uint64_strict(const std::string &text,
                                         const std::string &field) {
  if (text.empty() || text.front() == '-') {
    throw std::invalid_argument(field + " must be an unsigned integer");
  }
  std::size_t consumed = 0;
  const auto value = std::stoull(text, &consumed);
  if (consumed != text.size()) {
    throw std::invalid_argument(field + " must be an unsigned integer");
  }
  return value;
}

inline double parse_double_strict(const std::string &text,
                                  const std::string &field) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (text.empty() || consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(field + " must be a finite number");
  }
  return value;
}

inline bool parse_double_list_strict(const std::string &text,
                                     std::size_t expected_size,
                                     std::vector<double> *values) {
  std::istringstream input(text);
  char delimiter = '\0';
  input >> std::ws >> delimiter;
  if (delimiter != '[') {
    return false;
  }
  std::vector<double> parsed;
  parsed.reserve(expected_size);
  for (std::size_t index = 0; index < expected_size; ++index) {
    double value = 0.0;
    if (!(input >> std::ws >> value) || !std::isfinite(value)) {
      return false;
    }
    parsed.push_back(value);
    input >> std::ws >> delimiter;
    if (index + 1 < expected_size) {
      if (delimiter != ',') {
        return false;
      }
    } else if (delimiter != ']') {
      return false;
    }
  }
  input >> std::ws;
  if (!input.eof()) {
    return false;
  }
  if (values) {
    *values = std::move(parsed);
  }
  return true;
}

} // namespace ovrs
