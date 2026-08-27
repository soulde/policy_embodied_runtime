#pragma once

#include <string>
#include <utility>
#include <variant>

namespace policy_runtime {

enum class ErrorCode {
  invalid_argument,
  io,
  timeout,
  protocol,
  unavailable,
  internal,
};

struct Error {
  ErrorCode code;
  std::string message;
};

template <class T>
class Result {
 public:
  static Result success(T value) { return Result(std::move(value)); }

  static Result failure(Error error) { return Result(std::move(error)); }

  bool has_value() const noexcept { return std::holds_alternative<T>(storage_); }

  T& value() { return std::get<T>(storage_); }

  const Error& error() const { return std::get<Error>(storage_); }

 private:
  explicit Result(T value) : storage_(std::move(value)) {}

  explicit Result(Error error) : storage_(std::move(error)) {}

  std::variant<T, Error> storage_;
};

template <>
class Result<void> {
 public:
  static Result success() { return Result(std::monostate{}); }

  static Result failure(Error error) { return Result(std::move(error)); }

  bool has_value() const noexcept { return std::holds_alternative<std::monostate>(storage_); }

  void value() const { std::get<std::monostate>(storage_); }

  const Error& error() const { return std::get<Error>(storage_); }

 private:
  explicit Result(std::monostate value) : storage_(value) {}

  explicit Result(Error error) : storage_(std::move(error)) {}

  std::variant<std::monostate, Error> storage_;
};

}  // namespace policy_runtime
