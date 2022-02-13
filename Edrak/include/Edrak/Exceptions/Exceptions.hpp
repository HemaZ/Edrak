#ifndef __EXCEPTIONS_H__
#define __EXCEPTIONS_H__
#include <exception>
#include <stdexcept>
namespace Edrak {
namespace Exceptions {
class UnsupportedModeException : public std::invalid_argument {
public:
  UnsupportedModeException(
      const std::string &_reason = "Invalid or unsupported Type/Mode.")
      : std::invalid_argument(_reason) {}
};
class InvalidDimensions : public std::invalid_argument {
public:
  InvalidDimensions(const std::string &_reason = "Invalid dimensions detected")
      : std::invalid_argument(_reason) {}
};
class FileNotFound : public std::runtime_error {
public:
  FileNotFound(const std::string &_reason = "[Edrak] File Not found Exception")
      : std::runtime_error(_reason) {}
};
class WrongSize : public std::runtime_error {
public:
  WrongSize(const std::string &_reason = "[Edrak] File Not found Exception")
      : std::runtime_error(_reason) {}
};
} // namespace Exceptions

} // namespace Edrak

#endif // __EXCEPTIOnamespNS_H__