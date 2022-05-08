#include <cassert>
#include <chrono>
#include <iostream>
#include <string>
namespace Edrak {
namespace Benchmark {
class Timer {
public:
  Timer(const std::string text)
      : text_(text), t_start_(std::chrono::steady_clock::now()) {}

  void Stop() {

    assert(!stoped_);
    t_end_ = std::chrono::steady_clock::now();
    t_elapsed_ = std::chrono::duration_cast<std::chrono::duration<double>>(
        t_end_ - t_start_);
    std::cout << "======= Timing " << text_ << "Took " << t_elapsed_.count()
              << " Seconds ========" << std::endl;
    stoped_ = true;
  }
  void Restart(const std::string text = "") {
    if (!text.empty()) {
      text_ = text;
    }
    t_start_ = std::chrono::steady_clock::now();
    stoped_ = false;
  }
  ~Timer() {
    if (!stoped_)
      Stop();
  }

private:
  std::string text_;
  bool stoped_ = false;
  std::chrono::steady_clock::time_point t_start_;
  std::chrono::steady_clock::time_point t_end_;
  std::chrono::duration<double> t_elapsed_;
};
} // namespace Benchmark
} // namespace Edrak
