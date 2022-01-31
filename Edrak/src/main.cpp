#include "Edrak/Exceptions/Exceptions.hpp"

int main(int argc, char const *argv[]) {
  throw Edrak::Exceptions::UnsupportedModeException();
  return 0;
}
