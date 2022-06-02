# Edrak
A toolbox for autonomous perception 

[![CMake](https://github.com/HemaZ/Edrak/actions/workflows/conan.yml/badge.svg)](https://github.com/HemaZ/Edrak/actions/workflows/conan.yml)


## How to build

[Conan](https://conan.io/) package manager is used to install all the dependencies. Please install it first from [here](https://conan.io/downloads.html)

```console
$ mkdir build && cd build
$ conan install .. -s build_type=Debug --build missing
$ cmake .. -DCMAKE_BUILD_TYPE=DEBUG
$ make
```
