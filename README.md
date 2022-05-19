# sdengine


## Conventions

#### Coordinate
           x(front)
           |
(left)y -- . z

#### Euler Angle
Use ZYX, which means `RPY` will rotated in order of Yaw Pitch Roll


## Run Test

```sh
cmake -DSDENGINE_BUILD_TESTS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug -H$(pwd) -B$(pwd)/build-test
cmake --build $(pwd)/build-test --target all
cpplint --quiet --recursive $(pwd)/source
cppcheck --std=c++17 --quiet --enable=performance,portability $(pwd)/source
cd $(pwd)/build-test
ctest
ctest -T memcheck
```

## Installation

```sh
cmake -DCMAKE_INSTALL_PREFIX:STRING=$HOME/.local -H$(pwd) -B$(pwd)/build-release
cmake --build $(pwd)/build-release --target install
```

## Usage

see /test/example*_test.cc
