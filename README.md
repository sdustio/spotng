# sdquadx


## Coordinate
           x(front)
           |
(left)y -- . z


## Run Test

```sh
cmake -DSDQUADX_BUILD_TESTS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug -H$(pwd) -B$(pwd)/build
cmake --build $(pwd)/build --target all
cpplint --quiet --recursive $(pwd)/source
cppcheck --std=c++17 --quiet --enable=performance,portability $(pwd)/source
cd $(pwd)/build && ctest
```

## Installation

```sh
cmake -DCMAKE_INSTALL_PREFIX:STRING=$HOME/.local -H$(pwd) -B$(pwd)/build
cmake --build $(pwd)/build --target install
```

## Usage

see /test/example*_test.cc
