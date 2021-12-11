# sdquadx

## Installation

```sh
cmake -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INSTALL_PREFIX:STRING=$HOME/.local -H$(pwd) -B$(pwd)/build
cmake --build $(pwd)/build --target install -- -j4
```

## Usage

see /test/example.cc
