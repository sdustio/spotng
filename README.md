# Spotng

[![test](https://github.com/sdustio/spotng/actions/workflows/test.yml/badge.svg)](https://github.com/sdustio/spotng/actions/workflows/test.yml)
[![release](https://github.com/sdustio/spotng/actions/workflows/release.yml/badge.svg)](https://github.com/sdustio/spotng/actions/workflows/release.yml)


## Conventions

#### Coordinate
           x(front)
           |
(left)y -- . z

#### Euler Angle
Use ZYX, which means `RPY` will rotated in order of Yaw Pitch Roll

## Install

#### Install from source

**Requirements**

- vcpkg

**Build and Install**

assume $VCPKG_ROOT stands for root path of vcpkg

```sh
cmake -DCMAKE_INSTALL_PREFIX:STRING=$HOME/.local \
  -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake \
  -H$(pwd) -B$(pwd)/build-release
cmake --build $(pwd)/build-release --target install
```

## Usage

see /test/example*_test.cc


## Contribute to Spotng

- install valgrind clang-format cppcheck cpplint(pip)
- change some code
- run linter and test as follows, make them pass
- create a pull request

```sh
cmake -DSPOTNG_BUILD_TESTS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug \
  -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake \
  -H$(pwd) -B$(pwd)/build-test
cmake --build $(pwd)/build-test --target all
cd $(pwd)/build-test
ctest
ctest -T memcheck
./fmt-lint.sh
```
