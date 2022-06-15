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

#### Install from linux package

Download linux package and install it using related package manager:

**Ubuntu/Debian**

```sh
curl -LO https://github.com/sdustio/spotng/releases/download/1.0.1/spotng_1.0.1_amd64.deb
sudo dpkg -i spotng_1.0.1_amd64.deb
```

**Redhat/CentOS**

```sh
curl -LO https://github.com/sdustio/spotng/releases/download/1.0.1/spotng-1.0.1-1.x86_64.rpm
sudo rpm -i spotng-1.0.1-1.x86_64.rpm
```

#### Install from source

**Requirements**

- vcpkg

**Build and Install**

assume $VCPKG_ROOT stands for root path of vcpkg

```sh
vcpkg install eigen3 spdlog
cmake -DCMAKE_INSTALL_PREFIX:STRING=/opt \
  -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake \
  -H$(pwd) -B$(pwd)/build-release
cmake --build $(pwd)/build-release --target install
```

## Usage

see /test/example*_test.cc


## Contribute to Spotng

1. setup develop environments, We use [Visual Studio Code](https://code.visualstudio.com/) [Remote Development Extension: Container](https://code.visualstudio.com/docs/remote/containers)

2. change some code

3. run linter and test as follows, make them pass

```sh
./fmt-lint.sh
cppcheck --std=c++17 --quiet --enable=performance,portability ./source
cmake -DSPOTNG_BUILD_TESTS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug \
  -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake \
  -H$(pwd) -B$(pwd)/build
cmake --build $(pwd)/build --target all
cd $(pwd)/build
ctest
ctest -T memcheck
```

4. create a pull request
