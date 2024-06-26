#!/bin/bash

find $(pwd)/include -name '*.h' -exec clang-format -i '{}' \;

find $(pwd)/source -name '*.h' -exec clang-format -i '{}' \;
find $(pwd)/source -name '*.cc' -exec clang-format -i '{}' \;

cppcheck --std=c++17 --quiet --enable=performance,portability $(pwd)/source
