#!/bin/bash

find $(pwd)/include -name '*.h' -exec clang-format -i {} \;
find $(pwd)/include -name '*.cc' -exec clang-format -i {} \;

find $(pwd)/source -name '*.h' -exec clang-format -i {} \;
find $(pwd)/source -name '*.cc' -exec clang-format -i {} \;
