#!/bin/sh

# get directory of script and cd to its parent
DIR="$( cd "$( dirname "$0" )" && pwd )"
cd $DIR/..

# run initial cmake if neccessary
if [ ! -d "build" ]; then
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=release ..
    cd ..
fi

# buil it!
cd build
make -j16
