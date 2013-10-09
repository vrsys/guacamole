#!/bin/sh

mkdir build

cd build; cmake -DCMAKE_BUILD_TYPE=release ..; make -j16 && ( cd .. )
