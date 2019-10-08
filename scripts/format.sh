#!/bin/bash

shopt -s globstar

cd ..

clang-format-6.0 -i --sort-includes=false -style=file **/*.?pp

echo "Codebase formatted"