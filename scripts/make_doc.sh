#!/bin/sh

# get directory of script and cd to doc folder
DIR="$( cd "$( dirname "$0" )" && pwd )"
cd $DIR/../doc

doxygen

cp *.css doc/html
cp *.png doc/html
