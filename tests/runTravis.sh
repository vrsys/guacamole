cd tests
g++-4.8 -std=c++0x -I../include -c testBoundingBox.cpp &&
g++-4.8 -std=c++0x -c main.cpp &&
g++-4.8 -std=c++0x testBoundingBox.o main.o -o tests -lunittest++
./tests
