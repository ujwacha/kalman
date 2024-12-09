#!/bin/bash

g++ -I/usr/include/eigen3/ simulator.cpp  && ./a.out > data.txt && gnuplot plotsctipt.gp
