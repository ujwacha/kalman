#!/bin/bash

g++ -I/usr/include/eigen3/ simulator.cpp  && ./a.out && gnuplot plotsctipt.gp
#g++ -I/usr/include/eigen3/ simulator.cpp  
