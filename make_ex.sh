#!/bin/bash

# g++ -o test_geocalc main.cc geocalc.cc -L/usr/lib -I/usr/include/gdal

g++ -o test_geocalc main.cc geocalc.cc -lgdal -I/usr/include/gdal