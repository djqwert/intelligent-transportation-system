#!/bin/bash

chmod -R u+rwx *
cd ./G1;
make -j5 TARGET=sky clean;
make -j5 TARGET=sky;
cd ..
cd ./G2;
make -j5 TARGET=sky clean;
make -j5 TARGET=sky;
cd ..
cd ./TL;
make -j5 TARGET=sky clean;
make -j5 TARGET=sky;