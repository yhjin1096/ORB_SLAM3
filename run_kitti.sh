#!/bin/bash
cd build
make -j4
../Examples/Monocular/mono_kitti ../Vocabulary/ORBvoc.txt ../Examples/Monocular/KITTI04-12.yaml /app/data/05