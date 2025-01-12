#!/bin/bash
echo "Building ORB_SLAM3 python wrapper"

set -e 

if [[ -z "$ORBSLAM3_PYTHON_EXECUTABLE" ]]; then
    echo "Must provide ORBSLAM3_PYTHON_EXECUTABLE in environment" 1>&2
    exit 1
fi

#rm build -r
mkdir build -p
cd build
cmake .. \
	-DCMAKE_BUILD_TYPE=Release \
	-Dpybind11_DIR=${CONDA_PREFIX}/share/cmake/pybind11/ \
	-DPYTHON_EXECUTABLE=${ORBSLAM3_PYTHON_EXECUTABLE}

make -j24 orbslam3_python
cd ..

PHD_LIB_DIR=/home/john/projects/research/research/slam/orbslam3/lib
cd lib
cp libORB_SLAM3.so orbslam3_python.so ${PHD_LIB_DIR}
