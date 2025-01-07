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

cp lib/libORB_SLAM3.so ~/projects/research/orbslam3_wrapper/lib/libORB_SLAM3.so
cp lib/orbslam3_python.so ~/projects/research/orbslam3_wrapper/lib/orbslam3.so
