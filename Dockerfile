FROM ubuntu:14.04


# Install apt-getable dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git wget \
    python-dev python-pip libboost-python-dev \
    python-numpy python-scipy python-yaml \
    libopencv-dev python-opencv \
    libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


# Install Ceres from source
RUN \
    mkdir -p /source && cd /source && \
    wget http://ceres-solver.org/ceres-solver-1.10.0.tar.gz && \
    tar xvzf ceres-solver-1.10.0.tar.gz && \
    cd /source/ceres-solver-1.10.0 && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_C_FLAGS=-fPIC -DCMAKE_CXX_FLAGS=-fPIC -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DOPENMP=OFF && \
    make install && \
    cd / && \
    rm -rf /source/ceres-solver-1.10.0 && \
    rm -f /source/ceres-solver-1.10.0.tar.gz


# Install opengv from source
RUN \
    mkdir -p /source && cd /source && \
    git clone https://github.com/paulinus/opengv.git && \
    cd /source/opengv && \
    git checkout python-wrapper && \
    mkdir -p build && cd build && \
    cmake .. -DBUILD_TESTS=OFF -DBUILD_PYTHON=ON && \
    make install && \
    cd / && \
    rm -rf /source/opengv


# OpenSfM
RUN \
    mkdir -p /source && cd /source && \
    git clone https://github.com/mapillary/OpenSfM.git && \
    cd /source/OpenSfM && \
    pip install -r requirements.txt && \
    python setup.py build && \
    cd /


# GENERIC
WORKDIR /
