FROM quay.io/pypa/manylinux2014_x86_64
# FROM ubuntu:20.04

ENV PLAT=manylinux_2_17_x86_64
ARG DEBIAN_FRONTEND=noninteractive
ARG TWINE_USERNAME
ARG TWINE_PASSWORD
ENV TWINE_USERNAME=$TWINE_USERNAME
ENV TWINE_PASSWORD=$TWINE_PASSWORD

#RUN apt-get update \
#    && apt-get install -y \
#        build-essential \
#        cmake \
#        git \
#        libeigen3-dev \
#        libopencv-dev \
#        libceres-dev \
#        python3-dev \
#        python3-numpy \
#        python3-opencv \
#        python3-pip \
#        python3-pyproj \
#        python3-scipy \
#        python3-yaml \
#        curl \
#    && apt-get clean \
#    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \

# Install apt-getable dependencies
#RUN yum install -y \
#        build-essential \
#        cmake \
#        git \
#        libeigen3-dev \
#        libopencv-dev \
#        libceres-dev \
#        python3-dev \
#        python3-numpy \
#        opencv \
#        python3-pip \
#        python3-pyproj \
#        python3-scipy \
#        python3-yaml \
#        curl \
#    && yum clean all \

## Установим Miniconda
#RUN curl -sSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o miniconda.sh && \
#    bash miniconda.sh -b -p /opt/conda && \
#    rm miniconda.sh
#
## Добавим conda в PATH
#ENV PATH=/opt/conda/bin:$PATH
#
## Установим Ceres Solver 2.1 или 2.2 через conda
#RUN conda install -c conda-forge ceres-solver=2.2 opencv
#ENV CMAKE_PREFIX_PATH="/opt/conda/pkgs/ceres-solver-2.2.0-hfae76b8_3/lib/cmake/Ceres/"

#RUN /opt/python/cp39-cp39/bin/pip install --upgrade pip && \
#    /opt/python/cp39-cp39/bin/pip install auditwheel
#
##RUN yum install -y opencv opencv-devel \
##    ceres-solver ceres-solver-devel \
##    eigen3 \
##    glog-devel \
##    gflags-devel \
##    suitesparse-devel \
##    blas-devel \
##    lapack-devel \
##    openblas-devel
#
RUN yum install -y opencv opencv-devel \
    blas-devel \
    lapack-devel \
    metis-devel \
    tbb-devel \
    wget \
    openblas-devel \
    atlas-devel

RUN mkdir /source
WORKDIR /source
RUN git clone https://github.com/gflags/gflags.git
WORKDIR /source/gflags/
RUN git checkout tags/v2.2.2
RUN mkdir /source/gflags/build/
WORKDIR /source/gflags/build/
RUN cmake  -DCMAKE_CXX_FLAGS="-fPIC" ..
RUN make -j8
RUN make install

WORKDIR /source
RUN git clone https://github.com/google/glog.git
WORKDIR /source/glog
RUN git checkout tags/v0.6.0
RUN mkdir /source/glog/build/
WORKDIR /source/glog/build/
RUN cmake ..
RUN make -j8
RUN make install

WORKDIR /source
RUN git clone https://gitlab.com/libeigen/eigen.git
WORKDIR /source/eigen/
RUN git checkout tags/3.4.0
RUN mkdir build
WORKDIR /source/eigen/build/
RUN cmake ..
RUN make install

#RUN yum install -y lzip
#WORKDIR /source
#RUN wget https://gmplib.org/download/gmp/gmp-6.3.0.tar.lz
#RUN tar -xf gmp-6.3.0.tar.lz
#WORKDIR /source/gmp-6.3.0
#RUN ./configure
#RUN make
#RUN make install
#
#WORKDIR /source
#RUN wget https://www.mpfr.org/mpfr-current/mpfr-4.2.1.tar.xz
#RUN tar -xf mpfr-4.2.1.tar.xz
#WORKDIR /source/mpfr-4.2.1
#RUN ./configure
#RUN make
#RUN make install

#WORKDIR /source
#RUN git clone https://github.com/DrTimothyAldenDavis/SuiteSparse.git
#WORKDIR /source/SuiteSparse
#RUN git checkout tags/v7.7.0
#WORKDIR /source/SuiteSparse/build/
#RUN cmake .. -DBLAS_LIBRARIES=/usr/lib64/libopenblas.so -DLAPACK_LIBRARIES=/usr/lib64/liblapack.so
#RUN cmake --build .
#RUN cmake install

#WORKDIR /source
#RUN git clone https://github.com/google/googletest.git
#WORKDIR /source/googletest
#RUN git checkout tags/release-1.8.1
#RUN mkdir build
#WORKDIR /source/googletest/build
#RUN cmake ..
#RUN make -j4
#RUN make install

RUN yum install -y suitesparse-devel
WORKDIR /source
RUN git clone https://github.com/ceres-solver/ceres-solver.git
WORKDIR /source/ceres-solver
RUN git checkout tags/2.0.0
RUN mkdir build
WORKDIR /source/ceres-solver/build/
RUN cmake ..
RUN make -j8
RUN make install


ENV WHEEL_DIR=/source/wheelhouse
ENV SFM_DIR=/source/OpenSfM
COPY . $SFM_DIR

WORKDIR $SFM_DIR
RUN rm -rf cmake_build
RUN sh /source/OpenSfM/build_wheel.sh
# RUN sh /source/OpenSfM/test_and_upload_wheel.sh
