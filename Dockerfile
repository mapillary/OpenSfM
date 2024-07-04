#FROM quay.io/pypa/manylinux2014_x86_64
FROM ubuntu:20.04

ENV PLAT=manylinux_2_31_x86_64
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


#RUN yum install -y opencv opencv-devel \
#    blas-devel \
#    lapack-devel \
#    metis-devel \
#    tbb-devel \
#    wget \
#    openblas-devel \
#    atlas-devel \
#    suitesparse-devel

RUN apt-get update \
    && apt-get install -y \
        python3-dev \
        python3-numpy \
        python3-opencv \
        python3-pip \
        libblas-dev \
        liblapack-dev \
        libmetis-dev \
        libtbb-dev \
        wget \
        libopenblas-dev \
        libatlas-base-dev \
        git \
        cmake \
        build-essential \
        libsuitesparse-dev \
        curl

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

WORKDIR /source
RUN git clone https://github.com/ceres-solver/ceres-solver.git
WORKDIR /source/ceres-solver
RUN git checkout tags/2.0.0
RUN mkdir build
WORKDIR /source/ceres-solver/build/
RUN cmake ..
RUN make -j8
RUN make install

WORKDIR /source
RUN git clone https://github.com/NixOS/patchelf.git
WORKDIR /source/patchelf/
RUN git checkout tags/0.16.0
RUN ./bootstrap.sh
RUN ./configure
RUN make
RUN make install


RUN apt-get install -y liblzma-dev libssl-dev libopencv-dev
RUN apt-get install -y libncurses-dev  libffi-dev libreadline6-dev libbz2-dev libsqlite3-dev

WORKDIR $SFM_DIR
RUN curl https://pyenv.run | bash
RUN /root/.pyenv/bin/pyenv install 3.9.9
RUN /root/.pyenv/bin/pyenv global 3.9.9
ENV PATH=$PATH:/root/.pyenv/versions/3.9.9/bin/
RUN python -m pip install --upgrade pip

ENV WHEEL_DIR=/source/wheelhouse
ENV SFM_DIR=/source/OpenSfM
COPY . $SFM_DIR

WORKDIR $SFM_DIR
RUN rm -rf cmake_build
RUN /root/.pyenv/versions/3.9.9/bin/pip install -r requirements.txt
RUN /root/.pyenv/versions/3.9.9/bin/pip wheel $SFM_DIR --no-deps -w $WHEEL_DIR
RUN  ls /source/wheelhouse/*.whl | xargs -n 1 -I {}  auditwheel repair {} --plat $PLAT -w $WHEEL_DIR
RUN cd ${WHEEL_DIR} && rm -rf *-linux*whl
RUN /root/.pyenv/versions/3.9.9/bin/pip install opensfm --no-index -f $WHEEL_DIR
RUN python -c "import opensfm"
RUN ls /source/wheelhouse/*.whl | xargs -n 1 -I {} python -m twine upload --repository-url "http://pypi.artichoke-labs.ai" {}
