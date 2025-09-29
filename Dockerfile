FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y \
        build-essential \
        cmake \
        git \
        libeigen3-dev \
        libopencv-dev \
        libceres-dev \
        python3-dev \
        python3-numpy \
        python3-opencv \
        python3-pip \
        python3-pyproj \
        python3-scipy \
        python3-yaml \
        curl \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN pip3 install \
        cloudpickle==0.4.0 \
        exifread==2.1.2 \
        flask==2.3.2 \
        fpdf2==2.4.6 \
        joblib==0.14.1 \
        matplotlib \
        networkx==2.5 \
        numpy>=1.19 \
        Pillow>=8.1.1 \
        pyproj>=1.9.5.1 \
        pytest==3.0.7 \
        python-dateutil>=2.7 \
        pyyaml>=5.4 \
        scipy>=1.10.0 \
        Sphinx==4.2.0 \
        xmltodict==0.10.2 \
        wheel \
        opencv-python

COPY . /source/OpenSfM

WORKDIR /source/OpenSfM

RUN python3 setup.py build
