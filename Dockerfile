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
        ExifRead==2.1.2 \
        Flask==2.3.2 \
        fpdf2==2.4.6 \
        joblib==0.14.1 \
        matplotlib==3.7.5 \
        networkx==2.5 \
        numpy==1.24.4 \
        opencv-python==4.12.0.88 \
        pillow==10.4.0 \
        pyproj==2.5.0 \
        pytest==3.0.7 \
        python-dateutil==2.9.0.post0 \
        PyYAML==6.0.3 \
        scipy==1.10.1 \
        Sphinx==4.2.0 \
        xmltodict==0.10.2


COPY . /source/OpenSfM

WORKDIR /source/OpenSfM

RUN python3 setup.py build
