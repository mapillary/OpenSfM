FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# Install apt-getable dependencies
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

RUN pip3 install cloudpickle==0.4.0 && \
    pip3 install exifread==2.1.2 && \
    pip3 install flask==2.3.2 && \
    pip3 install fpdf2==2.4.6 && \
    pip3 install joblib==0.14.1 && \
    pip3 install matplotlib && \
    pip3 install networkx==2.5 && \
    pip3 install numpy==1.21  && \
    pip3 install Pillow>=8.1.1 && \
    pip3 install pyproj>=1.9.5.1 && \
    pip3 install pytest==3.0.7 && \
    pip3 install python-dateutil>=2.7 && \
    pip3 install pyyaml==5.4 && \
    pip3 install scipy>=1.10.0 && \
    pip3 install Sphinx==4.2.0 && \
    pip3 install six && \
    pip3 install xmltodict==0.10.2 && \
    pip3 install wheel && \
    pip3 install opencv-python

RUN pip3 install jupyterlab && \
    pip3 install addict && \
    pip3 install fiona && \
    pip3 install trimesh && \
    pip3 install rasterio && \
    pip3 install manifold3d && \
    pip3 install -U scipy && \
    pip3 install pandas && \
    pip3 install shapely && \
    pip3 install tqdm

WORKDIR /source/OpenSfM

# COPY . /source/OpenSfM



# RUN pip3 install -r requirements.txt && \
#     python3 setup.py build
