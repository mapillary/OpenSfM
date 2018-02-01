FROM paulinus/opensfm-docker-base:python3

COPY . /source/OpenSfM

WORKDIR /source/OpenSfM

RUN pip3 install -r requirements.txt && \
    python3 setup.py build
