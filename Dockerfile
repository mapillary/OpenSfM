FROM paulinus/opensfm-docker-base

COPY requirements.txt /source/OpenSfM/requirements.txt
RUN pip install -r /source/OpenSfM/requirements.txt

COPY . /source/OpenSfM
WORKDIR /source/OpenSfM

RUN python setup.py build
