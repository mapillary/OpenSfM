docker run -it --rm --net host  \
--mount type=bind,source=/home/user/work/OpenSfM-ConePairing,target=/source/OpenSfM  \
--mount type=bind,source=/home/user/work/opensfm_pairing_draft,target=/source/opensfm_pairing_draft  \
opensfmc