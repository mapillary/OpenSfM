import numpy as np
import networkx as nx
import opensfm.reconstruction


def get_reconstruction():
    return {
        "shots": {
            "02.jpg": {
                "orientation": 1,
                "accelerometer": [ 0.9766708, -0.01353197, 0.2143152 ],
                "compass": { "angle": 60.50159, "accuracy": 10 },
                "camera": "v2 apple iphone 4s back camera 4.28mm f/2.4 3264 2448 perspective 0.9722",
                "gps_dop": 5.0,
                "rotation": [ 1.3023920748972277, -0.39876276438278901, 0.52073620735582438 ],
                "translation": [ 0.075308840686505188, 2.0243909580054082, 0.38021303490795211 ],
                "gps_position": [ 0.8171305709135197, -4.2240837056851888, 1.9999985732138157 ],
            },
            "01.jpg": {
                "orientation": 1,
                "accelerometer": [ 0.9466457, 0.01739691, 0.3218063 ],
                "compass": { "angle": 54.74463, "accuracy": 10 },
                "camera": "v2 apple iphone 4s back camera 4.28mm f/2.4 3264 2448 perspective 0.9722",
                "gps_dop": 5.0,
                "rotation": [ 1.2178347158182881, -0.39586089498470911, 0.55343866151371035 ],
                "translation": [ -0.25502760140629521, 3.9848086288723663, 5.396371729191662 ],
                "gps_position": [ -5.5938624256460514, -5.1513907870939875, 1.9999954858794808 ]
            },
            "03.jpg": {
                "orientation": 1,
                "accelerometer": [ 0.9116519, -0.007386226, 0.4108968 ],
                "compass": { "angle": 61.76562, "accuracy": 10 },
                "camera": "v2 apple iphone 4s back camera 4.28mm f/2.4 3264 2448 perspective 0.9722",
                "gps_dop": 5.0,
                "rotation": [ 1.0889876182043234, -0.37872140775966501, 0.55044900143952558 ],
                "translation": [ 0.1885250822577707, -1.2491779194125316, -7.3414521921519205 ],
                "gps_position": [ 4.7768476283690688, 9.376468512815336, 1.9999913610517979 ]
            }
        }
    }


def test_align_reconstruction():
    reconstruction = get_reconstruction()
    del reconstruction['shots']['02.jpg']
    del reconstruction['shots']['03.jpg']
    opensfm.reconstruction.align_reconstruction_sensors_similarity(reconstruction)
    print reconstruction


if __name__ == "__main__":
    test_align_reconstruction()

