import logging
from timeit import default_timer as timer

import numpy as np

from opensfm import dataset
from opensfm import io
from opensfm import log
from opensfm import matching
from opensfm import pairs_selection
from opensfm.context import parallel_map


logger = logging.getLogger(__name__)



def run(data):
    images=[]

    for i in data.images():
        images.append(i)

    start = timer()
    pairs_matches, preport = matching.match_images(data, images, images)
    save_matches(data, images, pairs_matches)
    end = timer()

    with open(data.profile_log(), 'a') as fout:
        fout.write('match_features: {0}\n'.format(end - start))
    write_report(data, preport, list(pairs_matches.keys()), end - start)
    

def save_matches(data, images_ref, matched_pairs):
    """ Given pairwise matches (image 1, image 2) - > matches,
    save them such as only {image E images_ref} will store the matches.
    """

    matches_per_im1 = {im: {} for im in images_ref}
    for (im1, im2), m in matched_pairs.items():
        matches_per_im1[im1][im2] = m

    for im1, im1_matches in matches_per_im1.items():
        #data.save_matches(im1, im1_matches)
        data.save_total_matches(im1,im1_matches) 

def write_report(data, preport, pairs, wall_time):
    report = {
        "wall_time": wall_time,
        "num_pairs": len(pairs),
        "pairs": pairs,
    }
    report.update(preport)
    data.save_report(io.json_dumps(report), 'matches.json')
