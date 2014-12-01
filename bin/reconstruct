#!/usr/bin/env python
import os.path, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import argparse
import time
from opensfm import dataset
from opensfm import reconstruction

if __name__ == "__main__":
    start = time.time()
    parser = argparse.ArgumentParser(description='Compute reconstruction')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')
    args = parser.parse_args()

    data = dataset.DataSet(args.dataset)
    reconstruction.incremental_reconstruction(data)
    end = time.time()
    with open(data.profile_log(), 'a') as fout:
        fout.write('reconstruct: {0}\n'.format(end - start))
