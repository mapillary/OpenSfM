#!/usr/bin/env python

import argparse
import logging
import multiprocessing
import os
import subprocess
import sys

from opensfm.large import metadataset

logger = logging.getLogger(__name__)

logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)


class Reconstructor:
    def __init__(self, command, complete):
        self._command = command
        self._complete = complete

    def __call__(self, submodel_path):
        logger.info("===========================================================")
        logger.info("Reconstructing submodel {}".format(submodel_path))
        logger.info("===========================================================")

        if self._complete:
            self._run_command([self._command, 'extract_metadata', submodel_path])
            self._run_command([self._command, 'detect_features', submodel_path])
            self._run_command([self._command, 'match_features', submodel_path])

        self._run_command([self._command, 'create_tracks', submodel_path])
        self._run_command([self._command, 'reconstruct', submodel_path])

        logger.info("===========================================================")
        logger.info("Submodel {} reconstructed".format(submodel_path))
        logger.info("===========================================================")

    def _run_command(self, args):
        result = subprocess.Popen(args).wait()
        if result != 0:
            raise RuntimeError(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Reconstruct all submodels')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')
    parser.add_argument('-c', '--complete',
                        help='Run the complete pipeline on each subset',
                        action='store_true')
    parser.add_argument('-p', '--processes',
                        help='Number of parallel processes to run',
                        type=int, default=1)
    args = parser.parse_args()

    meta_data = metadataset.MetaDataSet(args.dataset)
    exec_dir = os.path.join(os.getcwd(), os.path.dirname(sys.argv[0]))
    command = os.path.join(exec_dir, "bin/opensfm")

    submodel_paths = meta_data.get_submodel_paths()
    reconstructor = Reconstructor(command, args.complete)

    if args.processes == 1:
        for submodel_path in submodel_paths:
            reconstructor(submodel_path)
    else:
        p = multiprocessing.Pool(args.processes)
        p.map(reconstructor, submodel_paths)
