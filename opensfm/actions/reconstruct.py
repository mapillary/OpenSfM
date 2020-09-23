import time

from opensfm import io
from opensfm import reconstruction


def run_dataset(data):
    """ Compute the SfM reconstruction. """

    start = time.time()
    tracks_manager = data.load_tracks_manager()
    report, reconstructions = reconstruction.\
        incremental_reconstruction(data, tracks_manager)
    end = time.time()
    with open(data.profile_log(), 'a') as fout:
        fout.write('reconstruct: {0}\n'.format(end - start))
    data.save_reconstruction(reconstructions)
    data.save_report(io.json_dumps(report), 'reconstruction.json')
