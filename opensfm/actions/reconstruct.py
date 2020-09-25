from opensfm import io
from opensfm import reconstruction


def run_dataset(data):
    """ Compute the SfM reconstruction. """

    tracks_manager = data.load_tracks_manager()
    report, reconstructions = reconstruction.\
        incremental_reconstruction(data, tracks_manager)
    data.save_reconstruction(reconstructions)
    data.save_report(io.json_dumps(report), 'reconstruction.json')
