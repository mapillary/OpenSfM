import logging
import time

from opensfm import dataset
from opensfm import matching

logger = logging.getLogger(__name__)


class Command:
    name = 'create_tracks'
    help = "Link matches pair-wise matches into tracks"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        start = time.time()
        data = dataset.DataSet(args.dataset)
        images = data.images()

        try:
            graph = data.load_tracks_graph()
            tracks, processed_images = matching.tracks_and_images(graph)
        except IOError:
            graph = None
            tracks = None
            processed_images = []

        remaining_images = set(images) - set(processed_images)

        # Read local features
        logging.info('reading features')
        features = {}
        colors = {}
        for im in images:
            p, f, c = data.load_features(im)
            features[im] = p[:, :2]
            colors[im] = c

        # Read matches
        matches = {}
        for im1 in remaining_images:
            try:
                im1_matches = data.load_matches(im1)
            except IOError:
                continue
            for im2 in im1_matches:
                matches[im1, im2] = im1_matches[im2]

        tracks_graph = matching.create_tracks_graph(features, colors, matches,
                                                    data.config, data)
        data.save_tracks_graph(tracks_graph)

        end = time.time()
        with open(data.profile_log(), 'a') as fout:
            fout.write('create_tracks: {0}\n'.format(end - start))
