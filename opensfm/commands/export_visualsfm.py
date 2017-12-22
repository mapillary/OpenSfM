import logging
import os

from opensfm import dataset
from opensfm import transformations as tf

logger = logging.getLogger(__name__)


class Command:
    name = 'export_visualsfm'
    help = "Export reconstruction to NVM_V3 format from VisualSfM"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        reconstructions = data.load_reconstruction()
        graph = data.load_tracks_graph()

        if reconstructions:
            self.export(reconstructions[0], graph, data)

    def export(self, reconstruction, graph, data):
        lines = ['NVM_V3', '', str(len(reconstruction.shots))]
        for shot in list(reconstruction.shots.values()):
            q = tf.quaternion_from_matrix(shot.pose.get_rotation_matrix())
            o = shot.pose.get_origin()
            words = [
                self.image_path(shot.id, data),
                shot.camera.focal * max(shot.camera.width, shot.camera.height),
                q[0], q[1], q[2], q[3],
                o[0], o[1], o[2],
                '0', '0',
            ]
            lines.append(' '.join(map(str, words)))
        lines += ['0', '', '0', '', '0']

        with open(data.data_path + '/reconstruction.nvm', 'w') as fout:
            fout.write('\n'.join(lines))

    def image_path(self, image, data):
        """Path to the undistorted image relative to the dataset path."""
        path = data._undistorted_image_file(image)
        return os.path.relpath(path, data.data_path)
