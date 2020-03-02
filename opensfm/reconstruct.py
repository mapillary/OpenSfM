import logging
import time

from opensfm import dataset
from opensfm import io
from opensfm import reconstruction

logger = logging.getLogger(__name__)




def run(data):
    start = time.time()
    graph=data.track_graph_of_images
    report, reconstructions = reconstruction.\
        incremental_reconstruction(data, graph)
    end = time.time()
    with open(data.profile_log(), 'a') as fout:
        fout.write('reconstruct: {0}\n'.format(end - start))
 
    data.reconstructions=reconstructions
    #data.save_reconstruction(reconstructions)
    #data.save_report(io.json_dumps(report), 'reconstruction.json')
