import logging
import time

from opensfm import dataset
from opensfm import io
from opensfm import reconstruction
from collections import ChainMap

logger = logging.getLogger(__name__)




def run(data):
    start = time.time()
    graph=data.track_graph_of_images
    report, reconstructions = reconstruction.\
        incremental_reconstruction(data, graph)
    end = time.time()
    with open(data.profile_log(), 'a') as fout:
        fout.write('reconstruct: {0}\n'.format(end - start))
    
    print(type(reconstructions))
    

    data.reconstructions=reconstructions
    data.save_reconstruction_to_json(reconstructions[0])

    data.save_reconstruction(reconstructions)
    data.save_report(io.json_dumps(report), 'reconstruction.json')

    
    