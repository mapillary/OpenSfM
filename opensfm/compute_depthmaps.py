import logging

from opensfm import dataset
from opensfm import dense
from opensfm import undistorted_dataset

logger = logging.getLogger(__name__)

def run(data):
    #udata = dataset.UndistortedDataSet(data, args.subfolder)
    udata=data.undistorted_data
    data.config['interactive'] = False# FAlse
    reconstructions = data.udata_reconstruction
    graph = data.udata_track_graph
    dense.compute_depthmaps(data,udata, graph, reconstructions[0])
