import logging
import time

from opensfm import dataset
from opensfm import io
from opensfm import reconstruction

logger = logging.getLogger(__name__)




def run(data):
    start = time.time()

    # cnt=0
    # for node, data in dataset.track_graph_of_images.nodes(data=True):
    #     if data['bipartite'] == 0:
    #         image = node
    #         for track, data in dataset.track_graph_of_images[image].items():
    #             x, y = data['feature']
    #             s = data['feature_scale']
    #             fid = data['feature_id']
    #             r, g, b = data['feature_color']
    #             if cnt==0:
    #                 cnt+=1
    #                 print(track,data)
    #                 print(fid)
    #                 print(x,y)
    #                 print(r,g,b)
    #             exit()

    #graph = data.load_tracks_graph()
    graph=data.track_graph_of_images
    report, reconstructions = reconstruction.\
        incremental_reconstruction(data, graph)
    end = time.time()
    with open(data.profile_log(), 'a') as fout:
        fout.write('reconstruct: {0}\n'.format(end - start))
 
    data.reconstructions=reconstructions
    data.save_reconstruction(reconstructions)
    data.save_report(io.json_dumps(report), 'reconstruction.json')
