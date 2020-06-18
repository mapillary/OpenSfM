import os.path, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import slam_debug
from slam_system import SlamSystem
import argparse
import logging
logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)
from opensfm import dataset
from opensfm import io

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


# For visualization
RUN_VISUALIZATION = True
if RUN_VISUALIZATION:
    from opensfm import visualization
    import numpy as np
    import threading


def run_slam(slam_vis = None):
    # Create the top-level parser
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', help='dataset to process')
    args = parser.parse_args()
    slam_system = SlamSystem(args)
    data = dataset.DataSet(args.dataset)
    start_id = 0
    n_kfs = 0
    for idx, im_name in enumerate(sorted(data.image_list)):
        if idx < start_id:
            continue
        gray_scale_img = io.imread(data._image_file(im_name), grayscale=True)  # The gray-scale image
        ret = slam_system.process_frame(im_name, gray_scale_img)

        if slam_vis is not None and RUN_VISUALIZATION:
            slam_vis.update_image(gray_scale_img)
            # Update map only after KF insertion
            if n_kfs != len(slam_system.slam_mapper.keyframes):
                slam_system.reconstruction.map.color_map()
                slam_vis.update_reconstruction(slam_system.reconstruction, slam_system.slam_mapper.keyframes)
                n_kfs = len(slam_system.slam_mapper.keyframes)

        slam_debug.avg_timings.printAvgTimings()
        if ret:
            logger.info("Successfully tracked {}".format(im_name))
        else:
            logger.info("Trying to init with {}".format(im_name))

    slam_system.slam_mapper.save_reconstruction(im_name + "_finished.json")
if RUN_VISUALIZATION:
    vis = visualization.Visualization(np.array([370, 1226]))
    th = threading.Thread(target=run_slam, args=(vis,))
    th.start()
    vis.run_visualization()
else:
    run_slam()