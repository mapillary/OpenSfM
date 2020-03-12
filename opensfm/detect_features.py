import logging
from timeit import default_timer as timer

import numpy as np

from opensfm import bow
from opensfm import dataset
from opensfm import features
from opensfm import io
from opensfm import log
from opensfm.context import parallel_map, parallel_map_thread

logger = logging.getLogger(__name__)

feature_of_images={}


class detecting_features():
    def __init__(self):
        self.feature_of_images={}
        

    def run(self, data):
        images = data.images()
        arguments = [(image, data) for image in images]
    
        start = timer()
        processes = data.config['processes']
        print(data)

        # parallel_map(self.detect, arguments, processes, 1)
        # data.save_total_fetures(self.feature_of_images)
        
        #parallel_map(detection,arguments,processes,1)
        
        parallel_map_thread(detection, arguments,processes, 1)
        data.save_total_features(feature_of_images)
        
        end = timer()

        with open(data.profile_log(), 'a') as fout:
            fout.write('detect_features: {0}\n'.format(end - start))

        self.write_report(data, end - start)
        logger.info('Feature detected in {}sec'.format(end-start))

        return 


    def write_report(self,data, wall_time):
        image_reports = []
        for image in data.images():
            try:
                txt = data.load_report('features/{}.json'.format(image))
                image_reports.append(io.json_loads(txt))
            except IOError:
                logger.warning('No feature report image {}'.format(image))

        report = {
            "wall_time": wall_time,
            "image_reports": image_reports
        }
        data.save_report(io.json_dumps(report), 'features.json')


    def detect(self,args):
        
        image, data = args
        log.setup()
        print()
        print(data)
        self.feature_pdc={}   
        need_words = data.config['matcher_type'] == 'WORDS' or data.config['matching_bow_neighbors'] > 0
        
        logger.info('Extracting {} features for image {}'.format(
            data.feature_type().upper(), image))
        start = timer()

        #print(image)## 1.jpg
        p_unmasked, f_unmasked, c_unmasked = features.extract_features(
            data.load_image(image), data.config)
        #print(p_unmasked, f_unmasked, c_unmasked)
        fmask = data.load_features_mask(image, p_unmasked)  
        
       
        p_unsorted = p_unmasked[fmask]
        f_unsorted = f_unmasked[fmask]
        c_unsorted = c_unmasked[fmask]

        if len(p_unsorted) == 0:
            logger.warning('No features found in image {}'.format(image))
            return

        size = p_unsorted[:, 2]
        order = np.argsort(size)
        p_sorted = p_unsorted[order, :] # points  == numpy.ndarray
        f_sorted = f_unsorted[order, :] # descriptors
        c_sorted = c_unsorted[order, :] # colors
        #data.save_features(image, p_sorted, f_sorted, c_sorted)

        points=p_sorted.astype(np.float32)
        descriptors=f_sorted.astype(np.uint8)
        colors=c_sorted
        OPENSFM_FEATURES_VERSION=1

        self.feature_pdc.update({'points':points})
        self.feature_pdc.update({'descriptors':descriptors})
        self.feature_pdc.update({'colors':colors})
        self.feature_pdc.update({'OPENSFM_FEATURES_VERSION':OPENSFM_FEATURES_VERSION})
        # self.feature_pdc.update({'points':p_sorted})
        # self.feature_pdc.update({'descriptors':f_sorted})
        # self.feature_pdc.update({'colors':c_sorted})
        self.feature_of_images.update({image:self.feature_pdc})
        
        print("self.feature_of_images==",hex(id(self.feature_of_images)))
        print(image)
        # if need_words:
        #     bows = bow.load_bows(data.config)
        #     n_closest = data.config['bow_words_to_match']
        #     closest_words = bows.map_to_words(
        #         f_sorted, n_closest, data.config['bow_matcher_type'])
        #     data.save_words(image, closest_words)
        end = timer()
        report = {
            "image":image,
            "num_features": len(p_sorted),
            "wall_time": end - start,
        }
        data.save_report_of_features(image,report)
        data.save_report(io.json_dumps(report), 'features/{}.json'.format(image))

def detection(args):
    image, data = args
    log.setup()
    print()
    print(data)
    feature_pdc={}   
    need_words = data.config['matcher_type'] == 'WORDS' or data.config['matching_bow_neighbors'] > 0

    logger.info('Extracting {} features for image {}'.format(
        data.feature_type().upper(), image))
    start = timer()

    #print(image)## 1.jpg
    p_unmasked, f_unmasked, c_unmasked = features.extract_features(
        data.load_image(image), data.config)
    #print(p_unmasked, f_unmasked, c_unmasked)
    fmask = data.load_features_mask(image, p_unmasked)  

    # print(p_unmasked, f_unmasked, c_unmasked)
    # print(fmask)
    # exit()

    p_unsorted = p_unmasked[fmask]
    f_unsorted = f_unmasked[fmask]
    c_unsorted = c_unmasked[fmask]

    if len(p_unsorted) == 0:
        logger.warning('No features found in image {}'.format(image))
        return

    size = p_unsorted[:, 2]
    order = np.argsort(size)
    p_sorted = p_unsorted[order, :] # points  == numpy.ndarray
    f_sorted = f_unsorted[order, :] # descriptors
    c_sorted = c_unsorted[order, :] # colors
    #data.save_features(image, p_sorted, f_sorted, c_sorted)

    points=p_sorted.astype(np.float32)
    descriptors=f_sorted.astype(np.uint8)
    colors=c_sorted
    OPENSFM_FEATURES_VERSION=1

    feature_pdc.update({'points':points})
    feature_pdc.update({'descriptors':descriptors})
    feature_pdc.update({'colors':colors})
    feature_pdc.update({'OPENSFM_FEATURES_VERSION':OPENSFM_FEATURES_VERSION})
    # self.feature_pdc.update({'points':p_sorted})
    # self.feature_pdc.update({'descriptors':f_sorted})
    # self.feature_pdc.update({'colors':c_sorted})
    feature_of_images.update({image:feature_pdc})

    print("self.feature_of_images==",hex(id(feature_of_images)))
    print(image)
    # if need_words:
    #     bows = bow.load_bows(data.config)
    #     n_closest = data.config['bow_words_to_match']
    #     closest_words = bows.map_to_words(
    #         f_sorted, n_closest, data.config['bow_matcher_type'])
    #     data.save_words(image, closest_words)
    end = timer()
    report = {
        "image":image,
        "num_features": len(p_sorted),
        "wall_time": end - start,
    }
    data.save_report_of_features(image,report)
    data.save_report(io.json_dumps(report), 'features/{}.json'.format(image))