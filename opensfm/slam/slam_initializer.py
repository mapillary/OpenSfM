import numpy as np
import networkx as nx
import slam_debug
from slam_mapper import SlamMapper
from opensfm import pyslam
from opensfm import pymap
from opensfm import features
from opensfm import reconstruction
from opensfm import pysfm
# from opensfm import reconstruction_map
class SlamInitializer(object):

    def __init__(self, data, camera, matcher, reconstruction):
        print("initializer")
        self.init_shot = None
        self.prev_pts = None
        self.data = data
        self.camera = camera
        self.matcher = matcher
        self.system_initialized = False
        self.map = reconstruction

    def initialize(self, curr_shot):
        if self.init_shot is None:
            self.init_shot = curr_shot
            self.prev_pts =\
                pyslam.SlamUtilities.undist_keypts_from_shot(curr_shot)[:, 0:2]
            return False, None
        else:
            return self.initialize_openvslam(curr_shot)

    def initialize_openvslam(self, curr_shot):
        """Initialize similar to ORB-SLAM and Openvslam"""
        # We should have two frames: the current one and the init frame
        # TODO: think about prev_matches!
        matches = self.matcher.match_shot_to_shot(self.init_shot, curr_shot, self.prev_pts, 100)
        matches = np.array(matches)
        if (len(matches) < 100):
            return False, None
        print("Matches: ", len(matches), self.init_shot.id, "<->", curr_shot.id)
        # Update pts
        self.prev_pts[matches[0, :], :] =\
            pyslam.SlamUtilities.undist_keypts_from_shot(curr_shot)[matches[1, :], 0:2]

        f1_points = pyslam.SlamUtilities.keypts_from_shot(self.init_shot)
        f2_points = pyslam.SlamUtilities.keypts_from_shot(curr_shot)
        # test reconstructability
        threshold = 4 * self.data.config['five_point_algo_threshold']
        args = []
        im1 = self.init_shot.id
        im2 = curr_shot.id
        norm_p1 = f1_points[matches[:, 0], 0:2]
        norm_p2 = f2_points[matches[:, 1], 0:2]
        args.append((im1, im2, norm_p1, norm_p2,
                     self.camera, self.camera, threshold))
        chrono = slam_debug.Chronometer()
        chrono.lap("others")
        # np.random.seed(None)
        i1, i2, r = reconstruction._compute_pair_reconstructability(args[0])
        chrono.lap("pair rec")
        if r == 0:
            return False, None
        # create the graph with the new tracks manager
        tracks_graph = pysfm.TracksManager()
        for (track_id, (f1_id, f2_id)) in enumerate(matches):
            tracks_graph.add_observation(
                im1, str(track_id), self.init_shot.get_observation(f1_id))
            tracks_graph.add_observation(
                im2, str(track_id), curr_shot.get_observation(f2_id))

        chrono.lap("track graph")
        rec_report = {}
        rec_init, rec_report['bootstrap'] = \
            reconstruction.bootstrap_reconstruction(self.data, tracks_graph, self.data.load_camera_models(),
                                                    im1, im2, norm_p1, norm_p2)
        success = reconstruction is not None
        chrono.lap("boot rec")
        print("Init timings: ", chrono.lap_times())
        if success:
            print("Created init rec from {}<->{} with {} points from {} matches"
                  .format(im1, im2, len(rec_init.points), len(matches)))
        # rec_init.map.set_landmark_unique_id(len(matches))
        return success, rec_init

    def initialize_openvslam_old(self, curr_shot):
        """Initialize similar to ORB-SLAM and Openvslam"""
        # We should have two frames: the current one and the init frame
        # TODO: think about prev_matches!
        matches = self.matcher.match_shot_to_shot(self.init_shot, curr_shot, self.prev_pts, 100)
        matches = np.array(matches)
        if (len(matches) < 100):
            return None, None, None
        print("Matches: ", len(matches), self.init_shot.id,"<->", curr_shot.id)
        # Update pts
        self.prev_pts[matches[0, :], :] =\
            pyslam.SlamUtilities.undist_keypts_from_shot(curr_shot)[matches[1, :], 0:2]

        f1_points = pyslam.SlamUtilities.keypts_from_shot(self.init_shot)
        f2_points = pyslam.SlamUtilities.keypts_from_shot(curr_shot)
        
        # test reconstructability
        threshold = 4 * self.data.config['five_point_algo_threshold']
        args = []
        im1 = self.init_shot.id
        im2 = curr_shot.id
        norm_p1 = features.\
            normalized_image_coordinates(f1_points[matches[:, 0], 0:2], self.camera[1].width, self.camera[1].height)
        norm_p2 = features.\
            normalized_image_coordinates(f2_points[matches[:, 1], 0:2], self.camera[1].width, self.camera[1].height)
        norm_size = max(self.camera[1].width, self.camera[1].height)
        args.append((im1, im2, norm_p1, norm_p2,
                     self.camera[1], self.camera[1], threshold))
        chrono = slam_debug.Chronometer()
        chrono.lap("others")
        i1, i2, r = reconstruction._compute_pair_reconstructability(args[0])
        chrono.lap("pair rec")
        if r == 0:
            return None, None, None
        scale_1 = f1_points[matches[:, 0], 2] / norm_size
        scale_2 = f2_points[matches[:, 1], 2] / norm_size
        # create the graph
        tracks_graph = nx.Graph()
        tracks_graph.add_node(str(im1), bipartite=0)
        tracks_graph.add_node(str(im2), bipartite=0)
        for (track_id, (f1_id, f2_id)) in enumerate(matches):
            x, y = norm_p1[track_id, 0:2]
            s = scale_1[track_id]
            r, g, b = [255, 0, 0]#self.init_frame.colors[f1_id, :]
            tracks_graph.add_node(str(track_id), bipartite=1)
            tracks_graph.add_edge(str(im1),
                                  str(track_id),
                                  feature=(float(x), float(y)),
                                  feature_scale=float(s),
                                  feature_id=int(f1_id),
                                  feature_color=(float(r), float(g), float(b)))
            x, y = norm_p2[track_id, 0:2]
            s = scale_2[track_id]
            r, g, b = [255, 0, 0] #frame.colors[f2_id, 0:3]
            tracks_graph.add_edge(str(im2),
                                  str(track_id),
                                  feature=(float(x), float(y)),
                                  feature_scale=float(s),
                                  feature_id=int(f2_id),
                                  feature_color=(float(r), float(g), float(b)))
        chrono.lap("track graph")
        rec_report = {}
        reconstruction_init, graph_inliers, rec_report['bootstrap'] = \
            reconstruction.bootstrap_reconstruction(self.data, tracks_graph, self.data.load_camera_models(),
                                                    im1, im2, norm_p1, norm_p2)
        chrono.lap("boot rec")
        print("Init timings: ", chrono.lap_times())
        if reconstruction_init is not None:
            print("Created init rec from {}<->{} with {} points from {} matches"
                  .format(im1, im2, len(reconstruction_init.points), len(matches)))
        # slam_debug.visualize_graph(graph_inliers, self.init_shot.id, curr_shot.id, self.data, do_show=True)
        return reconstruction_init, graph_inliers, matches

