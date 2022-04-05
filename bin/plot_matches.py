#!/usr/bin/env python3

import argparse
import os.path
from itertools import combinations

import matplotlib.cm as cm
import matplotlib.pyplot as pl
import numpy as np
from opensfm import dataset
from opensfm import features
from opensfm import io
from numpy import ndarray
from typing import List


def plot_matches(im1, im2, p1: ndarray, p2: ndarray) -> None:
    h1, w1, c = im1.shape
    h2, w2, c = im2.shape
    image = np.zeros((max(h1, h2), w1 + w2, 3), dtype=im1.dtype)
    image[0:h1, 0:w1, :] = im1
    image[0:h2, w1 : (w1 + w2), :] = im2

    p1 = features.denormalized_image_coordinates(p1, w1, h1)
    p2 = features.denormalized_image_coordinates(p2, w2, h2)
    pl.imshow(image)
    for a, b in zip(p1, p2):
        pl.plot([a[0], b[0] + w1], [a[1], b[1]], "c")

    pl.plot(p1[:, 0], p1[:, 1], "ob")
    pl.plot(p2[:, 0] + w1, p2[:, 1], "ob")


def plot_graph(data) -> None:
    cmap = cm.get_cmap("viridis")
    connectivity = {}
    for im1 in images:
        for im2, matches in data.load_matches(im1).items():
            if len(matches) == 0:
                continue
            connectivity[tuple(sorted([im1, im2]))] = len(matches)
    all_values = connectivity.values()
    lowest = np.percentile(list(all_values), 5)
    highest = np.percentile(list(all_values), 95)

    exifs = {im: data.load_exif(im) for im in data.images()}
    reference = data.load_reference()

    for (node1, node2), edge in sorted(connectivity.items(), key=lambda x: x[1]):
        gps1 = exifs[node1]["gps"]
        o1 = np.array(
            reference.to_topocentric(gps1["latitude"], gps1["longitude"], 0)[:2]
        )
        gps2 = exifs[node2]["gps"]
        o2 = np.array(
            reference.to_topocentric(gps2["latitude"], gps2["longitude"], 0)[:2]
        )
        c = max(0, min(1.0, 1 - (edge - lowest) / (highest - lowest)))
        pl.plot([o1[0], o2[0]], [o1[1], o2[1]], linestyle="-", color=cmap(c))

    for node in data.images():
        gps = exifs[node]["gps"]
        o = np.array(reference.to_topocentric(gps["latitude"], gps["longitude"], 0)[:2])
        c = 0
        pl.plot(o[0], o[1], linestyle="", marker="o", color=cmap(c))

    pl.xticks([])
    pl.yticks([])
    ax = pl.gca()
    for b in ["top", "bottom", "left", "right"]:
        ax.spines[b].set_visible(False)
    pl.savefig(os.path.join(data.data_path, "matchgraph.png"))


def plot_matches_for_images(data, image, images) -> None:
    if image:
        pairs = [(image, o) for o in images if o != image]
    elif images:
        subset = images.split(",")
        pairs = combinations(subset, 2)
    else:
        pairs = combinations(images, 2)

    i = 0
    for im1, im2 in pairs:
        matches = data.find_matches(im1, im2)
        if len(matches) == 0:
            continue
        print("plotting {} matches between {} {}".format(len(matches), im1, im2))

        features_data1 = data.load_features(im1)
        features_data2 = data.load_features(im2)
        assert features_data1
        assert features_data2
        p1 = features_data1.points[matches[:, 0]]
        p2 = features_data2.points[matches[:, 1]]

        pl.figure(figsize=(20, 10))
        pl.title("Images: " + im1 + " - " + im2 + ", matches: " + str(matches.shape[0]))
        plot_matches(data.load_image(im1), data.load_image(im2), p1, p2)
        i += 1
        if args.save_figs:
            p = os.path.join(args.dataset, "plot_tracks")
            io.mkdir_p(p)
            pl.savefig(os.path.join(p, "{}_{}.jpg".format(im1, im2)), dpi=100)
            pl.close()
        else:
            if i >= 10:
                i = 0
                pl.show()

    if not args.save_figs and i > 0:
        pl.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot matches between images")
    parser.add_argument("dataset", help="path to the dataset to be processed")
    parser.add_argument("--image", help="show tracks for a specific")
    parser.add_argument(
        "--images", help="show tracks between a subset of images (separated by commas)"
    )
    parser.add_argument("--graph", help="display image graph", action="store_true")
    parser.add_argument(
        "--save_figs", help="save figures instead of showing them", action="store_true"
    )
    args: argparse.Namespace = parser.parse_args()

    data = dataset.DataSet(args.dataset)
    images: List[str] = data.images()

    if args.graph:
        plot_graph(data)
    else:
        plot_matches_for_images(data, args.image, args.images)
