from opensfm import dataset


def load_features(data):
    features = {}
    colors = {}
    for im in data.images():
        p, f, c = data.load_features(im)
        features[im] = p[:, :2]
        colors[im] = c
    return features, colors


def load_matches(data):
    matches = {}
    for im1 in data.images():
        try:
            im1_matches = data.load_matches(im1)
        except IOError:
            continue
        for im2 in im1_matches:
            matches[im1, im2] = im1_matches[im2]
    return matches
