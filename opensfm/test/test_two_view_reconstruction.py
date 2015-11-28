import numpy as np


def normalized(x):
    return x / np.linalg.norm(x)

def generateRandomPoint( maximumDepth, minimumDepth ):
    cleanPoint = np.random.uniform(-1.0, 1.0, 3)
    direction = normalized(cleanPoint)
    return (maximumDepth - minimumDepth) * cleanPoint + minimumDepth * direction


def generateRandomTranslation( maximumParallax ):
    return np.random.uniform(-maximumParallax, maximumParallax, 3)


def generateRandomRotation( maxAngle ):
    rpy = np.random.uniform(-maxAngle, maxAngle, 3)

    R1 = np.array([[1.0,  0.0,  0.0],
                   [0.0,  np.cos(rpy[0]), -np.sin(rpy[0])],
                   [0.0,  np.sin(rpy[0]),  np.cos(rpy[0])]])

    R2 = np.array([[ np.cos(rpy[1]),  0.0,  np.sin(rpy[1])],
                   [0.0,  1.0,  0.0],
                   [-np.sin(rpy[1]),  0.0,  np.cos(rpy[1])]])

    R3 = np.array([[np.cos(rpy[2]), -np.sin(rpy[2]),  0.0],
                   [np.sin(rpy[2]),  np.cos(rpy[2]),  0.0],
                   [0.0,  0.0,  1.0]])

    return R3.dot(R2.dot(R1))


def addNoise( noiseLevel, cleanPoint ):
    noisyPoint = cleanPoint + np.random.uniform(-noiseLevel, noiseLevel, 3)
    return normalized(noisyPoint)


class TwoViewReconstructionDataset:
    def __init__(self, num_points, noise, outlier_fraction, rotation_only=False):
        self.camera1 = {
            'width': 800,
            'height': 600,
            'projection_type': 'perspective',
            'focal': 0.8,
            'focal_prior': 0.8,
            'k1': 0,
            'k2': 0
        }
        self.camera2 = {
            'width': 800,
            'height': 600,
            'projection_type': 'perspective',
            'focal': 0.8,
            'focal_prior': 0.8,
            'k1': 0,
            'k2': 0
        }

        # generate a random pose for viewpoint 1
        position1 = np.zeros(3)
        rotation1 = np.eye(3)

        # generate a random pose for viewpoint 2
        if rotation_only:
            position2 = np.zeros(3)
        else:
            position2 = generateRandomTranslation(2.0)
        rotation2 = generateRandomRotation(0.5)

        # initialize point-cloud
        min_depth = 4
        max_depth = 8

        self.points = np.empty((num_points, 3))
        for i in range(num_points):
            self.points[i] = generateRandomPoint(max_depth, min_depth)

        # derive correspondences based on random point-cloud
        self.generateCorrespondences(noise, outlier_fraction)

        # Extract the relative pose
        self.position, self.rotation = extractRelativePose(
            position1, position2, rotation1, rotation2)
        if not rotation_only:
            self.essential = essentialMatrix(self.position, self.rotation)


    def generateCorrespondences(self, noise, outlier_fraction):
        num_points = len(self.points)
        self.bearing_vectors1 = np.empty((num_points, 3))
        self.bearing_vectors2 = np.empty((num_points, 3))
        for i in range(num_points):
            # get the point in viewpoint 1
            body_point1 = rotation1.T.dot(self.points[i] - position1)

            # get the point in viewpoint 2
            body_point2 = rotation2.T.dot(self.points[i] - position2)

            self.bearing_vectors1[i] = normalized(body_point1)
            self.bearing_vectors2[i] = normalized(body_point2)

            # add noise
            if noise > 0.0:
                self.bearing_vectors1[i] = addNoise(noise, self.bearing_vectors1[i])
                self.bearing_vectors2[i] = addNoise(noise, self.bearing_vectors2[i])

        # add outliers
        num_outliers = int(outlier_fraction * num_points)
        for i in range(num_outliers):
            # create random point
            p = generateRandomPoint(max_depth, min_depth)

            # project this point into viewpoint 2
            body_point = rotation2.T.dot(p - position2)

            # normalize the bearing vector
            self.bearing_vectors2[i] = normalized(body_point)

