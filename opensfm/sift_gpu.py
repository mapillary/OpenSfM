try:
    from silx.image import sift
except ImportError:
    raise ImportError('Cant import silx library for running SIFT_GPU feature extractor or matching,'
                      'please change the config file or install the silx library via pip')


def check_gpu_initialization(config, image, data=None):
    if 'sift_gpu' not in globals():
        global sift_gpu
        if data is not None and isinstance(image, str):
            sift_gpu = SiftGpu.sift_gpu_from_config(config, data.load_image(image))
        else:
            sift_gpu = SiftGpu.sift_gpu_from_config(config, image)


class SiftGpu:
    """
    This Class contain the implementation of sift feature detector and matching on GPU
    The implementation is based on opencl and the python library pyopencl.
    The sift code is based on the library silx
    For more information see this link: http://www.silx.org/doc/silx/dev/index.html.
    for info about the detection or matching please see the links bellow:
    detection - http://www.silx.org/doc/silx/dev/_modules/silx/opencl/sift/plan.html#SiftMatch
    matching - http://www.silx.org/doc/silx/dev/_modules/silx/opencl/sift/match.html

    Note for developers: the initialization of the feature extractor takes approximately 0.2 sec. In order to prevent
    initializing every time the feature extractor, I will use this class as global object.
    """

    def __init__(self, image, sigma=1.6, pix_per_kp=10, device='GPU'):
        """
        initializing the feature detector and matching
        :param image: template image
        :param sigma: parameter for feature detection
        :param pix_per_kp: parameter for feature detection
        :param device: 'CPU' or 'GPU'
        """
        self.sigma = sigma
        self.pix_per_kp = pix_per_kp
        self.device = device
        self.gpu_matching = sift.MatchPlan()
        img_shape = image.shape
        self.width, self.height = img_shape[:2]
        self.init_feature_extractor(image)

    @classmethod
    def sift_gpu_from_config(cls, config, image):
        """
        create class from image and config file
        :param config:
        :param image:
        :return:
        """
        sigma = config["sift_gpu_init_sigma"]
        pix_per_kp = config["pix_per_keypoints"]
        device = config["sift_gpu_device_type"]
        return cls(image, sigma, pix_per_kp, device)

    def detect_image(self, image):
        """
        checks if the image matches the template image.

        :param image: The image we want to detect
        :return: Keypoints of the detection
        """
        img_shape = image.shape
        width, height = img_shape[:2]
        if width != self.width or height != self.height:
            self.init_feature_extractor(image)
        return self.gpu_sift(image)

    def init_feature_extractor(self, image):
        """
        initialize the feature extractor with different template image
        :param image: The new template image
        :return: Nothing
        """
        self.gpu_sift = sift.SiftPlan(template=image,
                                      init_sigma=self.sigma,
                                      PIX_PER_KP=self.pix_per_kp,
                                      devicetype=self.device)

    def match_images(self, kp1, kp2):
        """
        return the matches between to keypoints (of two images)
        :param kp1: keypoints of image 1 (output of detect_image())
        :param kp2: keypoints of image 2 (output of detect_image())
        :return: index array of size(n,2)
        """
        return self.gpu_matching(kp1, kp2, raw_results=True)
