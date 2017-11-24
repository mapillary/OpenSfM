import argparse
import datetime
import shutil

from opensfm import dataset
from opensfm import io


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Plot inlier and outlier matches between images')
    parser.add_argument(
        'dataset',
        help='path to the data set to be processed')

    args = parser.parse_args()
    data = dataset.DataSet(args.dataset)
    images_by_time = {i: [] for i in range(24)}
    for image in data.images():
        exif = data.load_exif(image)
        t = datetime.datetime.fromtimestamp(exif['capture_time'])
        images_by_time[t.hour].append(image)

    for i in range(9, 22):
        io.mkdir_p('images_by_time/{}'.format(i))
        for image in images_by_time[i]:
            print 'images/{}'.format(image)
            # shutil.copyfile(
            #     data.image_files[image],
            #     'images_by_time/{}/{}.jpg'.format(i, image))
