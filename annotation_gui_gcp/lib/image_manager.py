from flask import send_file
from magic import Magic
from opensfm import dataset


class ImageManager:
    def __init__(self, seqs, path):
        self.seqs = seqs
        self.path = path

    def get_image(self, image_name):
        path_image = f"{self.path}/images/{image_name}"
        magic = Magic(mime=True)
        mimetype = magic.from_file(path_image)
        return send_file(path_image, mimetype=mimetype)

    def load_latlons(self):
        data = dataset.DataSet(self.path)
        latlons = {}
        for keys in self.seqs.values():
            for k in keys:
                if not data.exif_exists(k):
                    continue
                exif = data.load_exif(k)
                if "l" in exif:
                    latlons[k] = exif["l"]
                elif "gps" in exif:
                    latlons[k] = {
                        "lat": exif["gps"]["latitude"],
                        "lon": exif["gps"]["longitude"],
                    }
                elif "cl" in exif:
                    latlons[k] = exif["cl"]
        return latlons
