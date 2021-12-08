import typing as t
from io import BytesIO

from flask import send_file
from magic import Magic
from opensfm import dataset
from PIL import Image


class ImageManager:
    def __init__(self, seqs: t.Dict[str, t.List[str]], path: str):
        self.seqs = seqs
        self.path = path

    def get_image(self, image_name: str, max_sz: int = 0):
        path_image = f"{self.path}/images/{image_name}"

        # Downsample before sending if max_sz
        if max_sz > 0:
            im = Image.open(path_image)
            im.thumbnail((max_sz, max_sz))

            im_bytes = BytesIO()
            im.save(im_bytes, format="JPEG")
            im_bytes.seek(0)

            return send_file(im_bytes, mimetype="image/jpeg")
        # Just send the file
        else:
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
