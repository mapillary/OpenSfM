from opensfm import dataset


class ImageManager:
    def __init__(self, seqs, path):
        self.seqs = seqs
        self.path = path

    def image_path(self, image_name):
        return f"{self.path}/images/{image_name}"

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
