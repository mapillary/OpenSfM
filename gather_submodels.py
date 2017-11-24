import sys

from opensfm import dataset
from opensfm.large import metadataset

r = []
a = []
path = sys.argv[1]
metadata = metadataset.MetaDataSet(path)

for submodel in metadata.get_submodel_paths():
    data = dataset.DataSet(submodel)
    r.extend(data.load_reconstruction('reconstruction.unaligned.json'))
    a.extend(data.load_reconstruction('reconstruction.aligned.json'))

data = dataset.DataSet(path)
data.save_reconstruction(r, 'reconstruction.unaligned.json')
data.save_reconstruction(a, 'reconstruction.aligned.json')
