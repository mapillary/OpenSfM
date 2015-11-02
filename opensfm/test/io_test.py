from opensfm import io

filename = 'reconstruction_berlin.json'

def import_opensfm_test():

    reconstructions = io.import_opensfm(filename)

    assert len(reconstructions) == 1
    assert len(reconstructions[0].cameras) == 1
    assert len(reconstructions[0].shots) == 3
    assert len(reconstructions[0].points) == 1588

if __name__ == "__main__":
    import_opensfm_test()
