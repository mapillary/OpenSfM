from opensfm import pymap

class Shot(object):
    def __init__(self, shot_id):
        self.shot_id = shot_id




class TestView(object):
    def __init__(self):
        print("__init__")
        self.l = {"shot1": Shot(1), "shot2": Shot(2), "shot4": Shot(4)}
    
    def __len__(self):
        print("__len__")
        return len(self.l)
    
    def __iter__(self):
        print("__iter__")
        for key in self.l.keys():
            yield key

    def values(self):
        for val in self.l.values():
            yield val.shot_id 

    def keys(self):
        for key in self.l.values():
            yield key

test_view =  pymap.TestView()

for shot in test_view:
    print(shot)
for val in test_view.items():
    print(val)