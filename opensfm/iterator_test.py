from opensfm import pymap
from opensfm import pygeometry
from opensfm import types
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

# test_view =  pymap.TestView()

# for shot in test_view:
#     print(shot)
# for val in test_view.items():
#     print(val)


t_rec = types.Reconstruction()


rec = pymap.Map()
cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
cam1.id = "cam1"
cam = rec.create_camera(cam1)
shot_view = pymap.ShotView(rec)
shot1 = rec.create_shot("shot1", "cam1")
shot2 = rec.create_shot("shot2", "cam1")
print("shot1: ", shot1)
print("shot2: ", shot2)
# for s2 in shot_view:
#     print("s2", s2)
# for s3 in shot_view.keys():
#     print("s3", s3)
# for s1 in shot_view.values():
#     print("s:", s1, s1.id, shot_view.get(s1.id))
# # for k, v in shot_view.items():
# #     print("s4", k)
# #     print("v", v)
# #     print("vid", v.id)
# for s1 in shot_view.values():
#     print("s:", s1, s1.id, shot_view.get(s1.id))
# for k, v in shot_view.items():
#     print("s4", k)
#     print("v", v)
#     print("vid", v.id)
# for k, v in shot_view.items():
#     print(k, v)
# print("Test")
