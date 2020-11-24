import tkinter as tk
import main
import GUI
from gcp_manager import GroundControlPointManager
from image_manager import ImageManager
from oblique_manager import ObliqueManager

def test_main():
    args=main.parse_args(["/Users/jetolan/src/OpenSfM/data/ground_sfm/test", "--oblique", "/Users/jetolan/src/OpenSfM/data/aerial_sfm/test"])
    path=args.dataset
    ob=args.oblique
    groups, sequence_groups = main.group_images(args)
    image_manager = ImageManager(groups, path, preload_images=not args.no_preload)
    gcp_manager = GroundControlPointManager(path)
    root = tk.Tk()
    root.resizable(True, True)
    ui = GUI.Gui(root, gcp_manager, image_manager, sequence_groups, args.ortho, args.oblique)
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.title("Tools")
    root.mainloop()
    return ui

def test_oblique_manager():
    path = '/Users/jetolan/src/OpenSfM/data/aerial_sfm/test'
    image_manager = ObliqueManager(path)
    image_manager.get_candidates(47.6147, -122.3449)
    assert(len(image_manager.aerial_matches)==3)
    return image_manager

def test_image_manager():
    args = main.parse_args(["/Users/jetolan/src/OpenSfM/data/ground_sfm/test", "--oblique", "/Users/jetolan/src/OpenSfM/data/aerial_sfm/test"])
    path = args.dataset
    ob = args.oblique
    groups, sequence_groups = main.group_images(args)
    image_manager = ImageManager(groups, path, preload_images=not args.no_preload)
    latlon = image_manager.get_nearest_feature('3221-1572996250_front_0000004951.jpg', -.2079, .2462)
    assert(latlon[0]==47.61557121074083)
    return image_manager

if __name__=="__main__":
     # https://stackoverflow.com/questions/45720153/python-multiprocessing-error-attributeerror-module-main-has-no-attribute
    __spec__ = None
    ui=test_main()
    #image_manager=test_oblique_manager()
    #image_manager=test_image_manager()
