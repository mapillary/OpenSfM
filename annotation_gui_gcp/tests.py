import tkinter as tk
import main
import GUI
from gcp_manager import GroundControlPointManager
from image_manager import ImageManager

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
    return True

if __name__=="__main__":
    __spec__ = None
    ui=test_main()
