from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

from GUI import Gui
from Database import Database
from mytk import tk
from pathlib import Path
from imageSugestion import get_most_common_seqs, load_from_pair_list, get_all_images
from opensfm import dataset
import sys
import argparse
import pickle


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__)
    parser.add_argument(
        'dataset',
        help='dataset',
    )
    parser.add_argument(
        '-n',
        '--no-preload',
        help='skip preloading',
        action='store_true'
    )
    parser.add_argument('--select_most_common_seqs',
                        help='brings images from most common seqs side by side',
                        action='store_true')
    parser.add_argument('--selected_pairs_path',
                        help='path to pickled list of selected images')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    path = args.dataset
    selected_pairs_path = args.selected_pairs_path
    if args.select_most_common_seqs:
        seqs = get_most_common_seqs(path)
    elif selected_pairs_path:
        seqs = load_from_pair_list(selected_pairs_path)
    else:
        seqs = get_all_images(path)
    database = Database(seqs, path, preload_images=not args.no_preload)
    root = tk.Tk()
    root.resizable(True, True)
    my_gui = Gui(root, database)
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.title("ANNOTATION GUI")
    root.mainloop()
