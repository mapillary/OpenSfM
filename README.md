
labmv
=====
This is a testbed for SFM pipelines.

Dependencies
------------
* libmv
* PCV
* flann (included in third_party)
* vlfleat (included in third_party)

Building
--------
1. build libs in the third_party folder
2. build the code in src
    2.1. mkdir build; cd build
    2.2. cmake ../src
    2.3. make
    2.4. pray

Running
-------
1. put some images in dataset_folder/images
2. run detect_features.py dataset_folder/images
3. run match_features.py dataset_folder/images
4. run robust_match.py dataset_folder/images
5. run create_tracks.py dataset_folder/images
6. run find_homographies.py dataset_folder/images
