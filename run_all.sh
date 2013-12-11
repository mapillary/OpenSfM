./focal_from_exif.py $1
./detect_features.py $1
./match_features.py -r $1
./create_tracks.py $1
./two_view_reconstruction.py $1
