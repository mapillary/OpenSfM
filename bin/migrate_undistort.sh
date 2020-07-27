#!/bin/bash

# Migrate dataset to the new undistort folder structure.

if [ $# -le 0 ] 
then
    echo "Migrate dataset to the new undistort folder structure." 
    echo
	echo "Usage:"
    echo
    echo "    $0 dataset"
    echo
	exit 1
fi 

cd $1

if [ -d "undistorted/images" ]
then
    echo "Some folders already ported. Aborting."
    exit 2
fi

mv undistorted undistorted_images
mkdir undistorted
mv undistorted_images undistorted/images
mv undistorted_masks undistorted/masks
mv undistorted_segmentations undistorted/segmentations
mv undistorted_instances undistorted/instances
mv undistorted_tracks.csv undistorted/tracks.csv
mv undistorted_reconstruction.json undistorted/reconstruction.json
mv depthmaps undistorted/depthmaps
