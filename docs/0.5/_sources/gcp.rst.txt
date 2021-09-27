
Ground Control Points
---------------------

When EXIF data contains GPS location, it is used by OpenSfM to georeference the reconstruction.  Additionally, it is possible to use ground control points.

Ground control points (GCP) are landmarks visible on the images for which the geospatial position (latitude, longitude and altitude) is known.  A single GCP can be observed in one or more images.

OpenSfM uses GCP in two steps of the reconstruction process: alignment and bundle adjustment.  In the alignment step, points are used to globaly move the reconstruction so that the observed GCP align with their GPS position.  Two or more observations for each GCP are required for it to be used during the aligment step.

In the bundle adjustment step, GCP observations are used as a constraint to refine the reconstruction.  In this step, all ground control points are used.  No minimum number of observation is required.

GPSs can be specified in two file formats.  If existing, both are loaded.

.. _json-gcps:

JSON file format
~~~~~~~~~~~~~~~~
GCPs can be specified by adding a text file named ``ground_control_points.json`` at the root folder of the dataset. The format of the file should be as follows::

    {
      "points": [
        {
          "id": STRING,
          "position": {
            "latitude": FLOAT,
            "longitude": FLOAT,
            "altitude": FLOAT  # optional
          },
          "observations": [
            {
              "shot_id": STRING,
              "projection": [FLOAT, FLOAT]  # in normalized image coordinates
            },
            ...
          ]
        },
        ...
      ]
    }

For each point, the latitude and longitude in `WGS84`_ are required.  The altitude, in meters, is optional.  For each image where the point has been observed, the image id and the coordinates of the observation in :ref:`normalized-image-coordinates` are required.


TXT file format
~~~~~~~~~~~~~~~
GCPs can be specified by adding a text file named ``gcp_list.txt`` at the root folder of the dataset. The format of the file should be as follows.

- The first line should contain the name of the projection used for the geo coordinates.

- The following lines should contain the data for each ground control point observation. One per line and in the format::

      <geo_x> <geo_y> <geo_z> <im_x> <im_y> <image_name>

  Where ``<geo_x> <geo_y> <geo_z>`` are the geospatial coordinates of the GCP and ``<im_x> <im_y>`` are the :ref:`pixel-coordinates` where the GCP is observed.  If the altitude of the GCP is not known, set ``<geo_z>`` to ``NaN``.


**Supported projections**

The geospatial coordinates can be specified in one the following formats.

- `WGS84`_: This is the standard latitude, longitude coordinates used by most GPS devices. In this case, ``<geo_x> = longitude``, ``<geo_y> = latitude`` and ``<geo_z> = altitude``

- `UTM`_: UTM projections can be specified using a string projection string such as ``WGS84 UTM 32N``, where 32 is the region and N is . In this case, ``<geo_x> = E``, ``<geo_y> = N`` and ``<geo_z> = altitude``

- `proj4`_: Any valid proj4 format string can be used. For example, for UTM 32N we can use ``+proj=utm +zone=32 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs``

.. _WGS84: https://en.wikipedia.org/wiki/World_Geodetic_System
.. _UTM: https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
.. _proj4: http://proj4.org/

**Example**

This file defines 2 GCP whose coordinates are specified in the WGS84 standard. The first one is observed in both ``01.jpg`` and ``02.jpg``, while the second one is only observed in ``01.jpg`` ::

  WGS84
  13.400740745 52.519134104 12.0792090446 2335.0 1416.7 01.jpg
  13.400740745 52.519134104 12.0792090446 2639.1 938.0 02.jpg
  13.400502446 52.519251158 16.7021233002 766.0 1133.1 01.jpg


