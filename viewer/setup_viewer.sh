#!/bin/bash

_dat_gui_version="0.7.7"
_gl_matrix_version="3.3.0"
_mapillary_js_version="3.1.0"

_dat_gui="dat.gui.min.js"
_gl_matrix="gl-matrix-min.js"
_mapillary="mapillary.min.js"
_mapillary_css="mapillary.min.css"

_unpkg="https://unpkg.com"

_dest="$(dirname "${BASH_SOURCE[0]}")/mjs/dependencies"
mkdir -p "$_dest"

curl -sSL "$_unpkg/dat.gui@$_dat_gui_version/build/$_dat_gui" \
  > "$_dest/$_dat_gui"
curl -sSL "$_unpkg/gl-matrix@$_gl_matrix_version/$_gl_matrix" \
  > "$_dest/$_gl_matrix"
curl -sSL "$_unpkg/mapillary-js@$_mapillary_js_version/dist/$_mapillary" \
  > "$_dest/$_mapillary"
curl -sSL "$_unpkg/mapillary-js@$_mapillary_js_version/dist/$_mapillary_css" \
  > "$_dest/$_mapillary_css"
