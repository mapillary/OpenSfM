/**
 * @format
 */

import {
  PlaneGeometry,
  Vector3,
  Mesh,
  TextureLoader,
  MeshBasicMaterial,
  Object3D,
  BufferGeometry,
  Float32BufferAttribute,
  Scene,
} from '../../node_modules/three/build/three.module.js';

import {
  geodeticToEnu,
  enuToGeodetic,
} from '../../node_modules/mapillary-js/dist/mapillary.module.js';

import {
  formatString,
} from '../util/format.js';

function lng2tilex(lon, zoom) {
  return Math.floor(((lon + 180) / 360) * Math.pow(2, zoom));
}
function lat2tiley(lat, zoom) {
  return Math.floor(
    ((1 -
      Math.log(
        Math.tan((lat * Math.PI) / 180) + 1 / Math.cos((lat * Math.PI) / 180),
      ) /
        Math.PI) /
      2) *
      Math.pow(2, zoom),
  );
}
function tilex2lng(x, z) {
  return (x / Math.pow(2, z)) * 360 - 180;
}
function tiley2lat(y, z) {
  const n = Math.PI - (2 * Math.PI * y) / Math.pow(2, z);
  return (180 / Math.PI) * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
}

export class BasemapRenderer {
  constructor(options) {
    this._scene = new Scene();
    this._tiles = null;
    this._viewer = null;

    this._opacity = options.basemapOpacity;
    this._altitude = options.basemapAltitude;
    this._zoomLevel = options.basemapZoomLevel;
    this._tileCount = options.basemapTileCount;
    this._tileServerUrl = options.tileServerUrl;

    // We store the reference to be able to update tiles when any of the
    // following change:
    //  - the number of tiles
    //  - the tile server url
    //  - the zoom level
    this._reference = null;
  }

  get scene() {
    return this._scene;
  }

  setOpacity(opacity) {
    this._opacity = opacity;
    if (this._tiles) {
      this._tiles.children.forEach(child => {
        if (child instanceof Mesh) {
          child.material.opacity = opacity;
        }
      });
    }
  }

  setAltitude(altitude) {
    this._altitude = altitude;
    if (this._reference && this._tiles) {
      // Regenerate the order, use to reconstruct positions with new altitude
      const spiralOrder = this._spiralOrder(this._tileCount);
      const { x, y } = this._computeTileCoords(this._reference, this._zoomLevel);
      this._tiles.children.forEach((tile, index) => {
        const corners = this._tileEnuCorners(x + spiralOrder[index][0],
          y + spiralOrder[index][1], this._zoomLevel, this._reference);
        const {geometry, center} = this._makeTileGeometryFromCorners(corners);
        let oldGeometry = tile.geometry;
        tile.geometry = geometry;
        tile.position.set(center.x, center.y, center.z);
        oldGeometry.dispose();
      });
    }
  }

  setTileCount(tileCount) {
    const newTileCount = Math.round(tileCount);
    if (this._tileCount === newTileCount)
      return;

    // New tile count is less than the current, just pop and dispose some of the
    // tiles at the end
    if (newTileCount < this._tileCount) {
      const tilesToPop = this._tileCount - newTileCount;
      for (let i = 0; i < tilesToPop; ++i) {
        if (this._tiles) {
          this._disposeTile(this._tiles.children.pop());
        }
      }

      this._tileCount = newTileCount;
    }
    else {
      this._tileCount = newTileCount;

      if (this._reference) {
        if (!this._tiles) {
          this._tiles = new Object3D();
        }

        this._createTiles(this._reference);
      }
    }
  }

  setZoomLevel(zoomLevel) {
    const newZoomLevel = Math.round(zoomLevel);
    if (this._zoomLevel === newZoomLevel)
      return;

    this._zoomLevel = Math.round(zoomLevel);
    if (this._reference) {
      this.onReference(this._reference);
    }
  }

  setTileServerUrl(tileServerUrl) {
    if (this._tileServerUrl === tileServerUrl)
      return;

    this._tileServerUrl = tileServerUrl;
    if (this._reference) {
      this.onReference(this._reference);
    }
  }

  onAdd(viewer, camera, reference) {
    this._viewer = viewer;
    this.onReference(reference);
  }

  onReference(reference) {
    this._reference = reference;
    this._clearTiles();
    this._tiles = new Object3D();
    this._createTiles(this._reference);
    this._scene.add(this._tiles);
  }

  onRemove() {
    this._clearTiles();
  }

  _disposeTile(tile) {
    if (tile instanceof Mesh) {
      if (tile.material instanceof MeshBasicMaterial) {
        if (tile.material.map) {
          tile.material.map.dispose();
        }
      }
      tile.material.dispose();
      tile.geometry.dispose();
    }
  }

  _clearTiles() {
    if (this._tiles) {
      this._scene.remove(this._tiles);
      this._tiles.children.forEach(child => {
        this._disposeTile(child);
      });
      this._tiles = null;
    }
  }

  _createTiles(reference) {
    const zoom = this._zoomLevel;

    const { x, y } = this._computeTileCoords(reference, zoom);

    // Remove the first N values from the spiral order, as we already have
    // these tiles in our _tiles container object
    const spiralOrder = this._spiralOrder(this._tileCount);
    spiralOrder.splice(0, this._tiles.children.length);
    spiralOrder.forEach(offset => {
      this._tiles.add(
        this._createTile(x + offset[0], y + offset[1], zoom, reference));
    });
  }

  _makeTileGeometryFromCorners(corners) {
    const width = Math.abs(corners[0].x - corners[1].x);
    const height = Math.abs(corners[0].y - corners[1].y);
    const geometry = new PlaneGeometry(width, height);

    const center = new Vector3();
    center.addVectors(corners[0], corners[1]).divideScalar(2);
    return {geometry, center};
  }

  _tileEnuCorners(x, y, z, reference) {
    // Get corners of tile in geodetic coordinates
    const lng0 = tilex2lng(x, z);
    const lat0 = tiley2lat(y, z);
    const lng1 = tilex2lng(x + 1, z);
    const lat1 = tiley2lat(y + 1, z);

    // Get corners of tile in local (enu) reference frame
    const plane_altitude = reference.alt + this._altitude;
    const point0 = new Vector3(
      ...geodeticToEnu(
        lng0,
        lat0,
        plane_altitude,
        reference.lng,
        reference.lat,
        reference.alt,
      ),
    );
    const point1 = new Vector3(
      ...geodeticToEnu(
        lng1,
        lat1,
        plane_altitude,
        reference.lng,
        reference.lat,
        reference.alt,
      ),
    );

    return [point0, point1];
  }

  _createTile(x, y, z, reference) {
    const url = formatString(this._tileServerUrl, {x, y, z});
    const corners = this._tileEnuCorners(x, y, z, reference);
    const {geometry, center} = this._makeTileGeometryFromCorners(corners);
    const viewer = this._viewer;
    const texture = new TextureLoader().load(
      url,
      texture => {
        viewer.triggerRerender();
      },
    );

    const material = new MeshBasicMaterial({map: texture});
    material.transparent = true;
    material.opacity = this._opacity;
    const plane = new Mesh(geometry, material);

    plane.position.set(center.x, center.y, center.z);
    return plane;
  }

  _computeTileCoords(reference, zoom) {
    const lla = enuToGeodetic(
      0,
      0,
      0,
      reference.lng,
      reference.lat,
      reference.alt,
    );

    const x = lng2tilex(lla[0], zoom);
    const y = lat2tiley(lla[1], zoom);
    return { x, y };
  }

  /**
   * Creates the spiral order as shown in the ascii art below
   * The letter 'C' represents the center at (0, 0) and
   * the numbers represent the lengths of segments that we are processing in
   * order (first segments of length 1, then 2 and so on)
   *  +---+-----------+
      |   |     3     |
      |   +-----------+
      | 3 | 1 | 1 |
      |   |---+---+
      |   | C |   |
      +---+---+ 2 |
      |   2   |   |
      +-------+---+
   */
  _spiralOrder(desiredLength) {
    if (desiredLength === 0) {
      return [];
    }

    const moves = [[0, 1], [1, 0], [0, -1], [-1, 0]];

    // Tracks the current direction of the spiral: we start with "Up"
    // Check the 'moves' variable to see the order of the 4 directions
    let dir = 0;

    // Tracks the length of the current segment: this is the number
    // shown in the ASCII diagram above
    let n = 1;

    // Tracks the index of the element in the current segment [0, n-1]
    let k = 0;

    // Tracks which same-length segment do we currently process, either 0 or 1
    let p = 0;

    let spiral = [[0, 0]];

    let x = 0;
    let y = 0;
    while (spiral.length < desiredLength) {
      x = x + moves[dir][0];
      y = y + moves[dir][1];
      spiral.push([x, y]);

      k++;
      if (k === n) {
        dir = (dir + 1) % 4;

        k = 0;
        p = (p + 1) % 2; // switch into the next segment of the same length
        n = n + 1 - p; // is the new segment 0 again (p === 0)? Increase length
      }
    }

    return spiral;
  }
}
