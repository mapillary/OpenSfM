/**
 * @format
 */

import {enuToGeodetic} from '../../node_modules/mapillary-js/dist/mapillary.module.js';
import {imageEntToUniqueID} from '../util/ids.js';
import * as math from './math.js';

export class DataConverter {
  constructor(options) {
    this._options = Object.assign({}, options);
  }

  cluster(cluster, clusterId, reference) {
    const points = cluster.points ?? {};
    const normalize = 1 / 255;
    for (const point of Object.values(points)) {
      const color = point.color;
      color[0] *= normalize;
      color[1] *= normalize;
      color[2] *= normalize;
    }

    return {
      id: clusterId,
      points,
      reference,
    };
  }

  coreImageContract(cellId, items) {
    const contract = {
      cell_id: cellId,
      images: [],
    };
    for (const uniqueId in items) {
      if (!items.hasOwnProperty(uniqueId)) {
        continue;
      }
      contract.images.push(items[uniqueId]);
    }
    return contract;
  }

  entContracts(items) {
    const contracts = [];
    for (const node_id in items) {
      if (!items.hasOwnProperty(node_id)) {
        continue;
      }
      contracts.push({
        node: items[node_id],
        node_id: node_id,
      });
    }
    return contracts;
  }

  mesh(shot) {
    const cameraMatrix = math.createCameraMatrix(
      shot.rotation,
      shot.translation,
    );
    const scaledCameraMatrix = math.mat4Scale(cameraMatrix, shot.scale);
    const vertices = [];

    for (const vertex of shot.vertices) {
      const projected = math.project(vertex, scaledCameraMatrix);
      vertices.push(...projected);
    }

    const faces = this._flatten(shot.faces);

    return {faces: faces, vertices: vertices};
  }

  image(id, clusterId, shot, camera, reference) {
    // Invent unique id
    const uniqueId = imageEntToUniqueID({cluster: {id: clusterId}, id});

    // Core
    const rotation = shot.rotation;
    const translation = shot.translation;
    const opticalCenter = math.createOpticalCenter(rotation, translation);
    const [clng, clat, calt] = this._enuToGeodetic(opticalCenter, reference);
    const computed_altitude = calt;
    const computed_geometry = {
      lat: clat,
      lng: clng,
    };
    const [olng, olat, oalt] = shot.gps_position
      ? this._enuToGeodetic(shot.gps_position, reference)
      : [clng, clat, calt];
    const altitude = oalt;
    const geometry = {
      lat: olat,
      lng: olng,
    };
    const sequence = {id: uniqueId};

    // Camera
    const camera_parameters = [camera.focal, camera.k1, camera.k2];
    const camera_type = this._mapProjectionType(camera.projection_type);

    // URL
    let imageUrl = new URL(this._options.imagePath, this._options.endpoint)
      .href;
    imageUrl += imageUrl.endsWith('/') ? '' : '/';
    const thumb = {id: uniqueId, url: new URL(id, imageUrl).href};
    const mesh = {id: uniqueId, url: uniqueId};

    // Spatial
    const vd = math.createViewingDirection(rotation);
    const rt = math.createCameraMatrix(rotation, translation);
    const upVector = math.createUpVector(shot.orientation, rt);
    const computed_compass_angle = math.createBearing(vd, upVector);
    const compass_angle = computed_compass_angle;
    const atomic_scale = shot.scale;
    const captured_at = shot.capture_time;
    const cluster = {id: clusterId, url: clusterId};
    const computed_rotation = shot.rotation;
    const height = camera.height;
    const merge_id = shot.merge_cc == null ? null : shot.merge_cc.toString();
    const exif_orientation = shot.orientation;
    const quality_score = 1;
    const width = camera.width;

    // Unset
    const organization = {id: null};
    const priv = null;
    const creator = {id: null, username: null};
    const owner = {id: null};

    return {
      altitude,
      atomic_scale,
      computed_rotation,
      compass_angle,
      camera_type,
      captured_at,
      cluster,
      computed_compass_angle,
      computed_altitude,
      camera_parameters,
      camera_type,
      computed_geometry,
      creator,
      geometry,
      height,
      id: uniqueId,
      merge_id,
      mesh,
      organization,
      exif_orientation,
      private: priv,
      quality_score,
      sequence,
      thumb,
      owner,
      width,
    };
  }

  reference(reference) {
    const alt = !!reference ? reference.altitude : 0;
    const lat = !!reference ? reference.latitude : 0;
    const lng = !!reference ? reference.longitude : 0;
    return {alt, lat, lng};
  }

  sequence(imageId) {
    return {id: imageId, image_ids: [imageId]};
  }

  _mapProjectionType(projectionType) {
    return projectionType === 'equirectangular' ? 'spherical' : projectionType;
  }

  _enuToGeodetic(enu, reference) {
    return enuToGeodetic(
      enu[0],
      enu[1],
      enu[2],
      reference.lng,
      reference.lat,
      reference.alt,
    );
  }

  _flatten(a) {
    return a.reduce(
      (accumulator, currentValue) => accumulator.concat(currentValue),
      [],
    );
  }
}
