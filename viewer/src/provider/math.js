/**
 * @format
 */

import * as vec3 from '../../node_modules/gl-matrix/esm/vec3.js';
import * as mat4 from '../../node_modules/gl-matrix/esm/mat4.js';

export function angleAxisToAngle(a) {
  const [x, y, z] = a;
  return Math.sqrt(x * x + y * y + z * z);
}

export function createBearing(direction, up) {
  const upProjection = vec3.dot(direction, up);
  const scaledUp = vec3.clone(up);
  vec3.scale(scaledUp, scaledUp, upProjection);
  const planeProjection = vec3.create();
  vec3.sub(planeProjection, direction, scaledUp);
  const phi = Math.atan2(planeProjection[1], planeProjection[0]);
  const bearing = -phi + Math.PI / 2;
  return wrap(radToDeg(bearing), 0, 360);
}

export function createCameraMatrix(rotation, translation) {
  const [rx, ry, rz] = rotation;
  const [tx, ty, tz] = translation;
  const angle = angleAxisToAngle(rotation);
  const axis = vec3.fromValues(rx, ry, rz);
  vec3.scale(axis, axis, 1 / angle);

  const rt = mat4FromAngleAxis(angle, axis);
  rt[12] = tx;
  rt[13] = ty;
  rt[14] = tz;
  return rt;
}

export function createOpticalCenter(rotation, translation) {
  const [rx, ry, rz] = rotation;
  const [tx, ty, tz] = translation;

  const angle = angleAxisToAngle(rotation);
  const axis = vec3.fromValues(-rx, -ry, -rz);
  vec3.scale(axis, axis, 1 / angle);

  const t = vec3.fromValues(-tx, -ty, -tz);
  return vec3Rotate(t, angle, axis);
}

export function createUpVector(orientation, rt) {
  switch (orientation) {
    case 3:
      return [rt[1], rt[5], rt[9]];
    case 6:
      return [-rt[0], -rt[4], -rt[8]];
    case 8:
      return [rt[0], rt[4], rt[8]];
    case 1:
    default:
      return [-rt[1], -rt[5], -rt[9]];
  }
}

export function createViewingDirection(rotation) {
  const [rx, ry, rz] = rotation;
  const angle = angleAxisToAngle(rotation);
  const axis = vec3.fromValues(-rx, -ry, -rz);
  vec3.scale(axis, axis, 1 / angle);

  const vd = vec3.fromValues(0, 0, 1);
  return vec3Rotate(vd, angle, axis);
}

export function mat4FromAngleAxis(angle, axis) {
  const m = mat4.create();
  return mat4.fromRotation(m, angle, axis);
}

export function mat4Scale(m, s) {
  const scaled = mat4.clone(m);
  mat4.scale(scaled, scaled, [s, s, s]);
  scaled[12] = s * scaled[12];
  scaled[13] = s * scaled[13];
  scaled[14] = s * scaled[14];
  return scaled;
}

export function vec3Rotate(v, angle, axis) {
  const m = mat4FromAngleAxis(angle, axis);
  const r = vec3.clone(v);
  vec3.transformMat4(r, r, m);
  return r;
}

export function project(vertex, projectionMatrix) {
  const [x, y, z] = vertex;
  const v = vec3.fromValues(x, y, z);
  const projected = vec3.create();
  vec3.transformMat4(projected, v, projectionMatrix);
  return projected;
}

export function radToDeg(rad) {
  return (180 * rad) / Math.PI;
}

export function wrap(value, min, max) {
  if (max < min) {
    throw new Error('Invalid arguments: max must be larger than min.');
  }

  const interval = max - min;
  while (value > max || value < min) {
    if (value > max) {
      value = value - interval;
    } else if (value < min) {
      value = value + interval;
    }
  }
  return value;
}
