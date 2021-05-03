/**
 * @format
 */

import {CameraControls} from '../../node_modules/mapillary-js/dist/mapillary.module.js';

export const CameraControlMode = Object.freeze({
  EARTH: 'earth',
  ORBIT: 'orbit',
  STREET: 'street',
});

export function convertCameraControlMode(mode) {
  switch (mode) {
    case CameraControlMode.EARTH:
      return CameraControls.Earth;
    case CameraControlMode.ORBIT:
      return CameraControls.Custom;
    case CameraControlMode.STREET:
      return CameraControls.Street;
    default:
      throw new Error('Camera control mode does not exist');
  }
}
