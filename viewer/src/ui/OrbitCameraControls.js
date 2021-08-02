/**
 * @format
 */

import {
  Matrix4,
  PerspectiveCamera,
  Vector3,
} from '../../node_modules/three/build/three.module.js';
import * as common from '../../node_modules/gl-matrix/esm/common.js';
import {OrbitControls} from '../../node_modules/three/examples/jsm/controls/OrbitControls.js';

const DAMPING_FACTOR = 0.1;
const FOV = 90;
const MAX_DISTANCE = 5000;
const MIN_DISTANCE = 1;

export class OrbitCameraControls {
  constructor() {
    this._controls = null;
    this._reference = null;
    this._projectionMatrixCallback = null;
    this._viewMatrixCallback = null;
  }

  onActivate(viewer, viewMatrix, projectionMatrix, reference) {
    this._reference = reference;

    const container = viewer.getContainer();
    const aspect = this._calcAspect(container);
    const camera = new PerspectiveCamera(FOV, aspect, 0.1, 10000);
    camera.up.set(0, 0, 1);

    const controls = new OrbitControls(camera, container);
    this._controls = controls;
    controls.minDistance = MIN_DISTANCE;
    controls.maxDistance = MAX_DISTANCE;
    controls.enableDamping = true;
    controls.dampingFactor = DAMPING_FACTOR;

    const viewMatrixInverse = new Matrix4().fromArray(viewMatrix).invert();
    const me = viewMatrixInverse.elements;
    const invent = this._shouldInventPose(me);

    if (invent) {
      controls.object.position.set(-50, -50, 40);
      controls.target.set(0, 0, 0);
    } else {
      controls.object.position.set(me[12], me[13], me[14]);
      controls.target.copy(this._makeTarget(viewMatrixInverse));
      controls.object.matrixWorld.copy(viewMatrixInverse);
      controls.object.matrixWorldInverse.fromArray(viewMatrix);
    }
    controls.object.updateMatrixWorld(true);

    controls.addEventListener('change', this._onControlsChange);
    controls.update();

    this._updateProjectionMatrix();
  }

  onAnimationFrame(viewer, frameId) {
    // Workaround until MJS is fixed to not invoke before onActivate
    if (this._controls) {
      this._controls.update();
    }
  }

  onAttach(viewer, viewMatrixCallback, projectionMatrixCallback) {
    this._viewMatrixCallback = viewMatrixCallback;
    this._projectionMatrixCallback = projectionMatrixCallback;
  }

  onDeactivate(viewer) {
    this._controls.removeEventListener('change', this._onControlsChange);
    this._controls.dispose();
    this._controls = null;
  }

  onDetach(viewer) {
    this._projectionMatrixCallback = null;
    this._viewMatrixCallback = null;
  }

  onReference(viewer, reference) {
    const oldReference = this._reference;
    const camera = this._controls.object;
    const target = this._controls.target;
    const targetOffset = target.clone().sub(camera.position);

    const enu = camera.position;
    const [lng, lat, alt] = enuToGeodetic(
      enu.x,
      enu.y,
      enu.z,
      oldReference.lng,
      oldReference.lat,
      oldReference.alt,
    );
    const [e, n, u] = geodeticToEnu(
      lng,
      lat,
      alt,
      reference.lng,
      reference.lat,
      reference.alt,
    );

    const position = new Vector3(e, n, u);
    this._controls.object.position.copy(position);
    this._controls.object.updateMatrixWorld(true);
    this._controls.target.copy(position.clone().add(targetOffset));

    this._reference = reference;
  }

  onResize(viewer) {
    this._updateProjectionMatrix();
  }

  _calcAspect(element) {
    const width = element.offsetWidth;
    const height = element.offsetHeight;
    return width === 0 || height === 0 ? 0 : width / height;
  }

  _onControlsChange = () => {
    this._controls.object.updateMatrixWorld(true);
    this._viewMatrixCallback(
      this._controls.object.matrixWorldInverse.toArray(),
    );
  };

  _updateProjectionMatrix() {
    const camera = this._controls.object;
    camera.aspect = this._calcAspect(this._controls.domElement);
    camera.updateProjectionMatrix();
    this._projectionMatrixCallback(camera.projectionMatrix.toArray());
  }

  _shouldInventPose(viewMatrixInverse) {
    const me = viewMatrixInverse;
    const eye = [me[12], me[13], me[14]];
    return (
      common.equals(eye[0], 0) &&
      common.equals(eye[0], 0) &&
      common.equals(eye[0], 0)
    );
  }

  _makeTarget(viewMatrixInverse) {
    const me = viewMatrixInverse.elements;
    const eye = new Vector3(me[12], me[13], me[14]);
    const forward = new Vector3(-me[8], -me[9], -me[10]);
    const xy = Math.sqrt(forward.x * forward.x + forward.y * forward.y);
    const angle = Math.atan2(forward.z, xy);

    if (angle > -Math.PI / 45) {
      // Target a point 10 meters distance in front of eye
      return eye.clone().add(forward.normalize().multiplyScalar(10));
    } else {
      // Target a point on invented ground
      const l0 = eye.clone();
      const n = new Vector3(0, 0, 1);
      const p0 = new Vector3(0, 0, -2);
      const d = new Vector3().subVectors(p0, l0).dot(n) / forward.dot(n);
      const intersection = l0
        .clone()
        .add(forward.clone().multiplyScalar(Math.min(MAX_DISTANCE, d)));
      return intersection;
    }
  }
}
