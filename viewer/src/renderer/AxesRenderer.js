/**
 * @format
 */

import {
  AmbientLight,
  ConeGeometry,
  CylinderGeometry,
  DirectionalLight,
  Euler,
  Matrix4,
  Mesh,
  MeshPhongMaterial,
  Object3D,
  Scene,
  SphereGeometry,
} from '../../node_modules/three/build/three.module.js';

export class AxesRenderer {
  constructor() {
    this._scene = new Scene();
    this._scene.add(this._createAxes());
    this._scene.add(new AmbientLight(0xffffff, 0.5));
    const lightNE = new DirectionalLight(0xffffff, 0.4);
    lightNE.position.set(1, 1, 1);
    const lightSW = new DirectionalLight(0xffffff, 0.1);
    lightSW.position.set(-1, -1, 1);
    this._scene.add(lightNE, lightSW);
  }

  get scene() {
    return this._scene;
  }

  onAdd(viewer, camera, reference) {
    /* noop */
  }

  onReference(reference) {
    /* noop */
  }

  onRemove() {
    /* noop */
  }

  _createAxes() {
    const axisLength = 10;
    const coneLength = axisLength / 5;
    const coneRadius = axisLength / 15;
    const options = {
      coneLength,
      coneRadius,
      cylinderLength: axisLength - coneLength,
      cylinderRadius: 3e-1 * coneRadius,
    };

    const east = this._createAxis(
      Object.assign({}, options, {
        color: 0xff0000,
        euler: [0, 0, -Math.PI / 2],
      }),
    );
    const north = this._createAxis(
      Object.assign({}, options, {
        color: 0x00ff00,
        euler: [0, 0, 0],
      }),
    );
    const up = this._createAxis(
      Object.assign({}, options, {
        color: 0x0088ff,
        euler: [Math.PI / 2, 0, 0],
      }),
    );

    const sphereSegments = 12;
    const origin = new Mesh(
      new SphereGeometry(
        options.cylinderRadius,
        sphereSegments,
        sphereSegments,
      ),
      new MeshPhongMaterial({
        color: 0xffffff,
        flatShading: true,
      }),
    );

    const axes = new Object3D();
    axes.add(east, north, up, origin);
    axes.position.z = -2;

    return axes;
  }

  _createAxis(options) {
    const color = options.color;
    const euler = options.euler;
    const coneLength = options.coneLength;
    const coneRadius = options.coneRadius;
    const cylinderLength = options.cylinderLength;
    const cylinderRadius = options.cylinderRadius;

    const rotation = new Matrix4().multiply(
      new Matrix4().makeRotationFromEuler(new Euler().fromArray(euler)),
    );
    const material = new MeshPhongMaterial({
      color,
      flatShading: true,
    });

    const cylinderSegments = 12;
    const cylinder = new Mesh(
      new CylinderGeometry(
        cylinderRadius,
        cylinderRadius,
        cylinderLength,
        cylinderSegments,
      ),
      material.clone(),
    );
    cylinder.applyMatrix4(
      rotation
        .clone()
        .multiply(new Matrix4().makeTranslation(0, cylinderLength / 2, 0)),
    );

    const coneSegments = 12;
    const cone = new Mesh(
      new ConeGeometry(coneRadius, coneLength, coneSegments),
      material.clone(),
    );
    cone.applyMatrix4(
      rotation
        .clone()
        .multiply(
          new Matrix4().makeTranslation(0, cylinderLength + coneLength / 2, 0),
        ),
    );

    const axis = new Object3D();
    axis.add(cylinder, cone);

    return axis;
  }
}
