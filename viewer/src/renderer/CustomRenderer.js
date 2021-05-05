/**
 * @format
 */

import {
  Camera,
  WebGLRenderer,
} from '../../node_modules/three/build/three.module.js';
import {EventEmitter} from '../util/EventEmitter.js';

export class CustomRenderer extends EventEmitter {
  constructor(viewer) {
    super();

    this._id = 'opensfm-renderer';
    this._camera = new Camera();
    this._camera.matrixAutoUpdate = false;
    this._viewer = viewer;
    this._reference = null;

    this._children = [];
  }

  get added() {
    return !!this._reference;
  }

  get id() {
    return this._id;
  }

  get reference() {
    return this._reference;
  }

  add(renderer) {
    if (this._children.indexOf(renderer) >= 0) {
      throw new Error(`Renderer already added`);
    }
    this._children.push(renderer);
    if (this.added) {
      renderer.onAdd(this._viewer, this._camera, this._reference);
    }
  }

  remove(renderer) {
    const index = this._children.indexOf(renderer);
    if (index > -1) {
      this._children.splice(index, 1);
      renderer.onRemove();
    }
  }

  onAdd(viewer, reference, context) {
    this._reference = reference;
    const canvas = viewer.getCanvas();
    const renderer = new WebGLRenderer({canvas, context});
    renderer.autoClear = false;
    this._renderer = renderer;
    for (const child of this._children) {
      child.onAdd(this._viewer, this._camera, this._reference);
    }
    this.fire('add', {target: this});
  }

  onReferenceChanged(viewer, reference) {
    this._reference = reference;
    for (const child of this._children) {
      child.onReference(this._reference);
    }
    this.fire('reference', {target: this});
  }

  onRemove(viewer, context) {
    for (const child of this._children) {
      child.onRemove();
    }
  }

  render(context, viewMatrix, projectionMatrix) {
    const camera = this._camera;
    camera.matrix.fromArray(viewMatrix).invert();
    camera.updateMatrixWorld(true);
    camera.projectionMatrix.fromArray(projectionMatrix);
    camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();

    const renderer = this._renderer;
    renderer.resetState();
    for (const child of this._children) {
      renderer.render(child.scene, camera);
    }
  }
}
