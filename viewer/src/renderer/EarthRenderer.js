/**
 * @format
 */

import {
  BufferGeometry,
  DoubleSide,
  Float32BufferAttribute,
  Mesh,
  Raycaster,
  Scene,
  ShaderMaterial,
  Vector2,
  Vector3,
} from '../../node_modules/three/build/three.module.js';
import {pixelToViewport} from '../util/coords.js';

const frag = `
#ifdef GL_ES
precision mediump float;
#endif

uniform float uLineWidth;
uniform float uTileSize;
uniform vec2 uMouse;
uniform bool uMouseIntersecting;
uniform float uRadius;

varying vec4 vWorldCoords;

void main()	{
    vec4 fragColor = vec4(0.0, 0.0, 0.0, 0.0);

    float mouseDist = distance(vWorldCoords.xy, uMouse);
    bool showIndicator =
        uMouseIntersecting &&
        mouseDist < uRadius;

    if (showIndicator) {
        float discAlpha = (uRadius - mouseDist) / uRadius;
        discAlpha = smoothstep(0.0, 1.0, discAlpha);

        vec2 pos = vWorldCoords.xy / uTileSize;
        vec2 f = abs(pos - floor(pos + vec2(0.5, 0.5)));
        vec2 df = fwidth(pos) * uLineWidth;
        vec2 g = smoothstep(-df, df, f);
        float grid = 1.0 - clamp(g.x * g.y, 0.0, 1.0);
        fragColor.rgb = mix(fragColor.rgb, vec3(1.0, 1.0, 1.0), grid);
        fragColor.a = discAlpha;
    }

    gl_FragColor = fragColor;
}
`;

const vert = `
#ifdef GL_ES
precision mediump float;
#endif

varying vec4 vWorldCoords;

void main() {
    vWorldCoords = vec4(position, 1.0);
    gl_Position = projectionMatrix * viewMatrix * vec4(position, 1.0);
}
`;

const SIZE = 10000;

export class EarthRenderer {
  constructor() {
    this._earth = this._makeEarth();
    this._scene = new Scene();
    this._scene.add(this._earth);

    this._camera = null;
    this._raycaster = new Raycaster();
    this._origin = new Vector3();
    this._direction = new Vector3();
  }

  get active() {
    return !!this._camera;
  }

  get scene() {
    return this._scene;
  }

  intersect(event) {
    if (!this.active) {
      throw new Error('Cannot intersect when inactive');
    }

    const camera = this._camera;
    const raycaster = this._raycaster;
    const origin = this._origin;
    const direction = this._direction;
    const uniforms = this._earth.material.uniforms;

    const viewport = pixelToViewport(event.pixelPoint, this._container);
    origin.setFromMatrixPosition(camera.matrixWorld);
    direction
      .set(viewport[0], viewport[1], 0.5)
      .applyMatrix4(camera.projectionMatrixInverse)
      .applyMatrix4(camera.matrixWorld)
      .sub(origin)
      .normalize();
    raycaster.set(origin, direction);

    const intersections = raycaster.intersectObject(this._earth);
    if (intersections.length) {
      const intersection = intersections[0];
      uniforms.uMouseIntersecting.value = true;
      uniforms.uMouse.value.x = intersection.point.x;
      uniforms.uMouse.value.y = intersection.point.y;
    } else {
      uniforms.uMouseIntersecting.value = false;
      uniforms.uMouse.value.x = 0;
      uniforms.uMouse.value.y = 0;
    }
  }

  onAdd(viewer, camera, reference) {
    this._container = viewer.getContainer();
    this._camera = camera;
  }

  onReference(reference) {
    /* noop */
  }

  onRemove() {
    this._camera = null;
  }

  _makeEarth() {
    const size = SIZE;
    const quad = [
      size / 2,
      size / 2,
      -2,
      size / 2,
      -size / 2,
      -2,
      -size / 2,
      -size / 2,
      -2,
      -size / 2,
      size / 2,
      -2,
    ];
    const indices = [0, 1, 2, 2, 3, 0];
    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new Float32BufferAttribute(quad, 3));
    geometry.setIndex(indices);
    const uniforms = {
      uLineWidth: {value: 2},
      uMouse: {value: new Vector2()},
      uMouseIntersecting: {value: false},
      uRadius: {value: 10},
      uTileSize: {value: 3},
    };
    const material = new ShaderMaterial({
      fragmentShader: frag,
      vertexShader: vert,
      side: DoubleSide,
      uniforms,
      transparent: true,
    });
    return new Mesh(geometry, material);
  }
}
