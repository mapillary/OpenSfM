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
import {CameraControlMode} from '../ui/modes.js';
import {pixelToViewport} from '../util/coords.js';

const dynamicFrag = `
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

const staticFrag = `
#ifdef GL_ES
precision mediump float;
#endif

uniform float uLineWidth;
uniform float uTileSize;

varying vec4 vWorldCoords;

void main()	{
    vec4 fragColor = vec4(0.0, 0.0, 0.0, 0.0);

    vec2 pos = vWorldCoords.xy / uTileSize;
    vec2 f = abs(pos - floor(pos + vec2(0.5, 0.5)));
    vec2 df = fwidth(pos) * uLineWidth;
    vec2 g = smoothstep(-df, df, f);
    float grid = 1.0 - clamp(g.x * g.y, 0.0, 1.0);
    fragColor.rgb = mix(fragColor.rgb, vec3(1.0, 1.0, 1.0), grid);
    fragColor.a = grid;

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
  constructor(options) {
    this._dynamicEarth = this._makeDynamicEarth();
    this._staticEarth = this._makeStaticEarth();
    this._scene = new Scene();

    this._camera = null;
    this._raycaster = new Raycaster();
    this._origin = new Vector3();
    this._direction = new Vector3();

    this.configure(options);
  }

  get active() {
    return !!this._camera;
  }

  get scene() {
    return this._scene;
  }

  configure(options) {
    this._scene.clear();
    this._resetIntersection();

    switch (options.mode) {
      case CameraControlMode.EARTH:
        this._scene.add(this._dynamicEarth);
        break;
      case CameraControlMode.ORBIT:
        this._scene.add(this._staticEarth);
        break;
      default:
        break;
    }
  }

  intersect(event) {
    if (!this.active) {
      throw new Error('Cannot intersect when inactive');
    }

    if (!this._scene.children.includes(this._dynamicEarth)) {
      return;
    }

    const camera = this._camera;
    const raycaster = this._raycaster;
    const origin = this._origin;
    const direction = this._direction;
    const uniforms = this._dynamicEarth.material.uniforms;

    const viewport = pixelToViewport(event.pixelPoint, this._container);
    origin.setFromMatrixPosition(camera.matrixWorld);
    direction
      .set(viewport[0], viewport[1], 0.5)
      .applyMatrix4(camera.projectionMatrixInverse)
      .applyMatrix4(camera.matrixWorld)
      .sub(origin)
      .normalize();
    raycaster.set(origin, direction);

    const intersections = raycaster.intersectObject(this._dynamicEarth);
    if (intersections.length) {
      const intersection = intersections[0];
      uniforms.uMouseIntersecting.value = true;
      uniforms.uMouse.value.x = intersection.point.x;
      uniforms.uMouse.value.y = intersection.point.y;
    } else {
      this._resetIntersection();
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

  _resetIntersection() {
    const uniforms = this._dynamicEarth.material.uniforms;
    uniforms.uMouseIntersecting.value = false;
    uniforms.uMouse.value.x = 0;
    uniforms.uMouse.value.y = 0;
  }

  _makeDynamicEarth() {
    const quad = this._makeQuad(SIZE);
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      'position',
      new Float32BufferAttribute(quad.vertices, 3),
    );
    geometry.setIndex(quad.indices);
    const uniforms = {
      uLineWidth: {value: 2},
      uMouse: {value: new Vector2()},
      uMouseIntersecting: {value: false},
      uRadius: {value: 10},
      uTileSize: {value: 5},
    };
    const material = new ShaderMaterial({
      depthWrite: false,
      fragmentShader: dynamicFrag,
      vertexShader: vert,
      side: DoubleSide,
      uniforms,
      transparent: true,
    });
    return new Mesh(geometry, material);
  }

  _makeStaticEarth() {
    const quad = this._makeQuad(100);
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      'position',
      new Float32BufferAttribute(quad.vertices, 3),
    );
    geometry.setIndex(quad.indices);
    const uniforms = {
      uLineWidth: {value: 2},
      uTileSize: {value: 5},
    };
    const material = new ShaderMaterial({
      depthWrite: false,
      fragmentShader: staticFrag,
      vertexShader: vert,
      side: DoubleSide,
      uniforms,
      transparent: true,
    });
    return new Mesh(geometry, material);
  }

  _makeQuad(size) {
    const vertices = [
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
    return {indices, vertices};
  }
}
