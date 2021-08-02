/**
 * @format
 */

import {
  CameraVisualizationMode,
  OriginalPositionMode,
} from '../../node_modules/mapillary-js/dist/mapillary.module.js';
import {CameraControlMode} from '../ui/modes.js';

export class KeyController {
  constructor(options) {
    this._emitter = options.emitter;
    this._eventTypes = options.eventTypes;
    this._config = options.config;

    const decrease = 0.9;
    const increase = 1.1;
    this._commands = {
      // visibility
      c: {value: 'axesVisible'},
      y: {value: 'gridVisible'},
      m: {value: 'basemapVisible'},
      d: {value: 'cellsVisible'},
      e: {value: 'commandsVisible'},
      r: {value: 'imagesVisible'},
      f: {value: 'pointsVisible'},
      t: {value: 'statsVisible'},
      v: {value: 'thumbnailVisible'},
      // activity
      l: {value: 'datToggle'},
      // mode
      '1': {value: 'cameraVisualizationMode'},
      '2': {value: 'originalPositionMode'},
      '3': {value: 'cameraControlMode'},
      // size
      q: {value: 'pointSize', coeff: decrease},
      w: {value: 'pointSize', coeff: increase},
      a: {value: 'cameraSize', coeff: decrease},
      s: {value: 'cameraSize', coeff: increase},
      z: {value: 'infoSize', coeff: decrease},
      x: {value: 'infoSize', coeff: increase},
    };

    this._bindKeys();
    this._customCommands = {};
  }

  get commands() {
    return this._commands;
  }

  addCommand(command) {
    const key = command.key;
    if (this._has(key)) {
      throw new Error(`Command already exists ${key}`);
    }
    this._customCommands[key] = command.handler;
  }

  _bindKeys() {
    window.document.addEventListener('keydown', event => {
      if (event.altKey || event.ctrlKey || event.metaKey || event.shiftKey) {
        return;
      }

      const key = event.key;
      if (!this._has(key)) {
        return;
      }
      const customCommands = this._customCommands;
      if (key in customCommands) {
        customCommands[key]();
        return;
      }

      const emitter = this._emitter;
      const command = this._commands[key];
      const type = this._eventTypes[command.value];
      switch (key) {
        case 'c':
        case 'd':
        case 'e':
        case 'f':
        case 'm':
        case 'r':
        case 't':
        case 'v':
        case 'y':
          const visible = this._toggle(command.value);
          emitter.fire(type, {type, visible});
          break;
        case 'l':
          emitter.fire(type, {type});
          break;

        case '1':
          const cvm = this._rotateCvm();
          emitter.fire(type, {type, mode: cvm});
          break;
        case '2':
          const opm = this._rotateOpm();
          emitter.fire(type, {type, mode: opm});
          break;
        case '3':
          const ccm = this._rotateCcm();
          emitter.fire(type, {type, mode: ccm});
          break;
        case 'a':
        case 'q':
        case 's':
        case 'w':
        case 'x':
        case 'z':
          const size = this._changeSize(command.value, command.coeff);
          emitter.fire(type, {size, type});
          break;
        default:
          break;
      }
    });
  }

  _changeSize(command, coeff) {
    const config = this._config;
    config[command] *= coeff;
    config[command] = Math.max(0.01, Math.min(1, config[command]));
    return config[command];
  }

  _has(key) {
    return key in this._commands || key in this._customCommands;
  }

  _rotateCcm() {
    const ccm = CameraControlMode;
    const orbit = ccm.ORBIT;
    const earth = ccm.EARTH;
    const street = ccm.STREET;

    const modeRotation = {};
    modeRotation[orbit] = earth;
    modeRotation[earth] = street;
    modeRotation[street] = orbit;

    const config = this._config;
    const mode = config.cameraControlMode;
    config.cameraControlMode = modeRotation[mode];
    return config.cameraControlMode;
  }

  _rotateCvm() {
    const cvm = CameraVisualizationMode;
    const hidden = cvm.Hidden;
    const homogeneous = cvm.Homogeneous;
    const cluster = cvm.Cluster;
    const connectedComponent = cvm.ConnectedComponent;
    const sequence = cvm.Sequence;

    const modeRotation = {};
    modeRotation[hidden] = homogeneous;
    modeRotation[homogeneous] = cluster;
    modeRotation[cluster] = connectedComponent;
    modeRotation[connectedComponent] = sequence;
    modeRotation[sequence] = hidden;

    const config = this._config;
    const mode = cvm[config.cameraVisualizationMode];
    config.cameraVisualizationMode = cvm[modeRotation[mode]];
    return cvm[config.cameraVisualizationMode];
  }

  _rotateOpm() {
    const opm = OriginalPositionMode;
    const hidden = opm.Hidden;
    const flat = opm.Flat;
    const altitude = opm.Altitude;

    const modeRotation = {};
    modeRotation[hidden] = flat;
    modeRotation[flat] = altitude;
    modeRotation[altitude] = hidden;

    const config = this._config;
    const mode = opm[config.originalPositionMode];
    config.originalPositionMode = opm[modeRotation[mode]];
    return opm[config.originalPositionMode];
  }

  _toggle(command) {
    const config = this._config;
    config[command] = !config[command];
    return config[command];
  }
}
