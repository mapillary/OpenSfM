/**
 * @format
 */

import {SpatialDataComponent as SDC} from '../../node_modules/mapillary-js/dist/mapillary.module.js';
import {EventEmitter} from '../util/EventEmitter.js';
import {DatController} from './DatController.js';
import {KeyController} from './KeyController.js';

export class OptionController {
  constructor(options) {
    const cvm = SDC.CameraVisualizationMode;
    const opm = SDC.OriginalPositionMode;
    const config = Object.assign({}, options, {
      cameraVisualizationMode: cvm[options.cameraVisualizationMode],
      originalPositionMode: opm[options.originalPositionMode],
    });
    const eventTypes = {
      cameraSize: 'camerasize',
      camerasVisible: 'camerasvisible',
      cameraVisualizationMode: 'cameravisualizationmode',
      commandsVisible: 'commandsvisible',
      datToggle: 'dattoggle',
      earthControls: 'earthcontrols',
      imagesVisible: 'imagesvisible',
      infoSize: 'infosize',
      originalPositionMode: 'originalpositionmode',
      pointSize: 'pointsize',
      pointsVisible: 'pointsvisible',
      reconstructionsSelected: 'reconstructionsselected',
      thumbnailVisible: 'thumbnailvisible',
      tilesVisible: 'tilesvisible',
    };
    const emitter = new EventEmitter();
    const internalOptions = {config, emitter, eventTypes};
    this._datController = new DatController(internalOptions);
    this._keyController = new KeyController(internalOptions);
    this._emitter = emitter;
    this._config = config;
  }

  get config() {
    return this._config;
  }
  get dat() {
    return this._datController;
  }
  get emitter() {
    return this._emitter;
  }
  get key() {
    return this._keyController;
  }

  on(type, callback) {
    this._emitter.on(type, callback);
  }
}
