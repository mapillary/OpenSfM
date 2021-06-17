/**
 * @format
 */

import {GUI} from '../../node_modules/dat.gui/build/dat.gui.module.js';
import {
  CameraVisualizationMode,
  OriginalPositionMode,
} from '../../node_modules/mapillary-js/dist/mapillary.module.js';
import {ListController} from './ListController.js';
import {CameraControlMode} from '../ui/modes.js';

export const FolderName = Object.freeze({
  INFO: 'info',
  IO: 'io',
  SPATIAL: 'spatial',
});

export class DatController {
  constructor(options) {
    this._emitter = options.emitter;
    this._eventTypes = options.eventTypes;

    this._config = options.config;

    this._gui = new GUI();
    const gui = this._gui;
    gui.width = 274;
    gui.close();

    this._folders = new Map();
    this._folders.set(FolderName.SPATIAL, this._createSpatialFolder(gui));
    this._folders.set(FolderName.INFO, this._createInfoFolder(gui));
    this._folders.set(FolderName.IO, this._createIOFolder(gui));

    this._listControllers = {};
    this._listControllers[
      'reconstruction'
    ] = this._createReconstructionsController(gui);
  }

  get gui() {
    return this._gui;
  }

  addReconstructionItems(items) {
    this._listControllers['reconstruction'].addItems(items);
  }

  addController(controller, folderName) {
    controller.addToFolder(this._folders.get(folderName));
  }

  _addBooleanOption(name, folder) {
    folder
      .add(this._config, name)
      .listen()
      .onChange(v => this._onChange(name, v));
  }

  _addCameraControlOption(folder) {
    const ccm = CameraControlMode;
    const ccms = [ccm.ORBIT, ccm.STREET, ccm.EARTH];
    folder
      .add(this._config, 'cameraControlMode', ccms)
      .listen()
      .onChange(m => this._onChange('cameraControlMode', m));
  }

  _addCameraVizualizationOption(folder) {
    const cvm = CameraVisualizationMode;
    const cvms = [
      cvm[cvm.Hidden],
      cvm[cvm.Homogeneous],
      cvm[cvm.Cluster],
      cvm[cvm.ConnectedComponent],
      cvm[cvm.Sequence],
    ];
    folder
      .add(this._config, 'cameraVisualizationMode', cvms)
      .listen()
      .onChange(m => this._onChange('cameraVisualizationMode', cvm[m]));
  }

  _addNumericOption(name, folder) {
    folder
      .add(this._config, name, 0, 1)
      .listen()
      .onChange(v => this._onChange(name, v));
  }

  _addPositionVisualizationOption(folder) {
    const opm = OriginalPositionMode;
    const opms = [opm[opm.Hidden], opm[opm.Flat], opm[opm.Altitude]];
    folder
      .add(this._config, 'originalPositionMode', opms)
      .listen()
      .onChange(m => this._onChange('originalPositionMode', opm[m]));
  }

  _createInfoFolder(gui) {
    const folder = gui.addFolder('Info');
    folder.open();
    this._addBooleanOption('commandsVisible', folder);
    this._addBooleanOption('thumbnailVisible', folder);
    this._addBooleanOption('statsVisible', folder);
    this._addNumericOption('infoSize', folder);
    return folder;
  }

  _createIOFolder(gui) {
    const folder = gui.addFolder('IO');
    folder.open();
    return folder;
  }

  _createReconstructionsController(gui) {
    const emitter = this._emitter;
    const eventType = this._eventTypes.reconstructionsSelected;
    const folder = gui.addFolder('Clusters');
    folder.open();
    const controller = new ListController({emitter, eventType, folder});
    return controller;
  }

  _createSpatialFolder(gui) {
    const folder = gui.addFolder('Spatial');
    folder.open();
    this._addBooleanOption('pointsVisible', folder);
    this._addNumericOption('pointSize', folder);
    this._addCameraVizualizationOption(folder);
    this._addNumericOption('cameraSize', folder);
    this._addPositionVisualizationOption(folder);
    this._addCameraControlOption(folder);
    this._addBooleanOption('cellsVisible', folder);
    this._addBooleanOption('imagesVisible', folder);
    this._addBooleanOption('axesVisible', folder);
    this._addBooleanOption('gridVisible', folder);
    this._addBooleanOption('basemapVisible', folder);
    return folder;
  }

  _onChange(name, value) {
    const emitter = this._emitter;
    const types = this._eventTypes;
    let mode = null;
    let type = null;
    switch (name) {
      case 'axesVisible':
      case 'cellsVisible':
      case 'commandsVisible':
      case 'gridVisible':
      case 'basemapVisible':
      case 'imagesVisible':
      case 'pointsVisible':
      case 'statsVisible':
      case 'thumbnailVisible':
        const visible = value;
        type = types[name];
        emitter.fire(type, {type, visible});
        break;
      case 'cameraControlMode':
      case 'originalPositionMode':
      case 'cameraVisualizationMode':
        mode = value;
        type = types[name];
        emitter.fire(type, {mode, type});
        break;
      case 'cameraSize':
      case 'pointSize':
      case 'infoSize':
        const size = value;
        type = types[name];
        emitter.fire(type, {size, type});
        break;
      default:
        break;
    }
  }
}
