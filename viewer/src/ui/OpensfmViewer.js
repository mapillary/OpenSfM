/**
 * @format
 */

import {
  CameraControls,
  CameraVisualizationMode,
  OriginalPositionMode,
  RenderMode,
  Viewer,
} from '../../node_modules/mapillary-js/dist/mapillary.module.js';
import {EventEmitter} from '../util/EventEmitter.js';
import {CommandExplainerControl} from '../control/CommandExplainerControl.js';
import {InfoControl} from '../control/InfoControl.js';
import {FolderName} from '../controller/DatController.js';
import {OptionController} from '../controller/OptionController.js';
import {ThumbnailControl} from '../control/ThumbnailControl.js';
import {FileController} from '../controller/FileController.js';

export class OpensfmViewer extends EventEmitter {
  constructor(options) {
    super();

    this._params = options.params;
    this._provider = options.provider;

    const document = window.document;
    const container = document.createElement('div');
    container.classList.add('opensfm-viewer');
    document.body.appendChild(container);

    const cvm = CameraVisualizationMode.Homogeneous;
    const opm = OriginalPositionMode.Hidden;
    const spatialConfiguration = {
      cameraSize: 0.5,
      cameraVisualizationMode: cvm,
      originalPositionMode: opm,
      pointSize: 0.2,
      pointsVisible: true,
      tilesVisible: false,
    };

    const imagesVisible = false;
    const cameraControls = CameraControls.Earth;
    this._viewer = new Viewer({
      apiClient: this._provider,
      cameraControls,
      combinedPanning: false,
      component: {
        bearing: false,
        cover: false,
        direction: false,
        image: imagesVisible,
        sequence: false,
        spatial: spatialConfiguration,
        zoom: false,
      },
      container,
      imageTiling: false,
      renderMode: RenderMode.Letterbox,
    });
    const viewer = this._viewer;
    this._spatial = viewer.getComponent('spatial');

    const infoSize = 0.3;
    const thumbnailVisible = false;
    const commandsVisible = true;

    const controllerOptions = {
      cameraControls,
      commandsVisible,
      imagesVisible,
      infoSize,
      thumbnailVisible,
    };
    this._optionController = new OptionController(
      Object.assign(
        {},
        viewer.getComponent('spatial').defaultConfiguration,
        spatialConfiguration,
        controllerOptions,
      ),
    );

    this._thumbnailControl = new ThumbnailControl({
      visible: thumbnailVisible,
      provider: this._provider,
    });
    this._commandExplainerControl = new CommandExplainerControl({
      visible: commandsVisible,
    });
    this._commandExplainerControl.add(this._optionController.key.commands);

    this._infoControl = new InfoControl({
      beforeContainer: viewer.getContainer(),
    });
    this._infoControl.addControl(this._commandExplainerControl);
    this._infoControl.addControl(this._thumbnailControl);
    this._infoControl.setWidth(infoSize);

    this._fileController = new FileController({
      classNames: ['opensfm-file-container'],
      itemsUrl: 'items',
      showExitButton: true,
    });
    this._optionController.dat.addController(
      this._fileController,
      FolderName.IO,
    );
    const toggleCommand = {
      key: 'p',
      value: 'toggleFileLoader',
      handler: async () => await this._fileController.toggle(),
    };
    this._optionController.key.addCommand(toggleCommand);
    this._commandExplainerControl.add({[toggleCommand.key]: toggleCommand});
  }

  get commands() {
    return this._commandExplainerControl;
  }
  get file() {
    return this._fileController;
  }
  get info() {
    return this._infoControl;
  }
  get option() {
    return this._optionController;
  }
  get params() {
    return this._params;
  }
  get provider() {
    return this._provider;
  }
  get viewer() {
    return this._viewer;
  }

  initialize() {
    this._listen();
    this._move();
    this._loadProvider().then(provider => {
      const items = Object.keys(provider.data.clusters);
      this._optionController.dat.addReconstructionItems(items);
    });
  }

  _configure(config) {
    this._spatial.configure(config);
  }

  _getRandomImageId(images) {
    return Object.keys(images)[0];
  }

  _listen() {
    this._fileController.on('load', event => this._onFileLoad(event));

    const optionController = this._optionController;
    optionController.on('cameracontrols', event =>
      this._onCameraControls(event),
    );
    optionController.on('camerasize', event => this._onCameraSize(event));
    optionController.on('cameravisualizationmode', event =>
      this._onCameraVisualizationMode(event),
    );
    optionController.on('commandsvisible', event =>
      this._onCommandsVisible(event),
    );
    optionController.on('dattoggle', event => this._onDatToggle(event));
    optionController.on('imagesvisible', event => this._onImagesVisible(event));
    optionController.on('infosize', event => this._onInfoSize(event));
    optionController.on('originalpositionmode', event =>
      this._onOriginalPositionMode(event),
    );
    optionController.on('pointsize', event => this._onPointSize(event));
    optionController.on('pointsvisible', event => this._onPointsVisible(event));
    optionController.on('thumbnailvisible', event =>
      this._onThumbnailVisible(event),
    );
    optionController.on('tilesvisible', event => this._onTilesVisible(event));
    optionController.on('reconstructionsselected', event =>
      this._onReconstructionsSelected(event),
    );

    this._provider.on('opensfmdatacreate', event =>
      this._onProviderOpensfmDataCreate(event),
    );

    this._viewer.on('image', event => this._onViewerImage(event));
  }

  _loadProvider() {
    const provider = this._provider;
    return new Promise(resolve => {
      if (provider.loaded) {
        resolve(provider);
        return;
      }
      provider.on('load', event => {
        resolve(event.target);
      });
    });
  }

  async _move() {
    const params = this._params;
    if (!!params.img) {
      this._moveTo(this._viewer, params.img);
    } else {
      const loadedProvider = await this._loadProvider();
      this._moveTo(
        this._viewer,
        this._getRandomImageId(loadedProvider.data.images),
      );
    }
  }

  _moveTo(viewer, imageId) {
    viewer.moveTo(imageId).catch(error => console.error(error));
  }

  _onCameraSize(event) {
    this._configure({cameraSize: event.size});
  }

  _onCameraVisualizationMode(event) {
    this._configure({cameraVisualizationMode: event.mode});
  }

  _onCommandsVisible(event) {
    if (event.visible) {
      this._commandExplainerControl.show();
    } else {
      this._commandExplainerControl.hide();
    }
  }

  _onDatToggle() {
    const dat = this._optionController.dat;
    if (dat.gui.closed) {
      dat.gui.open();
    } else {
      dat.gui.close();
    }
  }

  _onCameraControls(event) {
    const mode = event.mode;
    const bearing = 'bearing';
    const direction = 'direction';
    const zoom = 'zoom';
    this._viewer.setCameraControls(mode);
    if (mode === CameraControls.Earth) {
      this._viewer.deactivateComponent(bearing);
      this._viewer.deactivateComponent(direction);
      this._viewer.deactivateComponent(zoom);
    } else {
      this._viewer.activateComponent(bearing);
      this._viewer.activateComponent(direction);
      this._viewer.activateComponent(zoom);
    }
  }

  async _onFileLoad(event) {
    try {
      await this._provider.add(event.file);
    } catch (error) {
      console.error(error);
    }
  }

  _onImagesVisible(event) {
    if (event.visible) {
      this._viewer.activateComponent('image');
    } else {
      this._viewer.deactivateComponent('image');
    }
  }

  _onInfoSize(event) {
    this._infoControl.setWidth(event.size);
  }

  _onOriginalPositionMode(event) {
    this._configure({originalPositionMode: event.mode});
  }

  _onPointSize(event) {
    this._configure({pointSize: event.size});
  }

  _onPointsVisible(event) {
    this._configure({pointsVisible: event.visible});
  }

  _onProviderOpensfmDataCreate(event) {
    const clusters = Object.keys(event.data.clusters);
    this._optionController.dat.addReconstructionItems(clusters);
  }

  _onReconstructionsSelected(event) {
    const filter = ['in', 'clusterId', ...event.active];
    this._viewer.setFilter(filter);
  }

  _onTilesVisible(event) {
    this._configure({tilesVisible: event.visible});
  }

  _onThumbnailVisible(event) {
    if (event.visible) {
      this._thumbnailControl.show();
    } else {
      this._thumbnailControl.hide();
    }
  }

  _onViewerImage(event) {
    this._thumbnailControl.update(event.image);
  }
}
