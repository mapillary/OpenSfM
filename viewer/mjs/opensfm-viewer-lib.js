class EventEmitter {
    constructor() { this._listeners = {}; }

    on(type, callback) {
        if (!(type in this._listeners)) { this._listeners[type] = []; }
        this._listeners[type].push(callback);
    }

    fire(type, event) {
        if (!(type in this._listeners)) { return; }
        for (const callback of this._listeners[type]) {
            callback.call(this, event);
        }
    }
}

class ThumbnailHelper {
    constructor(options) {
        this._viewer = options.viewer;
        this._config = options.config;
        this._infoContainer = this._createContainer();
        this._img = this._infoContainer.firstElementChild;
        this._span = this._infoContainer.lastElementChild;
        this._node = null;
    }

    hide() { this._infoContainer.classList.add('hidden'); }

    listen() {
        const Viewer = Mapillary.Viewer;
        this._viewer.on(Viewer.nodechanged, node => {
            this._node = node;
            this.change();
        });
    }

    change() {
        if (!this._node || !this._config.thumbnailVisible) { return; }
        const node = this._node;
        this._img.src = node.image.src;
        const infoText = `${node.clusterKey}::${node.sequenceKey}::${node.key}`;
        this._span.textContent = infoText;
    }

    setWidth(value) { this._infoContainer.style.width = `${100 * value}%`; }
    show() { this._infoContainer.classList.remove('hidden'); }

    _createContainer() {
        const document = window.document;
        const span = document.createElement('span');
        span.classList.add('info-text');

        const img = document.createElement('img');
        img.classList.add('info-image');

        const container = document.createElement('div');
        container.classList.add('info-container', 'hidden');
        container.appendChild(img);
        container.appendChild(span);
        document.body.appendChild(container);
        return container;
    }
}

class ListController {
    constructor(options) {
        this._eventType = options.eventType;
        this._eventEmitter = options.eventEmitter;

        this._config = {
            items: {},
            toggle: () => this._onToggle(),
        };

        this._folder = options.folder;
        this._toggle = options.folder.add(this._config, 'toggle');
        this._setToggleText();
    }

    addItems(ids) {
        const items = this._config.items;
        const folder = this._folder;
        for (const id of ids) {
            if (id in items) { throw new Error(`Item exists ${id}`); }
            items[id] = true;
            folder
                .add(items, id)
                .listen()
                .onChange(() => this._onChange());
        }
        this._setToggleText();
    }

    _checkAllActive() {
        const items = this._config.items;
        const ids = Object.keys(items);
        return ids.length && ids
            .map(id => items[id])
            .reduce((acc, val) => acc && val, true);
    }

    _onChange() {
        this._setToggleText();
        const items = this._config.items;
        const active = Object.keys(items).filter(id => items[id]);
        this._eventEmitter.fire(
            this._eventType, { active });
    }

    _onToggle() {
        const items = this._config.items;
        const all = this._checkAllActive(items);
        for (const id of Object.keys(items)) { items[id] = !all; }
        this._onChange();
    }

    _setToggleText() {
        const all = this._checkAllActive(this._items);
        if (all) { this._toggle.name('Hide all'); }
        else { this._toggle.name('Show all'); }
    }
}

class DatGuiHelper {
    constructor(options) {
        this._eventEmitter = options.eventEmitter;
        this._provider = options.provider;
        this._viewer = options.viewer;
        this._spatial = options.viewer.getComponent('spatialData');

        this._eventEmitter.on(
            'reconstructionschanged',
            event => this._setFilter(event.active.slice()));

        this._config = options.config;
        this._modeConfig = options.modeConfig;

        this._thumbnailHelper = new ThumbnailHelper(options);
        this._thumbnailHelper.listen();
        this._thumbnailHelper.setWidth(this._config.thumbnailSize);

        const gui = new dat.GUI();
        gui.width = 300;
        gui.close();
        this._createSpatialFolder(gui);
        this._createInfoFolder(gui);

        this._listControllers = {};
        this._listControllers['reconstruction'] =
            this._createReconstructionsController(gui);
        this._gui = gui;
    }

    get gui() { return this._gui; }

    addReconstructionItems(items) {
        this._listControllers['reconstruction'].addItems(items);
    }

    _addBooleanOption(name, folder) {
        folder
            .add(this._config, name)
            .listen()
            .onChange(v => this._onChange(name, v));
    }

    _addCameraVizualizationOption(folder) {
        const Spatial = Mapillary.SpatialDataComponent;
        const cvm = Spatial.CameraVisualizationMode;
        const cvms = [
            cvm[cvm.Default],
            cvm[cvm.Cluster],
            cvm[cvm.ConnectedComponent],
            cvm[cvm.Sequence],
        ];
        folder
            .add(this._modeConfig, 'cameraVisualizationMode', cvms)
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
        const Spatial = Mapillary.SpatialDataComponent;
        const opm = Spatial.OriginalPositionMode;
        const opms = [
            opm[opm.Hidden],
            opm[opm.Flat],
            opm[opm.Altitude],
        ];
        folder
            .add(this._modeConfig, 'originalPositionMode', opms)
            .listen()
            .onChange(m => this._onChange('originalPositionMode', opm[m]));
    }

    _configure(name) {
        const c = {};
        c[name] = this._config[name];
        this._spatial.configure(c);
    }

    _createInfoFolder(gui) {
        const folder = gui.addFolder('Info');
        folder.open();
        this._addBooleanOption('thumbnailVisible', folder);
        this._addNumericOption('thumbnailSize', folder);
    }

    _createReconstructionsController(gui) {
        const eventEmitter = this._eventEmitter;
        const eventType = 'reconstructionschanged';
        const folder = gui.addFolder('Reconstructions');
        folder.open();
        const reconstructionsController =
            new ListController({ eventEmitter, eventType, folder });
        return reconstructionsController;
    }

    _createSpatialFolder(gui) {
        const folder = gui.addFolder('Spatial');
        folder.open();
        this._addBooleanOption('pointsVisible', folder);
        this._addNumericOption('pointSize', folder);
        this._addBooleanOption('camerasVisible', folder);
        this._addNumericOption('cameraSize', folder);
        this._addCameraVizualizationOption(folder);
        this._addPositionVisualizationOption(folder);
        this._addBooleanOption('tilesVisible', folder);
        this._addBooleanOption('imagesVisible', folder);
        this._addBooleanOption('earthControls', folder);
    }

    _onChange(name, value) {
        switch (name) {
            case 'camerasVisible':
            case 'pointsVisible':
            case 'tilesVisible':
                this._configure(name);
                break;
            case 'earthControls':
                this._setDirectionComponent();
                this._configure(name);
                break;
            case 'imagesVisible':
                this._setImagesComponent();
                break;
            case 'originalPositionMode':
            case 'cameraVisualizationMode':
            case 'cameraSize':
            case 'pointSize':
                this._setValue(name, value);
                break;
            case 'thumbnailVisible':
                if (value) { this._thumbnailHelper.show(); }
                else { this._thumbnailHelper.hide(); }
                this._thumbnailHelper.change();
                break;
            case 'thumbnailSize':
                this._thumbnailHelper.setWidth(value);
            default:
                break;
        }
    }

    _setDirectionComponent() {
        if (this._config.earthControls) {
            this._viewer.deactivateComponent('direction');
        }
        else { this._viewer.activateComponent('direction'); }
    }

    _setFilter(reconstructions) {
        const filter = ['in', 'clusterKey', ...reconstructions];
        this._viewer.setFilter(filter);
    }

    _setImagesComponent() {
        if (this._config.imagesVisible) {
            this._viewer.activateComponent('imagePlane');
        }
        else { this._viewer.deactivateComponent('imagePlane'); }
    }

    _setValue(name, size) {
        this._config[name] = size;
        this._configure(name);
    }
}

class KeyHandler {
    constructor(options) {
        this._viewer = options.viewer;
        this._spatial = options.viewer.getComponent('spatialData');

        this._config = options.config;
        this._modeConfig = options.modeConfig;
    }

    bindKeys() {
        window.document.addEventListener(
            'keydown',
            e => {
                let name = null;
                switch (e.key) {
                    case 'c': name = 'camerasVisible'; break;
                    case 'p': name = 'pointsVisible'; break;
                    case 't': name = 'tilesVisible'; break;
                    case 'i': this._toggleImages(); break;
                    case 'v': this._rotateCvm(); break;
                    case 'o': this._rotateOpm(); break;
                    case 'q': this._changeSize('pointSize', 0.9); break;
                    case 'w': this._changeSize('pointSize', 1.1); break;
                    case 'a': this._changeSize('cameraSize', 0.9); break;
                    case 's': this._changeSize('cameraSize', 1.1); break;
                    case 'e':
                        this._toggleBooleanSetting('earthControls');
                        this._setDirectionComponent();
                        break;
                    default: break;
                }

                if (!!name) { this._toggleBooleanSetting(name); }
            });
    }

    _changeSize(name, coeff) {
        const config = this._config;
        config[name] *= coeff;
        config[name] = Math.max(0.01, Math.min(1, config[name]));
        this._configure(name);
    }

    _configure(name) {
        const c = {}; c[name] = this._config[name];
        this._spatial.configure(c);
    }

    _rotateCvm() {
        const mode = Mapillary.SpatialDataComponent.CameraVisualizationMode;

        const none = mode.Default;
        const cluster = mode.Cluster;
        const connectedComponent = mode.ConnectedComponent;
        const sequence = mode.Sequence;

        const modeRotation = {};
        modeRotation[none] = cluster;
        modeRotation[cluster] = connectedComponent;
        modeRotation[connectedComponent] = sequence;
        modeRotation[sequence] = none;

        const config = this._config;
        config.cameraVisualizationMode =
            modeRotation[config.cameraVisualizationMode];
        this._configure('cameraVisualizationMode');
        this._modeConfig.cameraVisualizationMode =
            mode[config.cameraVisualizationMode];
    }

    _rotateOpm() {
        const mode = Mapillary.SpatialDataComponent.OriginalPositionMode;

        const hidden = mode.Hidden;
        const flat = mode.Flat;
        const altitude = mode.Altitude;

        const modeRotation = {};
        modeRotation[hidden] = flat;
        modeRotation[flat] = altitude;
        modeRotation[altitude] = hidden;

        const config = this._config;
        config.originalPositionMode =
            modeRotation[config.originalPositionMode];
        this._configure('originalPositionMode');
        this._modeConfig.originalPositionMode =
            mode[config.originalPositionMode];
    }

    _setDirectionComponent() {
        if (this._config.earthControls) {
            this._viewer.deactivateComponent('direction');
        }
        else { this._viewer.activateComponent('direction'); }
    }

    _setImagesComponent() {
        if (this._config.imagesVisible) {
            this._viewer.activateComponent('imagePlane');
        }
        else { this._viewer.deactivateComponent('imagePlane'); }
    }

    _toggleBooleanSetting(name) {
        const config = this._config;
        config[name] = !config[name];
        this._configure(name);
    }

    _toggleImages() {
        const config = this._config;
        config.imagesVisible = !config.imagesVisible;
        this._setImagesComponent();
    }
}

class OptionChangeHandler {
    constructor(options) {
        const eventEmitter = new EventEmitter();
        const config = Object.assign(
            {},
            options.viewer
                .getComponent('spatialData')
                .defaultConfiguration,
            {
                imagesVisible: false,
                thumbnailVisible: false,
                thumbnailSize: 0.2,
            },
            options.spatialConfiguration);

        const cvm = Mapillary.SpatialDataComponent.CameraVisualizationMode;
        const opm = Mapillary.SpatialDataComponent.OriginalPositionMode;
        const modeConfig = {
            'cameraVisualizationMode': cvm[config.cameraVisualizationMode],
            'originalPositionMode': opm[config.originalPositionMode],
        };

        const internalOptions = Object.assign(
            {},
            options,
            { config, eventEmitter, modeConfig });
        this._guiHelper = new DatGuiHelper(internalOptions);
        this._keyHandler = new KeyHandler(internalOptions);
        this._keyHandler.bindKeys();
        this._eventEmitter = eventEmitter;
    }

    get guiHelper() { return this._guiHelper; }
    get eventEmitter() { return this._eventEmitter; }

    on(type, callback) { this._eventEmitter.on(type, callback); }
}

class AbortFileLoadError extends Error {
    constructor(message) {
        super(message);
        Object.setPrototypeOf(this, AbortFileLoadError.prototype);
        this.name = "AbortFileLoadError";
    }
}

class FileLoader {
    constructor(classNames) {
        this._dropper = this._createDropper(classNames);
        this._picker = this._createPicker(classNames);
        this._preventDefault = event => event.preventDefault();
        this._resolve = null;
        this._reject = null;
    }

    get isActive() { return window.document.body.contains(this._dropper); }

    getFile() {
        this._setTextContent('file');
        return this._getFiles().then(files => files[0]);
    }

    getFiles() {
        this._setTextContent('files');
        const input = this._picker.lastElementChild;
        input.setAttribute('multiple', 'multiple');
        return this._getFiles().then(files => {
            input.removeAttribute('multiple');
            return files;
        })
    }

    hide() {
        const preventDefault = this._preventDefault;
        const document = window.document;
        document.removeEventListener('dragover', preventDefault);
        document.removeEventListener('drop', preventDefault);

        const body = document.body;
        body.removeChild(this._dropper);
        body.removeChild(this._picker);

        this._resolve = null;
        const reject = this._reject;
        if (!!reject) {
            this._reject = null;
            reject(new AbortFileLoadError('Aborted file load'));
        }
    }

    show() {
        const preventDefault = this._preventDefault;
        const document = window.document;
        document.addEventListener('dragover', preventDefault);
        document.addEventListener('drop', preventDefault);

        const body = document.body;
        const dropper = this._dropper;
        const picker = this._picker;
        body.appendChild(picker);
        body.appendChild(dropper);
    }

    _createDropper(classNames) {
        const document = window.document;
        const span = document.createElement('span');
        span.classList.add('file-text');

        const dropper = document.createElement('div');
        dropper.classList.add('file-drop', ...classNames);
        dropper.appendChild(span);

        dropper.addEventListener('dragenter', event => {
            dropper.classList.add('file-drop-hover');
            event.preventDefault();
        });
        dropper.addEventListener('dragleave', event => {
            dropper.classList.remove('file-drop-hover');
            event.preventDefault();
        });
        dropper.addEventListener('dragover', event => {
            event.dataTransfer.dropEffect = 'copy';
            event.preventDefault();
        });

        return dropper;
    }

    _createPicker(classNames) {
        const document = window.document;
        const span = document.createElement('span');
        span.classList.add('file-text');

        const input = document.createElement('input');
        input.type = 'file';

        const picker = document.createElement('label');
        picker.classList.add('file-pick', ...classNames);
        picker.appendChild(span);
        picker.appendChild(input);

        return picker;
    }

    _getFiles() {
        const promise = new Promise((resolve, reject) => {
            this._resolve = resolve;
            this._reject = reject;

            this._picker.addEventListener('change', this._onChange);
            this._dropper.addEventListener('drop', this._onDrop);
        });

        return promise;
    }

    _onChange = (event) => {
        const resolve = this._resolve;
        this._resolve = null;
        this._reject = null;

        this._picker.removeEventListener('change', this._onChange);
        resolve(event.target.files);
    }

    _onDrop = (event) => {
        const resolve = this._resolve;
        const reject = this._reject;
        this._resolve = null;
        this._reject = null;

        this._dropper.removeEventListener('drop', this._onDrop);
        this._dropper.classList.remove('file-drop-hover');
        event.preventDefault();
        const items = event.dataTransfer.items;
        if (!items) { reject(new Error('No files loaded')); }

        const files = [];
        for (const item of items) {
            if (!this._verifyKind(item.kind)) { continue; }
            files.push(item.getAsFile());
        }

        if (files.length > 0) { resolve(files); }
        reject(new Error('No files loaded'));
    }

    _setTextContent(content) {
        this._picker.firstElementChild.textContent = `Choose ${content}`;
        this._dropper.firstElementChild.textContent = `Drop ${content}`;
    }

    _verifyKind(kind) {
        if (kind !== 'file') {
            console.warn(`Unrecognized format ${kind}`);
            return false;
        }
        return true;
    }
}
