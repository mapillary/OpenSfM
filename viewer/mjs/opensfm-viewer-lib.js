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

class ThumbnailControl {
    constructor(options) {
        this._cluster = this._createText('Cluster: ');
        this._sequence = this._createText('Sequence: ');
        this._image = this._createText('Image: ');
        this._thumb = this._createThumb();
        const container = this._createContainer();
        container.appendChild(this._thumb);
        container.appendChild(this._cluster.element);
        container.appendChild(this._sequence.element);
        container.appendChild(this._image.element);
        this._container = container;
        this._viewer = options.viewer;
        this._thumbnailVisible = options.thumbnailVisible;
        this._node = null;
    }

    get container() { return this._container; }

    hide() {
        if (!this._thumbnailVisible) { return; }
        this._container.classList.add('opensfm-hidden');
        this._thumbnailVisible = false;
    }

    listen() {
        const Viewer = Mapillary.Viewer;
        this._viewer.on(Viewer.nodechanged, node => {
            this._node = node;
            this.update();
        });
    }

    update() {
        if (!this._node || !this._thumbnailVisible) { return; }
        const node = this._node;
        this._thumb.src = node.image.src;
        this._setTextContent(this._cluster, node.clusterKey);
        this._setTextContent(this._sequence, node.sequenceKey);
        this._setTextContent(this._image, node.key);
    }

    setWidth(value) { this._container.style.width = `${100 * value}%`; }

    show() {
        if (this._thumbnailVisible) { return; }
        this._container.classList.remove('opensfm-hidden');
        this._thumbnailVisible = true;
    }

    _createContainer() {
        const header = document.createElement('span');
        header.classList.add('opensfm-info-text', 'opensfm-info-text-header');
        header.textContent = 'Thumbnail';

        const container = document.createElement('div');
        container.classList.add(
            'opensfm-thumb-container',
            'opensfm-hidden');
        container.appendChild(header);
        return container;
    }

    _createText(prefix) {
        const document = window.document;
        const element = document.createElement('span');
        element.classList.add('opensfm-info-text', 'opensfm-info-inline');
        const copier = new Copier({ container: element, copyText: null });
        return { copier, element, prefix };
    }

    _createThumb() {
        const thumb = document.createElement('img');
        thumb.classList.add('opensfm-thumb');
        return thumb;
    }

    _setTextContent(textItem, content) {
        textItem.element.textContent = textItem.prefix + content;
        textItem.copier.setCopyText(content);
    }
}

class InfoControl {
    constructor(options) {
        const container = document.createElement('div');
        container.classList.add('opensfm-info-container');
        options.beforeContainer.after(container);
        this._container = container;
        this._controls = [];
    }

    addControl(control) {
        this._container.appendChild(control.container);
        this._controls.push(control);
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

class DatController {
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

        const gui = new dat.GUI();
        gui.width = 274;
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
        this._addNumericOption('infoSize', folder);
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
                this._eventEmitter.fire(
                    'thumbnailvisiblechanged',
                    { visible: value });
                break;
            case 'infoSize':
                this._eventEmitter.fire('infosizechanged', { width: value });
                break;
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

class OptionController {
    constructor(options) {
        const eventEmitter = new EventEmitter();
        const config = Object.assign(
            {},
            options.viewer
                .getComponent('spatialData')
                .defaultConfiguration,
            {
                imagesVisible: options.imagesVisible,
                thumbnailVisible: options.thumbnailVisible,
                infoSize: options.infoSize,
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
        this._datController = new DatController(internalOptions);
        this._keyHandler = new KeyHandler(internalOptions);
        this._keyHandler.bindKeys();
        this._eventEmitter = eventEmitter;
        this._config = config;
    }

    get dat() { return this._datController; }
    get eventEmitter() { return this._eventEmitter; }
    get config() { return this._config; }

    on(type, callback) { this._eventEmitter.on(type, callback); }
}

class AbortFileLoadError extends Error {
    constructor(message) {
        super(message);
        Object.setPrototypeOf(this, AbortFileLoadError.prototype);
        this.name = 'AbortFileLoadError';
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
        span.classList.add('opensfm-file-text');

        const dropper = document.createElement('div');
        dropper.classList.add('opensfm-file-drop', ...classNames);
        dropper.appendChild(span);

        dropper.addEventListener('dragenter', event => {
            dropper.classList.add('opensfm-file-drop-hover');
            event.preventDefault();
        });
        dropper.addEventListener('dragleave', event => {
            dropper.classList.remove('opensfm-file-drop-hover');
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
        span.classList.add('opensfm-file-text');

        const input = document.createElement('input');
        input.type = 'file';

        const picker = document.createElement('label');
        picker.classList.add('opensfm-file-pick', ...classNames);
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
        this._dropper.classList.remove('opensfm-file-drop-hover');
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

class Copier {
    constructor(options) {
        if (!window.navigator || !window.navigator.clipboard) {
            // Clipboard requires secure origin (https or localhost)
            // or setting a browser flag.
            return;
        }

        const container = options.container;
        const onShowTypes = ['pointerenter'];
        for (const type of onShowTypes) {
            container.addEventListener(type, this._onShow);
        }

        window.addEventListener('blur', this._onHide);
        const onHideTypes = ['pointercancel', 'pointerleave'];
        for (const type of onHideTypes) {
            container.addEventListener(type, this._onHide);
        }

        this._onHideTypes = onHideTypes;
        this._onShowTypes = onShowTypes;
        this._textContent = 'Copy';

        this._copyText = options.copyText;
        this._container = container;
        this._resetCursor = this._container.style.cursor;
        this._container.style.cursor = 'pointer';
        this._popupContainer = null;
    }

    dispose() {
        if (!this._container) { return; }

        this._onHide()

        const container = this._container;
        container.style.cursor = this._resetCursor;

        const onShowTypes = this._onShowTypes;
        for (const type in onShowTypes) {
            container.removeEventListener(type, this._onShow);
        }

        window.removeEventListener('blur', this._onHide);
        const onHideTypes = this._onHideTypes;
        for (const type in onHideTypes) {
            container.removeEventListener(type, this._onHide);
        }

        this._onHideTypes = null;
        this._onShowTypes = null;
        this._container = null;
    }

    setCopyText(content) {
        this._copyText = content;
    }

    _onClick = async () => {
        try {
            const navigator = window.navigator;
            await navigator.clipboard.writeText(this._copyText);
            await this._showCopied();
        } catch (error) {
            console.error(error);
        }
    }

    _showCopied() {
        if (!this._popupContainer) { return; }
        const popup = this._popupContainer.firstElementChild;
        popup.textContent = 'Copied to clipboard';
        return new Promise((resolve) => {
            window.setTimeout(
                () => {
                    if (!this._popupContainer) { resolve(); return; }
                    const popup = this._popupContainer.firstElementChild;
                    popup.textContent = this._textContent;
                    resolve();
                },
                850);
        })
    }

    _onShow = () => {
        if (!!this._popupContainer) { return; }
        const container = this._container;
        const left = container.offsetLeft;
        const top = container.offsetTop;

        const document = window.document;
        const popup = document.createElement('div');
        popup.classList.add('opensfm-popup');
        popup.textContent = this._textContent;
        const arrow = document.createElement('div');
        arrow.classList.add('opensfm-popup-arrow');

        const popupContainer = document.createElement('div');
        popupContainer.classList.add('opensfm-popup-container');
        popupContainer.style.position = 'absolute';
        popupContainer.style.left = `${left + 2}px`;
        popupContainer.style.top = `${top}px`;
        popupContainer.appendChild(popup);
        popupContainer.appendChild(arrow);
        container.parentNode.appendChild(popupContainer);
        this._popupContainer = popupContainer;

        container.addEventListener('click', this._onClick);
    }

    _onHide = () => {
        const popup = this._popupContainer;
        if (!popup) { return; }
        this._container.parentNode.removeChild(popup);
        this._container.removeEventListener('click', this._onClick);
        this._popupContainer = null;
    }
}
