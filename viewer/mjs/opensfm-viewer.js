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

class CommandExplainerControl {
    constructor(options) {
        this._visible = options.visible;
        this._container = this._createContainer();
        this._popup = this._createPopup();
    }

    get container() { return this._container; }

    add(commands) {
        const lines = Object
            .keys(commands)
            .map(
                key => {
                    const value = commands[key].value;
                    return `'${key}' - ${value}`;
                });
        this._popup.appendLines(lines);
    }

    hide() {
        if (!this._visible) { return; }
        this._container.classList.add('opensfm-hidden');
        this._visible = false;
    }

    show() {
        if (this._visible) { return; }
        this._container.classList.remove('opensfm-hidden');
        this._visible = true;
    }

    _createContainer() {
        const header = document.createElement('span');
        header.classList.add('opensfm-info-text', 'opensfm-info-text-header');
        header.textContent = 'Commands';

        const container = document.createElement('div');
        container.classList.add(
            'opensfm-control-container',
            'opensfm-hidden');
        container.appendChild(header);
        return container;
    }

    _createPopup() {
        return new Popup({
            container: this._container.firstElementChild,
            up: false,
        });
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
        this._visible = options.visible;
        this._node = null;
    }

    get container() { return this._container; }

    hide() {
        if (!this._visible) { return; }
        this._container.classList.add('opensfm-hidden');
        this._visible = false;
    }

    update(node) {
        this._node = node;
        this._render();
    }

    show() {
        if (this._visible) { return; }
        this._container.classList.remove('opensfm-hidden');
        this._visible = true;
        this._render();
    }

    _createContainer() {
        const header = document.createElement('span');
        header.classList.add('opensfm-info-text', 'opensfm-info-text-header');
        header.textContent = 'Thumbnail';

        const container = document.createElement('div');
        container.classList.add(
            'opensfm-control-container',
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

    _render() {
        if (!this._node || !this._visible) { return; }
        const node = this._node;
        this._thumb.src = node.image.src;
        this._setTextContent(this._cluster, node.clusterKey);
        this._setTextContent(this._sequence, node.sequenceKey);
        this._setTextContent(this._image, node.key);
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

    setWidth(value) {
        const width = `${100 * value}%`;
        this._container.style.width = width;
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
        const type = this._eventType;
        this._eventEmitter.fire(type, { active, type });
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
        this._eventTypes = options.eventTypes;

        this._config = options.config;

        this._gui = new dat.GUI();
        const gui = this._gui;
        gui.width = 274;
        gui.close();
        this._createSpatialFolder(gui);
        this._createInfoFolder(gui);

        this._listControllers = {};
        this._listControllers['reconstruction'] =
            this._createReconstructionsController(gui);
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
        const Spatial = Mapillary.SpatialDataComponent;
        const opm = Spatial.OriginalPositionMode;
        const opms = [
            opm[opm.Hidden],
            opm[opm.Flat],
            opm[opm.Altitude],
        ];
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
        this._addNumericOption('infoSize', folder);
    }

    _createReconstructionsController(gui) {
        const eventEmitter = this._eventEmitter;
        const eventType = this._eventTypes.reconstructionsSelected;
        const folder = gui.addFolder('Reconstructions');
        folder.open();
        const controller =
            new ListController({ eventEmitter, eventType, folder });
        return controller;
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
        const emitter = this._eventEmitter;
        const types = this._eventTypes;
        const cvm = Mapillary.SpatialDataComponent.CameraVisualizationMode;
        const opm = Mapillary.SpatialDataComponent.OriginalPositionMode;
        let mode = null;
        let type = null;
        switch (name) {
            case 'camerasVisible':
            case 'commandsVisible':
            case 'imagesVisible':
            case 'pointsVisible':
            case 'thumbnailVisible':
            case 'tilesVisible':
                const visible = value;
                type = types[name];
                emitter.fire(type, { type, visible });
                break;
            case 'earthControls':
                const active = value;
                type = types[name];
                emitter.fire(type, { active, type });
                break;
            case 'originalPositionMode': mode = value;
            case 'cameraVisualizationMode': mode = value;
                type = types[name];
                emitter.fire(type, { mode, type });
                break;
            case 'cameraSize':
            case 'pointSize':
            case 'infoSize':
                const size = value;
                type = types[name];
                emitter.fire(type, { size, type });
                break;
            default:
                break;
        }
    }
}

class KeyHandler {
    constructor(options) {
        this._eventEmitter = options.eventEmitter;
        this._eventTypes = options.eventTypes;

        this._config = options.config;

        const decrease = 0.9;
        const increase = 1.1;
        this._commands = {
            // visibility
            'c': { value: 'camerasVisible' },
            'e': { value: 'commandsVisible' },
            'f': { value: 'pointsVisible' },
            'd': { value: 'tilesVisible' },
            'r': { value: 'imagesVisible' },
            'v': { value: 'thumbnailVisible' },
            // activity
            'o': { value: 'earthControls' },
            'l': { value: 'datToggle' },
            // mode
            '1': { value: 'cameraVisualizationMode' },
            '2': { value: 'originalPositionMode' },
            // size
            'q': { value: 'pointSize', coeff: decrease },
            'w': { value: 'pointSize', coeff: increase },
            'a': { value: 'cameraSize', coeff: decrease },
            's': { value: 'cameraSize', coeff: increase },
            'z': { value: 'infoSize', coeff: decrease },
            'x': { value: 'infoSize', coeff: increase },
        };

        this._bindKeys();
        this._customCommands = {};
    }

    get commands() { return this._commands; }

    addCommand(command) {
        const key = command.key;
        if (this._has(key)) {
            throw new Error(`Command already exists ${key}`);
        }
        this._customCommands[key] = command.handler;
    }

    _bindKeys() {
        window.document.addEventListener(
            'keydown',
            event => {
                const key = event.key;
                if (!this._has(key)) { return; }
                const customCommands = this._customCommands;
                if (key in customCommands) { customCommands[key](); return; }

                const emitter = this._eventEmitter;
                const command = this._commands[key];
                const type = this._eventTypes[command.value];
                switch (key) {
                    case 'c':
                    case 'd':
                    case 'e':
                    case 'f':
                    case 'r':
                    case 'v':
                        const visible = this._toggle(command.value);
                        emitter.fire(type, { type, visible });
                        break;
                    case 'l':
                        emitter.fire(type, { type });
                        break;
                    case 'o':
                        const active = this._toggle(command.value);
                        emitter.fire(type, { active, type });
                        break;
                    case '1':
                        const cvm = this._rotateCvm();
                        emitter.fire(type, { type, mode: cvm });
                        break;
                    case '2':
                        const opm = this._rotateOpm()
                        emitter.fire(type, { type, mode: opm });
                        break;
                    case 'a':
                    case 'q':
                    case 's':
                    case 'w':
                    case 'x':
                    case 'z':
                        const size = this._changeSize(
                            command.value,
                            command.coeff);
                        emitter.fire(type, { size, type });
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

    _has(key) { return key in this._commands || key in this._customCommands; }

    _rotateCvm() {
        const cvm = Mapillary.SpatialDataComponent.CameraVisualizationMode;
        const none = cvm.Default;
        const cluster = cvm.Cluster;
        const connectedComponent = cvm.ConnectedComponent;
        const sequence = cvm.Sequence;

        const modeRotation = {};
        modeRotation[none] = cluster;
        modeRotation[cluster] = connectedComponent;
        modeRotation[connectedComponent] = sequence;
        modeRotation[sequence] = none;

        const config = this._config;
        const mode = cvm[config.cameraVisualizationMode];
        config.cameraVisualizationMode = cvm[modeRotation[mode]];
        return cvm[config.cameraVisualizationMode];
    }

    _rotateOpm() {
        const opm = Mapillary.SpatialDataComponent.OriginalPositionMode;
        const hidden = opm.Hidden;
        const flat = opm.Flat;
        const altitude = opm.Altitude;

        const modeRotation = {};
        modeRotation[hidden] = flat;
        modeRotation[flat] = altitude;
        modeRotation[altitude] = hidden;

        const config = this._config;
        const mode = opm[config.originalPositionMode];
        config.originalPositionMode =
            opm[modeRotation[mode]];
        return opm[config.originalPositionMode];
    }

    _toggle(command) {
        const config = this._config;
        config[command] = !config[command];
        return config[command];
    }
}

class OptionController {
    constructor(options) {
        const eventEmitter = new EventEmitter();
        const cvm = Mapillary.SpatialDataComponent.CameraVisualizationMode;
        const opm = Mapillary.SpatialDataComponent.OriginalPositionMode;
        const config = Object.assign(
            {},
            options,
            {
                'cameraVisualizationMode': cvm[options.cameraVisualizationMode],
                'originalPositionMode': opm[options.originalPositionMode],
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
        const internal = { config, eventEmitter, eventTypes };
        this._datController = new DatController(internal);
        this._keyHandler = new KeyHandler(internal);
        this._eventEmitter = eventEmitter;
        this._config = config;
    }

    get config() { return this._config; }
    get dat() { return this._datController; }
    get eventEmitter() { return this._eventEmitter; }
    get key() { return this._keyHandler; }

    on(type, callback) { this._eventEmitter.on(type, callback); }
}

class CancelledError extends Error {
    constructor(message) {
        super(message);
        Object.setPrototypeOf(this, CancelledError.prototype);
        this.name = 'CancelledError';
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
            reject(new CancelledError('Cancelled file load'));
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

class Popup {
    constructor(options) {
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

        this._container = container;
        this._lines = [];
        this._up = options.up;
        this._popup = null;
        this._popupContainer = null;
    }

    appendLines(lines) {
        this._lines.push(...lines);
        this._appendLines(lines.slice());
    }

    setLines(lines) {
        this._lines = lines.slice();
        this._clearLines();
        this._appendLines(this._lines);
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

    _appendLines(lines) {
        const popup = this._popup;
        if (!popup) { return; }
        for (const line of lines) {
            const span = document.createElement('span');
            span.style.display = 'block';
            span.textContent = line;
            popup.appendChild(span);
        }
    }

    _clearLines() {
        const popup = this._popup;
        if (!popup) { return; }
        while (popup.lastElementChild) {
            popup.removeChild(popup.lastElementChild);
        }
    }

    _onShow = () => {
        if (!!this._popup) { return; }
        const container = this._container;
        const left = container.offsetLeft;
        const boundingRect = container.getBoundingClientRect();
        const up = this._up;
        const top = up ?
            container.offsetTop :
            container.offsetTop + boundingRect.height;

        const directionClassname = up ?
            'opensfm-popup-up' : 'opensfm-popup-down';
        const document = window.document;
        const popup = document.createElement('div');
        popup.classList.add('opensfm-popup');

        const arrow = document.createElement('div');
        arrow.classList.add('opensfm-popup-arrow', directionClassname);

        const popupContainer = document.createElement('div');
        popupContainer.classList.add(
            'opensfm-popup-container',
            directionClassname);
        popupContainer.style.position = 'absolute';
        popupContainer.style.left = `${left + 2}px`;
        popupContainer.style.top = `${top}px`;
        popupContainer.appendChild(popup);
        if (up) { popupContainer.appendChild(arrow); }
        else { popup.before(arrow); }
        container.parentNode.appendChild(popupContainer);

        this._popupContainer = popupContainer;
        this._popup = popup;
        this._appendLines(this._lines);
    };

    _onHide = () => {
        const popupContainer = this._popupContainer;
        if (!popupContainer) { return; }
        this._container.parentNode.removeChild(popupContainer);
        this._popupContainer = null;
        this._popup = null;
    };
}

class Copier {
    constructor(options) {
        if (!window.navigator || !window.navigator.clipboard) {
            // Clipboard requires secure origin (https or localhost)
            // or setting a browser flag.
            return;
        }

        this._copyText = options.copyText;
        const container = options.container;
        this._resetCursor = container.style.cursor;
        container.style.cursor = 'pointer';
        container.addEventListener('click', this._onClick);

        this._popup = new Popup({ container, up: true });
        this._popup.setLines(['Copy'])
        this._container = container;
    }

    dispose() {
        if (!this._container) { return; }
        this._popup.dispose();
        this._popup = null;
        const container = this._container;
        container.removeEventListener('click', this._onClick);
        container.style.cursor = this._resetCursor;
        this._container = null;
    }

    setCopyText(content) { this._copyText = content; }

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
        if (!this._popup) { return; }
        this._popup.setLines(['Copied to clipboard']);
        return new Promise((resolve) => {
            window.setTimeout(
                () => {
                    if (!this._popup) { resolve(); return; }
                    this._popup.setLines(['Copy']);
                    resolve();
                },
                850);
        })
    }
}

class OpenSfmViewer {
    constructor(options) {
        this._params = options.params;
        this._provider = options.provider;

        const document = window.document;
        const container = document.createElement('div');
        container.classList.add('opensfm-viewer');
        document.body.appendChild(container);

        const spc = Mapillary.SpatialDataComponent;
        const cvm = spc.CameraVisualizationMode.Default;
        const opm = spc.OriginalPositionMode.Hidden;
        const spatialConfiguration = {
            cameraSize: 0.5,
            camerasVisible: true,
            earthControls: true,
            pointSize: 0.2,
            originalPositionMode: opm,
            pointsVisible: true,
            tilesVisible: false,
            cameraVisualizationMode: cvm,
        };
        const imagesVisible = false;

        this._viewer = new Mapillary.Viewer({
            apiClient: this._provider,
            combinedPanning: false,
            component: {
                cover: false,
                direction: false,
                imagePlane: imagesVisible,
                sequence: false,
                spatialData: spatialConfiguration,
            },
            container,
            renderMode: Mapillary.RenderMode.Letterbox,
        });
        const viewer = this._viewer;
        this._spatial = viewer.getComponent('spatialData');
        window.addEventListener('resize', () => viewer.resize());

        const infoSize = 0.3;
        const thumbnailVisible = false;
        const commandsVisible = false;

        this._optionController = new OptionController(
            Object.assign(
                {},
                viewer.getComponent('spatialData').defaultConfiguration,
                {
                    commandsVisible,
                    imagesVisible,
                    infoSize,
                    thumbnailVisible,
                },
                spatialConfiguration));

        this._thumbnailControl = new ThumbnailControl({
            visible: thumbnailVisible,
        })
        this._commandExplainerControl = new CommandExplainerControl({
            visible: commandsVisible,
        });
        this._commandExplainerControl.add(
            this._optionController.key.commands);

        this._infoControl = new InfoControl({
            beforeContainer: viewer.getContainer(),
        });
        this._infoControl.addControl(this._commandExplainerControl);
        this._infoControl.addControl(this._thumbnailControl);
        this._infoControl.setWidth(infoSize);

        this._listen();
    }

    get commands() { return this._commandExplainerControl; }
    get info() { return this._infoControl; }
    get option() { return this._optionController; }
    get params() { return this._params; }
    get provider() { return this._provider; }
    get viewer() { return this._viewer; }

    initialize() {
        this._move();
        this._loadProvider()
            .then(provider => {
                const items = Object.keys(provider.data.clusters);
                this._optionController.dat.addReconstructionItems(items);
            });

    }

    _configure(config) {
        this._spatial.configure(config);
    }

    _getRandomKey(reconstructions) {
        return Object.keys(reconstructions[0].shots)[0];
    }

    _listen() {
        const optionController = this._optionController;
        optionController.on('camerasize', this._onCameraSize);
        optionController.on('camerasvisible', this._onCamerasVisible);
        optionController.on(
            'cameravisualizationmode', this._onCameraVisualizationMode);
        optionController.on(
            'reconstructionsselected', this._onReconstructionsSelected);
        optionController.on('commandsvisible', this._onCommandsVisible);
        optionController.on('dattoggle', this._onDatToggle);
        optionController.on('earthcontrols', this._onEarthControls);
        optionController.on('imagesvisible', this._onImagesVisible);
        optionController.on('infosize', this._onInfoSize);
        optionController.on(
            'originalpositionmode', this._onOriginalPositionMode);
        optionController.on('pointsize', this._onPointSize);
        optionController.on('pointsvisible', this._onPointsVisible);
        optionController.on('thumbnailvisible', this._onThumbnailVisible);
        optionController.on('tilesvisible', this._onTilesVisible);

        const Viewer = Mapillary.Viewer;
        this._viewer.on(Viewer.nodechanged, this._onViewerNode);
    }

    _loadProvider() {
        const provider = this._provider;
        return new Promise((resolve) => {
            if (provider.loaded) {
                resolve(provider);
                return;
            }
            provider.on(
                'loaded',
                event => {
                    resolve(event.target);
                });
        })
    }

    async _move() {
        const params = this._params;
        if (!!params.img) {
            this._moveToKey(this._viewer, params.img);
        } else {
            const loadedProvider = await this._loadProvider();
            this._moveToKey(
                this._viewer,
                this._getRandomKey(loadedProvider.reconstructions));
        }
    }

    _moveToKey(viewer, key) {
        viewer
            .moveToKey(key)
            .catch(error => console.error(error));
    }

    _onCameraSize = event =>
        this._configure({ cameraSize: event.size });

    _onCamerasVisible = event =>
        this._configure({ camerasVisible: event.visible });

    _onCameraVisualizationMode = event =>
        this._configure({ cameraVisualizationMode: event.mode });

    _onReconstructionsSelected = event => {
        const filter = ['in', 'clusterKey', ...event.active];
        this._viewer.setFilter(filter);
    };

    _onCommandsVisible = event => {
        if (event.visible) { this._commandExplainerControl.show(); }
        else { this._commandExplainerControl.hide(); }
    };

    _onEarthControls = event => {
        const active = event.active;
        const direction = 'direction';
        this._configure({ earthControls: active });
        if (active) { this._viewer.deactivateComponent(direction); }
        else { this._viewer.activateComponent(direction); }
    };

    _onDatToggle = () => {
        const dat = this._optionController.dat;
        if (dat.gui.closed) {
            dat.gui.open();
        } else {
            dat.gui.close();
        }
    }

    _onImagesVisible = event => {
        if (event.visible) { this._viewer.activateComponent('imagePlane'); }
        else { this._viewer.deactivateComponent('imagePlane'); }
    };

    _onInfoSize = event =>
        this._infoControl.setWidth(event.size);

    _onOriginalPositionMode = event =>
        this._configure({ originalPositionMode: event.mode });

    _onPointSize = event =>
        this._configure({ pointSize: event.size });

    _onPointsVisible = event =>
        this._configure({ pointsVisible: event.visible });

    _onTilesVisible = event =>
        this._configure({ tilesVisible: event.visible });

    _onThumbnailVisible = event => {
        if (event.visible) { this._thumbnailControl.show(); }
        else { this._thumbnailControl.hide(); }
    };

    _onViewerNode = node =>
        this._thumbnailControl.update(node);
}

function parseHash(hash) {
    if (!hash) { return {}; }
    const hashContent = hash.substring(1);
    const pl = /\+/g,  // Regex for replacing addition symbol with a space
        search = /([^&=]+)=?([^&]*)/g,
        decode = s => decodeURIComponent(s.replace(pl, ' ')),
        params = {};

    let match;
    while (match = search.exec(hashContent)) {
        params[decode(match[1])] = decode(match[2]);
    }
    for (const param of Object.keys(params)) {
        const split = params[param].split(',');
        params[param] = split.length > 1 ? split : params[param];
    }
    return params;
}

function createProviderOptions(params) {
    const location = window.location;
    const options = {
        endpoint: `${location.protocol}//${location.host}`,
    };
    if (!!params.file) {
        if (params.file instanceof Array) {
            options.reconstructionPaths = params.file;
        } else {
            options.reconstructionPaths = [params.file];
            // fallback images based on file
            options.imagesPath =
                `${params.file.replace(/[^/]*$/, '')}${'images'}`;
        }

    }
    if (!!params.images) {
        // images takes precedence over fallback images
        options.imagesPath = params.images;
    }
    return options;
}
