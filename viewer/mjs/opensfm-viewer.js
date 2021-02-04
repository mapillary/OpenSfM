class ThumbnailHelper {
    constructor(options) {
        this._viewer = options.viewer;
        this._config = options.config;
        this._infoContainer =
            window.document.getElementById('info-container');

        this._node = null;
    }

    get infoContainer() { return this._infoContainer; }

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
        this._infoContainer.firstElementChild.src = node.image.src;
        const infoText = `${node.clusterKey}::${node.sequenceKey}::${node.key}`;
        this._infoContainer.lastElementChild.textContent = infoText;
    }
}

class DatGuiHelper {
    constructor(options) {
        this._provider = options.provider;
        this._viewer = options.viewer;
        this._spatial = options.viewer.getComponent('spatialData');
        this._thumbnailHelper = new ThumbnailHelper(options);

        this._config = options.config;
        this._modeConfig = options.modeConfig;
        this._recConfig = { reconstructions: {}, toggle: null };
        this._gui = null;
    }

    createGui() {
        this._thumbnailHelper.listen();

        const gui = new dat.GUI();
        gui.width = 300;
        const optionsFolder = gui.addFolder('Options');
        const reconstructionsFolder = gui.addFolder('Reconstructions');

        this._setThumbnailSize(this._config.thumbnailSize);

        this._addNumericOption('pointSize', optionsFolder);
        this._addNumericOption('cameraSize', optionsFolder);

        this._addBooleanOption('camerasVisible', optionsFolder);
        this._addBooleanOption('earthControls', optionsFolder);
        this._addBooleanOption('imagesVisible', optionsFolder);
        this._addBooleanOption('pointsVisible', optionsFolder);
        this._addBooleanOption('tilesVisible', optionsFolder);

        this._addCameraVizualizationOption(optionsFolder);
        this._addPositionVisualizationOption(optionsFolder);

        const recConfig = this._recConfig;
        const recs = recConfig.reconstructions;
        this._recConfig.toggle = () => {
            this._filter(recs, toggle);
            this._setToggleText(recs, toggle);
        };
        const toggle = reconstructionsFolder.add(recConfig, 'toggle');
        this._setToggleText(recs, toggle);

        this._addBooleanOption('thumbnailVisible', optionsFolder);
        this._addNumericOption('thumbnailSize', optionsFolder);

        if (this._provider.loaded) {
            this._createReconstructionControllers(
                this._provider.data,
                recs,
                reconstructionsFolder,
                toggle);
        } else {
            this._provider.on(
                'loaded',
                event => {
                    this._createReconstructionControllers(
                        event.target.data,
                        recs,
                        reconstructionsFolder,
                        toggle);
                });
        }

        optionsFolder.open();
        reconstructionsFolder.open();
        gui.close();

        this._gui = gui;
    }

    _addBooleanOption(name, folder) {
        folder
            .add(this._config, name)
            .listen()
            .onChange(v => { this._onChange(name, v); });
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
            .onChange(m => { this._onChange('cameraVisualizationMode', cvm[m]); });
    }

    _addNumericOption(name, folder) {
        folder
            .add(this._config, name, 0, 1)
            .listen()
            .onChange(v => { this._onChange(name, v); });
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
            .onChange(m => { this._onChange('originalPositionMode', opm[m]); });
    }

    _checkAllEnabled(recs) {
        const keys = Object.keys(recs);
        return keys.length &&
            keys
                .map(k => recs[k])
                .reduce((acc, val) => acc && val, true);
    }

    _configure(name) {
        const c = {};
        c[name] = this._config[name];
        this._spatial.configure(c);
    }

    _createReconstructionControllers(data, recs, folder, toggle) {
        for (const key in data.clusters) {
            if (!data.clusters.hasOwnProperty(key)) { continue; }
            recs[key] = true;
            folder
                .add(recs, key)
                .listen()
                .onChange(() => {
                    this._setToggleText(recs, toggle);
                    const filter = this._createFilter(recs);
                    this._viewer.setFilter(filter);
                });
        }
        this._setToggleText(recs, toggle);
    }

    _createFilter(recs) {
        const all = this._checkAllEnabled(recs);
        if (all) { return []; }

        const enabled = Object.keys(recs)
            .filter(k => recs[k]);
        return ['in', 'clusterKey', ...enabled];
    }

    _filter(recs) {
        const all = this._checkAllEnabled(recs);
        for (const key of Object.keys(recs)) {
            recs[key] = !all;
        }

        const filter = this._createFilter(recs);
        this._viewer.setFilter(filter);
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
                this._setThumbnail(value);
                this._thumbnailHelper.change();
                break;
            case 'thumbnailSize':
                this._setThumbnailSize(value);
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

    _setImagesComponent() {
        if (this._config.imagesVisible) {
            this._viewer.activateComponent('imagePlane');
        }
        else { this._viewer.deactivateComponent('imagePlane'); }
    }

    _setThumbnail(value) {
        const infoContainer = this._thumbnailHelper.infoContainer;
        if (value) { infoContainer.classList.remove('hidden'); }
        else { infoContainer.classList.add('hidden'); }
    }

    _setThumbnailSize(value) {
        this._thumbnailHelper.infoContainer.style.width = `${100 * value}%`;
    }

    _setToggleText(recs, toggle) {
        const all = this._checkAllEnabled(recs);
        if (all) { toggle.name('Hide all'); }
        else { toggle.name('Show all'); }
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

class FileHandler {
    constructor() {
        this._pickFile = document.getElementById('pick-file');
        this._dropFile = document.getElementById('drop-file');
        this._preventDefault = event => event.preventDefault();
    }

    getFile() {
        const document = window.document;

        const preventDefault = this._preventDefault;
        document.addEventListener('dragover', preventDefault);
        document.addEventListener('drop', preventDefault);

        const pickFile = this._pickFile;
        pickFile.classList.remove('hidden');

        const dropFile = this._dropFile;
        dropFile.classList.remove('hidden');
        dropFile.addEventListener('dragenter', event => {
            dropFile.classList.add('file-drop-hover');
            event.preventDefault();
        });
        dropFile.addEventListener('dragleave', event => {
            dropFile.classList.remove('file-drop-hover');
            event.preventDefault();
        });
        dropFile.addEventListener('dragover', event => {
            event.dataTransfer.dropEffect = 'copy';
            event.preventDefault();
        });

        return new Promise((resolve, reject) => {
            pickFile.addEventListener(
                'change',
                event => {
                    const file = event.target.files[0];
                    this._remove();
                    resolve(file);
                });
            dropFile.addEventListener('drop', event => {
                dropFile.classList.remove('file-drop-hover');
                event.preventDefault();
                const items = event.dataTransfer.items;

                if (!items) { return; }

                for (const item of items) {
                    if (!this._verifyKind(item.kind)) { continue; }
                    const file = item.getAsFile();
                    this._remove();
                    resolve(file);
                    break;
                }
            });
        });
    }

    _verifyKind(kind) {
        if (kind !== 'file') {
            console.warn(`Unrecognized format: ${kind}`);
            return false;
        }
        return true;
    }

    _remove() {
        this._pickFile.classList.add('hidden');
        this._dropFile.classList.add('hidden');

        const preventDefault = this._preventDefault;
        document.removeEventListener('dragover', preventDefault);
        document.removeEventListener('drop', preventDefault);
    }
}

function bindOptions(provider, viewer, initialConfig) {
    const spatial = viewer.getComponent('spatialData');

    const config = Object.assign(
        {},
        spatial.defaultConfiguration,
        {
            imagesVisible: false,
            thumbnailVisible: false,
            thumbnailSize: 0.2,
        },
        initialConfig);

    const cvm = Mapillary.SpatialDataComponent.CameraVisualizationMode;
    const opm = Mapillary.SpatialDataComponent.OriginalPositionMode;
    const modeConfig = {
        'cameraVisualizationMode': cvm[config.cameraVisualizationMode],
        'originalPositionMode': opm[config.originalPositionMode],
    };

    const options = { provider, viewer, config, modeConfig };
    const keyHandler = new KeyHandler(options);
    const datGuiHelper = new DatGuiHelper(options);

    keyHandler.bindKeys();
    datGuiHelper.createGui();
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

    return params;
}

function createProviderOptions(params) {
    const location = window.location;
    const options = {
        endpoint: `${location.protocol}//${location.host}`,
    };
    if (!!params.file) {
        options.reconstructionPath = params.file;
        options.imagesPath = !!params.images ?
            params.images :
            `${params.file.replace(/[^/]*$/, '')}${'images'}`;
    } else if (!!params.images) {
        options.imagesPath = params.images;
    }
    return options;
}

function initializeViewer(provider, params) {
    const spc = Mapillary.SpatialDataComponent;
    const cvm = spc.CameraVisualizationMode.Default;
    const opm = spc.OriginalPositionMode.Hidden;
    const spatialDataConfiguration = {
        cameraSize: 0.5,
        camerasVisible: true,
        earthControls: true,
        pointSize: 0.2,
        originalPositionMode: opm,
        pointsVisible: true,
        tilesVisible: true,
        cameraVisualizationMode: cvm,
    };

    const viewer = new Mapillary.Viewer({
        apiClient: provider,
        combinedPanning: false,
        component: {
            cover: false,
            direction: false,
            imagePlane: false,
            sequence: false,
            spatialData: spatialDataConfiguration,
        },
        container: 'mly',
        renderMode: Mapillary.RenderMode.Letterbox,
    });

    window.addEventListener('resize', () => viewer.resize());

    bindOptions(provider, viewer, spatialDataConfiguration);

    function getRandomKey(reconstructions) {
        return Object.keys(reconstructions[0].shots)[0];
    }

    function moveToKey(viewer, key) {
        viewer
            .moveToKey(key)
            .catch(error => console.error(error));
    }

    if (!!params.img) {
        moveToKey(viewer, params.img);
    } else {
        if (provider.loaded) {
            moveToKey(viewer, getRandomKey(provider.reconstructions));
        } else {
            provider.on(
                'loaded',
                event => {
                    moveToKey(
                        viewer,
                        getRandomKey(event.target.reconstructions));
                });
        }
    }
}

window.document.addEventListener('DOMContentLoaded', async () => {
    const params = parseHash(window.location.hash);
    const providerOptions = createProviderOptions(params);
    const provider = new OpenSfmDataProvider(providerOptions);

    if (!params.file) {
        const fileHandler = new FileHandler();
        while (!provider.loaded) {
            const file = await fileHandler.getFile();
            try {
                await provider.addFile(file);
            } catch (error) {
                console.error(error);
                console.log("File could not be loaded, please try another reconstruction file.");
            }
        }
    }

    initializeViewer(provider, params);
});
