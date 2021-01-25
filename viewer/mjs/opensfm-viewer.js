function setImagesComponent(config, viewer) {
    if (config.imagesVisible) { viewer.activateComponent('imagePlane'); }
    else { viewer.deactivateComponent('imagePlane'); }
}

function setDirectionComponent(config, viewer) {
    if (config.earthControls) { viewer.deactivateComponent('direction'); }
    else { viewer.activateComponent('direction'); }
}

function configure(name, config, spatial) {
    const c = {}; c[name] = config[name];
    spatial.configure(c);
}

function bindDatGui(config, modeConfig, spatial, provider, viewer) {
    const gui = new dat.GUI();
    gui.width = 300;
    const optionsFolder = gui.addFolder('Options');
    const reconstructionsFolder = gui.addFolder('Reconstructions');

    function setValue(name, size) {
        config[name] = size;
        configure(name, config, spatial);
    }

    const infoContainer =
        window.document.getElementById('info-container');

    viewer.on(Mapillary.Viewer.nodechanged, n => {
        infoContainer.firstElementChild.src = n.image.src;
        const infoText = `${n.clusterKey}::${n.sequenceKey}::${n.key}`;
        infoContainer.lastElementChild.textContent = infoText;
    });

    function setThumbnail(value) {
        if (value) {
            infoContainer.classList.remove('hidden');
        } else {
            infoContainer.classList.add('hidden');
        }
    }

    function setThumbnailSize(value) {
        infoContainer.style.width = `${100 * value}%`;
    }

    setThumbnailSize(config.thumbnailSize);

    function onChange(name, value) {
        switch (name) {
            case 'camerasVisible':
            case 'pointsVisible':
            case 'tilesVisible':
                configure(name, config, spatial);
                break;
            case 'earthControls':
                setDirectionComponent(config, viewer);
                configure(name, config, spatial);
                break;
            case 'imagesVisible':
                setImagesComponent(config, viewer);
                break;
            case 'originalPositionMode':
            case 'cameraVisualizationMode':
            case 'cameraSize':
            case 'pointSize':
                setValue(name, value, config, spatial);
                break;
            case 'thumbnailVisible':
                setThumbnail(value);
                break;
            case 'thumbnailSize':
                setThumbnailSize(value);
            default:
                break;
        }
    }

    function addNumericOption(name, options, folder) {
        folder
            .add(options, name, 0, 1)
            .listen()
            .onChange(v => { onChange(name, v); });
    }

    function addBooleanOption(name, options, folder) {
        folder
            .add(options, name)
            .listen()
            .onChange(v => { onChange(name, v); });
    }

    addNumericOption('pointSize', config, optionsFolder);
    addNumericOption('cameraSize', config, optionsFolder);
    addBooleanOption('camerasVisible', config, optionsFolder);
    addBooleanOption('earthControls', config, optionsFolder);
    addBooleanOption('imagesVisible', config, optionsFolder);
    addBooleanOption('pointsVisible', config, optionsFolder);
    addBooleanOption('tilesVisible', config, optionsFolder);

    const cvm = Mapillary.SpatialDataComponent.CameraVisualizationMode;
    const cvms = [
        cvm[cvm.Default],
        cvm[cvm.Cluster],
        cvm[cvm.ConnectedComponent],
        cvm[cvm.Sequence],
    ];
    optionsFolder
        .add(modeConfig, 'cameraVisualizationMode', cvms)
        .listen()
        .onChange(m => { onChange('cameraVisualizationMode', cvm[m]); });

    const opm = Mapillary.SpatialDataComponent.OriginalPositionMode;
    const opms = [
        opm[opm.Hidden],
        opm[opm.Flat],
        opm[opm.Altitude],
    ];
    optionsFolder
        .add(modeConfig, 'originalPositionMode', opms)
        .listen()
        .onChange(m => { onChange('originalPositionMode', opm[m]); });

    const recConfig = {
        toggle: () => {
            const recs = recConfig.reconstructions;
            const all = checkAllEnabled(recs);
            for (const key of Object.keys(recs)) {
                recs[key] = !all;
            }
            setToggleText();
            const filter = createFilter(recs);
            viewer.setFilter(filter);
        },
        reconstructions: {},
    }

    function checkAllEnabled(recs) {
        const keys = Object.keys(recs);
        return keys.length &&
            keys
                .map(k => recConfig.reconstructions[k])
                .reduce((acc, val) => acc && val, true);
    }

    function createFilter(recs) {
        const all = checkAllEnabled(recs);
        if (all) { return []; }

        const enabled = Object.keys(recs)
            .filter(k => recConfig.reconstructions[k]);
        return ['in', 'clusterKey', ...enabled];
    }

    function setToggleText() {
        const all = checkAllEnabled(recConfig.reconstructions);
        if (all) { toggle.name('Hide all'); }
        else { toggle.name('Show all'); }
    }

    const toggle = reconstructionsFolder.add(recConfig, 'toggle');
    setToggleText();

    function createReconstructionControllers(data) {
        const recs = recConfig.reconstructions;
        for (const key in data.clusters) {
            if (!data.clusters.hasOwnProperty(key)) { continue; }
            recs[key] = true;
            reconstructionsFolder
                .add(recs, key)
                .listen()
                .onChange(() => {
                    setToggleText();
                    const filter = createFilter(recs);
                    viewer.setFilter(filter);
                });
        }
        setToggleText();
    }

    addBooleanOption('thumbnailVisible', config, optionsFolder);
    addNumericOption('thumbnailSize', config, optionsFolder);

    if (provider.loaded) {
        createReconstructionControllers(provider.data);
    } else {
        provider.on(
            'loaded',
            event => { createReconstructionControllers(event.target.data); });
    }

    optionsFolder.open();
    reconstructionsFolder.open();
    gui.close()
}

function bindKeys(config, modeConfig, spatial, viewer) {
    function changeSize(name, coeff) {
        config[name] *= coeff;
        config[name] = Math.max(0.01, Math.min(1, config[name]));
        configure(name, config, spatial);
    }

    function rotateCvm() {
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

        config.cameraVisualizationMode =
            modeRotation[config.cameraVisualizationMode];
        configure('cameraVisualizationMode', config, spatial);
        modeConfig.cameraVisualizationMode =
            mode[config.cameraVisualizationMode];
    }

    function rotateOpm() {
        const mode = Mapillary.SpatialDataComponent.OriginalPositionMode;

        const hidden = mode.Hidden;
        const flat = mode.Flat;
        const altitude = mode.Altitude;

        const modeRotation = {};
        modeRotation[hidden] = flat;
        modeRotation[flat] = altitude;
        modeRotation[altitude] = hidden;

        config.originalPositionMode =
            modeRotation[config.originalPositionMode];
        configure('originalPositionMode', config, spatial);
        modeConfig.originalPositionMode =
            mode[config.originalPositionMode];
    }

    function toggleBooleanSetting(name) {
        config[name] = !config[name];
        configure(name, config, spatial);
    }

    function toggleImages() {
        config.imagesVisible = !config.imagesVisible;
        setImagesComponent(config, viewer)
    }

    window.document.addEventListener(
        'keydown',
        e => {
            let name = undefined;
            switch (e.key) {
                case 'c': name = 'camerasVisible'; break;
                case 'p': name = 'pointsVisible'; break;
                case 't': name = 'tilesVisible'; break;
                case 'i': toggleImages(); break;
                case 'v': rotateCvm(); break;
                case 'o': rotateOpm(); break;
                case 'q': changeSize('pointSize', 0.9); break;
                case 'w': changeSize('pointSize', 1.1); break;
                case 'a': changeSize('cameraSize', 0.9); break;
                case 's': changeSize('cameraSize', 1.1); break;
                case 'e':
                    toggleBooleanSetting('earthControls');
                    setDirectionComponent(config, viewer);
                    break;
                default: break;
            }

            if (!!name) { toggleBooleanSetting(name); }
        });
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

    bindKeys(config, modeConfig, spatial, viewer);
    bindDatGui(config, modeConfig, spatial, provider, viewer);
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
        pointSize: 0.3,
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

function getFile(provider, params) {
    const preventDefault = event => event.preventDefault();
    window.document.addEventListener('dragover', preventDefault);
    window.document.addEventListener('drop', preventDefault);

    const pickFile = window.document
        .getElementById('pick-file');
    pickFile.classList.remove('hidden');

    const dropFile = window.document
        .getElementById('drop-file');
    dropFile.classList.remove('hidden');

    function setupViewer(file) {
        try {
            provider.addFile(file);
        } catch (error) {
            console.error(error);
            return;
        }
        initializeViewer(provider, params);
        pickFile.classList.add('hidden');
        dropFile.classList.add('hidden');
        window.document.removeEventListener('dragover', preventDefault);
        window.document.removeEventListener('drop', preventDefault);
    }

    pickFile.addEventListener(
        'change',
        event => {
            const file = event.target.files[0];
            setupViewer(file);
        });

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
    dropFile.addEventListener('drop', async event => {
        dropFile.classList.remove('file-drop-hover');
        event.preventDefault();
        const items = event.dataTransfer.items;

        if (!items) { return; }

        for (const item of items) {
            if (item.kind !== 'file') {
                console.warn(`Unrecognized format: ${item.kind}`);
                continue;
            }

            const file = item.getAsFile();
            setupViewer(file);
            break;
        }
    });
}

window.document.addEventListener('DOMContentLoaded', () => {
    const params = parseHash(window.location.hash);
    const providerOptions = createProviderOptions(params);
    const provider = new OpenSfmDataProvider(providerOptions);

    if (params.file) {
        initializeViewer(provider, params);
    } else {
        getFile(provider, params);
    }
});
