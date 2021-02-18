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
        const infoSize = 0.3;
        const thumbnailVisible = false;

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

        this._thumbnailControl = new ThumbnailControl({ thumbnailVisible })
        this._thumbnailControl.setWidth(infoSize);

        this._infoControl = new InfoControl({
            beforeContainer: viewer.getContainer(),
        });
        this._infoControl.addControl(this._thumbnailControl);

        this._optionController = new OptionController(
            Object.assign(
                {},
                viewer.getComponent('spatialData').defaultConfiguration,
                {
                    imagesVisible,
                    infoSize,
                    thumbnailVisible,
                },
                spatialConfiguration));

        this._listen();
    }

    get info() { return this._infoControl; }
    get option() { return this._optionController; }
    get params() { return this._params; }
    get provider() { return this._provider; }
    get viewer() { return this._viewer; }

    initialize() {
        const target = this;
        const event =
            new CustomEvent('mapillarycreated', { detail: { target } });
        document.dispatchEvent(event);

        this._loadProvider()
            .then(provider => {
                const items = Object.keys(provider.data.clusters);
                this._optionController.dat.addReconstructionItems(items);
            });

        this._move();
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
        optionController.on('clusters', this._onClusters);
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

    _onClusters = event => {
        const filter = ['in', 'clusterKey', ...event.active];
        this._viewer.setFilter(filter);
    };

    _onEarthControls = event => {
        const active = event.active;
        const direction = 'direction';
        this._configure({ earthControls: active });
        if (active) { this._viewer.deactivateComponent(direction); }
        else { this._viewer.activateComponent(direction); }
    };

    _onImagesVisible = event => {
        if (event.visible) { this._viewer.activateComponent('imagePlane'); }
        else { this._viewer.deactivateComponent('imagePlane'); }
    };

    _onInfoSize = event =>
        this._thumbnailControl.setWidth(event.size);

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

window.document.addEventListener('DOMContentLoaded', async () => {
    const params = parseHash(window.location.hash);
    const providerOptions = createProviderOptions(params);
    const provider = new OpenSfmDataProvider(providerOptions);

    if (!params.file) {
        const loader = new FileLoader(['opensfm-file-container']);
        loader.show();
        while (!provider.loaded) {
            try {
                const files = await loader.getFiles();
                await provider.addFiles(files);
            } catch (error) {
                console.error(error);
            }
        }
        loader.hide();
    }

    const openSfmViewer = new OpenSfmViewer({ params, provider });
    openSfmViewer.initialize();
});
