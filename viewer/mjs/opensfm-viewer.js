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
        this._infoControl = null;
        this._optionController = null;
        this._provider = options.provider;
        this._params = options.params;
        this._viewer = null;
    }

    get info() { return this._infoControl; }
    get option() { return this._optionController; }
    get params() { return this._params; }
    get provider() { return this._provider; }
    get viewer() { return this._viewer; }

    async initialize() {
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

        const document = window.document;
        const container = document.createElement('div');
        container.classList.add('opensfm-viewer');
        document.body.appendChild(container);

        const imagesVisible = false;
        const provider = this._provider;
        this._viewer = new Mapillary.Viewer({
            apiClient: provider,
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
        window.addEventListener('resize', () => viewer.resize());

        const viewer = this._viewer;
        const infoSize = 0.3;
        const thumbnailVisible = false;
        this._thumbnailControl = new ThumbnailControl({
            thumbnailVisible,
            viewer,
        })
        this._thumbnailControl.listen();
        this._thumbnailControl.setWidth(infoSize);
        this._infoControl = new InfoControl({
            beforeContainer: viewer.getContainer(),
        });
        this._infoControl.addControl(this._thumbnailControl);

        const options = {
            imagesVisible,
            infoSize,
            provider,
            spatialConfiguration,
            thumbnailVisible,
            viewer
        };
        this._optionController = new OptionController(options);
        this._optionController.on(
            'thumbnailvisiblechanged',
            event => {
                if (event.visible) { this._thumbnailControl.show(); }
                else { this._thumbnailControl.hide(); }
                this._thumbnailControl.update();
            });
        this._optionController.on(
            'infosizechanged',
            event => this._thumbnailControl.setWidth(event.width));

        const target = this;
        const event =
            new CustomEvent('mapillarycreated', { detail: { target } });
        document.dispatchEvent(event);

        this._loadProvider()
            .then(provider => this._addReconstructionItems(provider));
        this._move();
    }

    _addReconstructionItems(provider) {
        const items = Object.keys(provider.data.clusters);
        this._optionController.dat.addReconstructionItems(items);
    }

    _getRandomKey(reconstructions) {
        return Object.keys(reconstructions[0].shots)[0];
    }

    async _move() {
        if (!!this._params.img) {
            this._moveToKey(this._viewer, this._params.img);
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
