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

class OpenSfmViewer {
    constructor(params, provider) {
        this._params = params;
        this._optionHandler = null;
        this._provider = provider;
        this._viewer = null;
    }

    get optionHandler() { return this._optionHandler; }
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
            tilesVisible: true,
            cameraVisualizationMode: cvm,
        };

        const document = window.document;
        const container = document.createElement('div');
        container.classList.add('opensfm-viewer');
        document.body.appendChild(container);

        const provider = this._provider;
        const viewer = new Mapillary.Viewer({
            apiClient: provider,
            combinedPanning: false,
            component: {
                cover: false,
                direction: false,
                imagePlane: false,
                sequence: false,
                spatialData: spatialConfiguration,
            },
            container,
            renderMode: Mapillary.RenderMode.Letterbox,
        });

        window.addEventListener('resize', () => viewer.resize());

        const handlerOptions = { provider, spatialConfiguration, viewer };
        this._optionHandler = new OptionChangeHandler(handlerOptions);
        this._viewer = viewer;
        const target = this;
        const event =
            new CustomEvent('mapillarycreated', { detail: { target } });
        document.dispatchEvent(event);

        this._addReconstructionItems();
        if (!!this._params.img) {
            this._moveToKey(viewer, this._params.img);
        } else {
            const loadedProvider = await this._loadProvider();
            this._moveToKey(
                viewer,
                this._getRandomKey(loadedProvider.reconstructions));
        }
    }

    async _addReconstructionItems() {
        const loadedProvider = await this._loadProvider();
        const items = Object.keys(loadedProvider.data.clusters);
        this._optionHandler.guiHelper.addReconstructionItems(items);
    }

    _getRandomKey(reconstructions) {
        return Object.keys(reconstructions[0].shots)[0];
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
                const file = await loader.getFile();
                await provider.addFile(file);
            } catch (error) {
                console.error(error);
            }
        }
        loader.hide();
    }

    const openSfmViewer = new OpenSfmViewer(params, provider);
    openSfmViewer.initialize();
});
