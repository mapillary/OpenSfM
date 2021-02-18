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
