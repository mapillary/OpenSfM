/**
 * @format
 */

import {
  fetchArrayBuffer,
  DataProviderBase,
  S2GeometryProvider,
} from '../../node_modules/mapillary-js/dist/mapillary.module.js';
import {isReconstructionData} from '../util/types.js';
import {DataConverter} from './DataConverter.js';

function* generator(items, map) {
  for (var item of items) {
    yield map(item);
  }
}

export class OpensfmDataProvider extends DataProviderBase {
  constructor(options, geometry) {
    super(geometry ?? new S2GeometryProvider(16));
    this._convert = new DataConverter(options);
    this._inventedImageBuffer = null;
    this._rawData = {};
    this._data = {
      cells: {},
      clusters: {},
      meshes: {},
      images: {},
      sequences: {},
    };

    const endpoint = options.endpoint;
    this._inventedImagePromise = !options.imagePath
      ? this._inventImagePromise().then(buffer => {
          this._inventedImageBuffer = buffer;
          this._inventedImagePromise = null;
          return buffer;
        })
      : null;

    const recs = options.reconstructionPaths;
    const urls = !!recs ? recs.map(path => new URL(path, endpoint).href) : [];

    if (urls.length > 0) {
      this._fetchReconstructions(urls);
    }
  }

  get data() {
    return this._data;
  }

  get loaded() {
    return !this._load && Object.keys(this._rawData).length > 0;
  }

  get loading() {
    return !!this._load;
  }

  get rawData() {
    return this._rawData;
  }

  add(file) {
    if (!this.loaded) {
      throw new Error('Not loaded');
    }
    return new Promise((resolve, reject) => {
      try {
        const created = this._addFile(file);

        // MJS data provider API data create event
        // (required for internal MJS cell updates).
        const type = 'datacreate';
        const event = {
          cellIds: Object.keys(created.data.cells),
          target: this,
          type,
        };
        this.fire(type, event);

        // OpenSfM specific event for simplified
        // cluster data handling.
        const opensfmType = 'opensfmdatacreate';
        const opensfmEvent = {
          data: created.data,
          rawData: created.rawData,
          target: this,
          type: opensfmType,
        };
        this.fire(opensfmType, opensfmEvent);
        resolve();
      } catch (error) {
        reject(error);
      }
    });
  }

  load(file, inventImages) {
    return new Promise((resolve, reject) => {
      if (this.loading) {
        reject(new Error('Already loading'));
        return;
      }

      try {
        this._initialize(file, inventImages);
        resolve();
      } catch (error) {
        reject(error);
      }
    });
  }

  getCoreImages(cellId) {
    return this._getDataLoadedPromise().then(() => {
      const cell = {};
      cell[cellId] = cellId in this._data.cells ? this._data.cells[cellId] : {};
      const images = this._convert.coreImageContract(cellId, cell[cellId]);
      return images;
    });
  }

  getCluster(clusterId) {
    return this._getDataLoadedPromise().then(
      () => this._data.clusters[clusterId],
    );
  }

  getSpatialImages(imageIds) {
    return this._getDataLoadedPromise()
      .then(() => this._filterItems(imageIds, this._data.images))
      .then(images => this._convert.entContracts(images));
  }

  getImages(imageIds) {
    return this._getDataLoadedPromise()
      .then(() => this._filterItems(imageIds, this._data.images))
      .then(images => this._convert.entContracts(images));
  }

  getImageBuffer(url, cancellation) {
    if (this._isInventingImages()) {
      return this._getInventedImageBuffer();
    }

    return fetchArrayBuffer(url, cancellation).catch(() =>
      this._inventImagePromise(),
    );
  }

  getMesh(url) {
    return this._getDataLoadedPromise().then(() => this._data.meshes[url]);
  }

  getSequence(sequenceId) {
    return this._getDataLoadedPromise().then(
      () => this._data.sequences[sequenceId],
    );
  }

  setUserToken() {
    /*noop*/
  }

  _addFile(file) {
    const cells = {};
    const clusters = {};
    const meshes = {};
    const images = {};
    const sequences = {};
    const rawData = {};
    let clusterIndex = Object.keys(this._data.clusters).length;
    for (const cluster of file.data) {
      const clusterId = this._inventClusterId(clusterIndex++);
      rawData[clusterId] = {
        cluster,
        id: clusterId,
        file: {
          url: file.url,
          children: file.children,
          name: file.name,
        },
      };
      const reference = this._convert.reference(cluster.reference_lla);
      clusters[clusterId] = this._convert.cluster(
        cluster,
        clusterId,
        reference,
      );

      const shots = cluster.shots;
      const cameras = cluster.cameras;
      for (const shotId in shots) {
        if (!shots.hasOwnProperty(shotId)) {
          continue;
        }

        const shot = shots[shotId];
        const camera = cameras[shot.camera];
        const image = this._convert.image(
          shotId,
          clusterId,
          shot,
          camera,
          reference,
        );
        const uniqueId = image.id;
        if (this._hasImage(uniqueId)) {
          throw new Error(`Image ${uniqueId} already exists`);
        }
        images[uniqueId] = image;

        const sequenceId = uniqueId;
        if (!(sequenceId in sequences)) {
          sequences[sequenceId] = this._convert.sequence(sequenceId);
        }

        meshes[uniqueId] = this._convert.mesh(shot);

        const cellId = this._geometry.lngLatToCellId(image.geometry);
        if (!(cellId in cells)) {
          cells[cellId] = {};
        }
        cells[cellId][uniqueId] = image;
      }
    }

    const data = {cells, clusters, images, meshes, sequences};
    this._appendData(data);
    Object.assign(this._rawData, rawData);

    return {data, rawData};
  }

  _appendData(data) {
    const tData = this._data;
    for (const cellId of Object.keys(data.cells)) {
      const cell = data.cells[cellId];
      if (!(cellId in tData.cells)) {
        tData.cells[cellId] = cell;
      } else {
        Object.assign(tData.cells[cellId], cell);
      }
    }
    Object.assign(tData.clusters, data.clusters);
    Object.assign(tData.images, data.images);
    Object.assign(tData.meshes, data.meshes);
    Object.assign(tData.sequences, data.sequences);
  }

  _fetchReconstruction(url) {
    return fetch(url, {
      method: 'GET',
      headers: {
        Accept: 'application/json',
        'Content-Type': 'application/json',
      },
    }).then(r => {
      if (r.ok) {
        return r.json();
      } else {
        throw new Error('Failed to fetch reconstruction');
      }
    });
  }

  _fetchReconstructions(urls) {
    const fetchDataGenerator = generator(urls, url => {
      return {
        url,
        fetch: this._fetchReconstruction(url),
      };
    });

    this._loadData(fetchDataGenerator);
  }

  _filterItems(ids, items) {
    const filtered = {};
    for (const id of ids) {
      filtered[id] = items[id];
    }
    return filtered;
  }

  _getDataLoadedPromise() {
    return this._load != null ? this._load : Promise.resolve();
  }

  _getInventedImageBuffer() {
    if (this._inventedImageBuffer) {
      return Promise.resolve(this._inventedImageBuffer);
    }
    return this._inventedImagePromise;
  }

  _hasImage(imageId) {
    return imageId in this._data.images;
  }

  _initialize(file, inventImages) {
    if (this.loaded) {
      reject(new Error('Already loaded'));
      return;
    }

    if (!isReconstructionData(file.data)) {
      reject(new Error('Not a reconstruction'));
      return;
    }

    this._addFile(file);
    if (inventImages) {
      this._inventedImagePromise = this._inventImagePromise().then(buffer => {
        this._inventedImageBuffer = buffer;
        this._inventedImagePromise = null;
        return buffer;
      });
    }
    this.fire('load', {target: this});
  }

  _inventClusterId(index) {
    return `cluster_${index}`;
  }

  async _inventImagePromise() {
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 1;

    return new Promise(resolve => {
      canvas.toBlob(async blob => {
        const buffer = await blob.arrayBuffer();
        resolve(buffer);
      }, 'image/jpeg');
    });
  }

  _isInventingImages() {
    return !!this._inventedImageBuffer || !!this._inventedImagePromise;
  }

  _loadData(iterator) {
    this._load = new Promise((resolve, reject) => {
      const loaders = [];
      for (const item of iterator) {
        loaders.push(
          item.fetch
            .then(async data => {
              const file = {
                children: [],
                data,
                name: item.url,
                url: item.url,
              };

              if (this.loaded) {
                return this.add(file);
              }

              await this._initialize(file);
              this._load = null;
              resolve();
              return Promise.resolve();
            })
            .catch(error => {
              console.error(error);
              return Promise.reject(error);
            }),
        );
      }

      Promise.allSettled(loaders).then(responses => {
        let succeeded = false;
        for (const response of responses) {
          if (response.status === 'fulfilled') {
            succeeded = true;
            break;
          }
        }

        if (!succeeded) {
          this._load = null;
          reject(new Error('All file loads failed.'));
        }
      });
    });

    return this._load;
  }
}
