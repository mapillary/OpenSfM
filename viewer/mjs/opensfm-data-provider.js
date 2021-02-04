function angleAxisToAngle(a) {
    const [x, y, z] = a;
    return Math.sqrt(x * x + y * y + z * z);
}

function createBearing(direction, up) {
    const upProjection = glMatrix.vec3.dot(direction, up);
    const scaledUp = glMatrix.vec3.clone(up);
    glMatrix.vec3.scale(scaledUp, scaledUp, upProjection);
    const planeProjection = glMatrix.vec3.create();
    glMatrix.vec3.sub(planeProjection, direction, scaledUp);
    const phi = Math.atan2(
        planeProjection[1],
        planeProjection[0]);
    const bearing = -phi + Math.PI / 2;
    return wrap(radToDeg(bearing), 0, 360);
}

function createCameraMatrix(rotation, translation) {
    const [rx, ry, rz] = rotation;
    const [tx, ty, tz] = translation;
    const angle = angleAxisToAngle(rotation);
    const axis = glMatrix.vec3.fromValues(rx, ry, rz);
    glMatrix.vec3.scale(axis, axis, 1 / angle);

    const rt = mat4FromAngleAxis(angle, axis);
    rt[12] = tx;
    rt[13] = ty;
    rt[14] = tz;
    return rt;
}

function createOpticalCenter(rotation, translation) {
    const [rx, ry, rz] = rotation;
    const [tx, ty, tz] = translation;

    const angle = angleAxisToAngle(rotation);
    const axis = glMatrix.vec3.fromValues(-rx, -ry, -rz);
    glMatrix.vec3.scale(axis, axis, 1 / angle);

    const t = glMatrix.vec3.fromValues(-tx, -ty, -tz);
    return vec3Rotate(t, angle, axis);
}

function createUpVector(orientation, rt) {
    switch (orientation) {
        case 3:
            return [rt[1], rt[5], rt[9]];
        case 6:
            return [-rt[0], -rt[4], -rt[8]];
        case 8:
            return [rt[0], rt[4], rt[8]];
        case 1:
        default:
            return [-rt[1], -rt[5], -rt[9]];
    }
}

function createViewingDirection(rotation) {
    const [rx, ry, rz] = rotation;
    const angle = angleAxisToAngle(rotation);
    const axis = glMatrix.vec3.fromValues(-rx, -ry, -rz);
    glMatrix.vec3.scale(axis, axis, 1 / angle);

    const vd = glMatrix.vec3.fromValues(0, 0, 1);
    return vec3Rotate(vd, angle, axis);
}

function mat4FromAngleAxis(angle, axis) {
    const m = glMatrix.mat4.create();
    return glMatrix.mat4.fromRotation(m, angle, axis);
}

function mat4Scale(m, s) {
    const scaled = glMatrix.mat4.clone(m);
    glMatrix.mat4.scale(scaled, scaled, [s, s, s]);
    scaled[12] = s * scaled[12];
    scaled[13] = s * scaled[13];
    scaled[14] = s * scaled[14];
    return scaled;
}

function vec3Rotate(v, angle, axis) {
    const m = mat4FromAngleAxis(angle, axis);
    const r = glMatrix.vec3.clone(v);
    glMatrix.vec3.transformMat4(r, r, m);
    return r;
}

function radToDeg(rad) {
    return 180 * rad / Math.PI;
}

function wrap(value, min, max) {
    if (max < min) {
        throw new Error('Invalid arguments: max must be larger than min.');
    }

    const interval = (max - min);
    while (value > max || value < min) {
        if (value > max) {
            value = value - interval;
        } else if (value < min) {
            value = value + interval;
        }
    }
    return value;
}

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

class GeohashGeometryProvider extends Mapillary.API.GeohashGeometryProvider {
    constructor() { super(); this._level = 5; }
}

class OpenSfmDataProvider extends Mapillary.API.DataProviderBase {
    constructor(options, geometry) {
        super(!!geometry ? geometry : new GeohashGeometryProvider());

        this._geoCoords = new Mapillary.Geo.GeoCoords();
        this._eventEmitter = new EventEmitter();

        this._data = null;
        this._reconstructions = null;
        this._fallbackBuffer = null;

        this._options = Object.assign({}, options);

        this._loaderPromise = !!this._options.reconstructionPath ?
            this._fetchReconstruction(
                new URL(
                    this._options.reconstructionPath,
                    this._options.endpoint).href)
                .then(
                    (reconstructions) => {
                        this._createData(reconstructions);
                        this._loaderPromise = null;
                        this._eventEmitter.fire('loaded', { target: this });
                    },
                    error => {
                        this._loaderPromise = null;
                        console.error('Fetching reconstruction failed', error);
                    }) :
            null;

        this._bufferPromise = !this._options.imagesPath ?
            this._createFallbackBuffer() :
            null;
    }

    get data() { return this._data; }
    get loaded() { return !this._loaderPromise && !!this._data; }
    get reconstructions() { return this._reconstructions; }

    addFile(file) {
        return new Promise((resolve, reject) => {
            if (file.type !== 'application/json') {
                reject(new Error(`Unrecognized file type: ${file.type}`));
                return;
            }

            const reader = new FileReader();
            reader.onload = event => {
                try {
                    const reconstructions = JSON.parse(event.target.result);
                    this._createData(reconstructions);
                } catch (error) {
                    this._data = null;
                    this._reconstructions = null;
                    reject(error);
                    return;
                }

                this._eventEmitter.fire('loaded', { target: this });
                resolve();
            };
            reader.onerror = event => { reject(event); };
            reader.readAsText(file);
        });
    }

    getCoreImages(cellId) {
        return this._getDataLoadedPromise()
            .then(() => {
                const nodes = {};
                nodes[cellId] = cellId in this._data.cells ?
                    this._data.cells[cellId] : {};
                return nodes;
            });
    }

    getClusterReconstruction(clusterKey) {
        return this._getDataLoadedPromise()
            .then(() => this._data.clusters[clusterKey]);
    }

    getFillImages(ids) {
        return this._getDataLoadedPromise()
            .then(() => this._filterItems(ids, this._data.nodes));
    }

    getFullImages(ids) {
        return this._getDataLoadedPromise()
            .then(() => this._filterItems(ids, this._data.nodes));
    }

    getImage(url, abort) {
        if (!!this._options.imagesPath) {
            return Mapillary.API.BufferFetcher
                .getArrayBuffer(url, abort)
                .catch(() => { return this._getFallbackBuffer(); });
        }

        return this._getFallbackBuffer();
    }

    getMesh(url) {
        return this._getDataLoadedPromise()
            .then(() => this._data.meshes[url]);
    }

    getSequences(sequenceKeys) {
        return this._getDataLoadedPromise()
            .then(() => this._filterItems(sequenceKeys, this._data.sequences));
    }

    on(type, callback) {
        this._eventEmitter.on(type, callback);
    }

    setToken() { /*noop*/ }

    _createClusterKey(index) { return `cluster_key_${index}`; }

    _createClusterReconstruction(cluster, clusterKey, reference) {
        const converted = {
            cameras: {},
            key: clusterKey,
            points: cluster.points,
            reference_lla: reference,
            shots: {},
        };

        for (const cameraKey in cluster.cameras) {
            if (!(cluster.cameras.hasOwnProperty(cameraKey))) {
                continue;
            }

            const camera = cluster.cameras[cameraKey];
            const projectionType =
                this._convertProjectionType(camera.projection_type);
            converted.cameras[cameraKey] = {
                focal: camera.focal,
                k1: camera.k1,
                k2: camera.k2,
                projection_type: projectionType,
            };
        }

        for (const shotKey in cluster.shots) {
            if (!cluster.shots.hasOwnProperty(shotKey)) {
                continue;
            }

            const shot = cluster.shots[shotKey];
            converted.shots[shotKey] = {
                camera: shot.camera,
                rotation: shot.rotation,
                translation: shot.translation,
            }
        }

        return converted;
    }

    _createData(reconstructions) {
        if (!(reconstructions instanceof Array)) {
            throw new Error('Reconstruction file must be a JSON array.');
        }

        const data = {
            cells: {},
            clusters: {},
            meshes: {},
            nodes: {},
            sequences: {},
        };
        for (let i = 0; i < reconstructions.length; i++) {
            const cluster = reconstructions[i];
            const reference = this._createReference(cluster.reference_lla);
            const clusterKey = this._createClusterKey(i);
            data.clusters[clusterKey] =
                this._createClusterReconstruction(
                    cluster, clusterKey, reference);

            const shots = cluster.shots;
            const cameras = cluster.cameras;
            const nodes = data.nodes;
            const sequences = data.sequences;
            const meshes = data.meshes;
            for (const key in shots) {
                if (!shots.hasOwnProperty(key)) {
                    continue;
                }

                const shot = shots[key];
                const camera = cameras[shot.camera];
                const node = this._createNode(
                    key,
                    clusterKey,
                    shot,
                    camera,
                    reference);

                nodes[key] = node;

                const sequenceKey = node.sequence_key;
                if (!(sequenceKey in sequences)) {
                    sequences[sequenceKey] = { key: sequenceKey, keys: [] };
                }
                sequences[sequenceKey].keys.push(key);

                meshes[key] = this._createMesh(shot);

                const cellId = this._geometry.latLonToCellId(node.cl);
                if (!(cellId in data.cells)) {
                    data.cells[cellId] = {};
                }
                data.cells[cellId][key] = node;
            }
        }

        this._reconstructions = reconstructions;
        this._data = data;
    }

    _createFallbackBuffer() {
        const canvas = document.createElement('canvas');
        canvas.width = canvas.height = 1;

        return new Promise(
            resolve => {
                canvas.toBlob(
                    async blob => {
                        const buffer = await blob.arrayBuffer();
                        resolve(buffer);
                    },
                    'image/jpeg');
            }).then(
                (buffer) => {
                    this._fallbackBuffer = buffer;
                    this._bufferPromise = null;
                    return buffer;
                });
    }

    _createMesh(shot) {
        const cameraMatrix =
            createCameraMatrix(shot.rotation, shot.translation);
        const scaledCameraMatrix = mat4Scale(cameraMatrix, shot.scale);
        const vertices = [];

        for (const [vx, vy, vz] of shot.vertices) {
            const v = glMatrix.vec3.fromValues(vx, vy, vz);
            const projected = glMatrix.vec3.create();
            glMatrix.vec3.transformMat4(projected, v, scaledCameraMatrix)
            vertices.push(...projected);
        }

        const faces = this._flatten(shot.faces);

        return { faces: faces, vertices: vertices };
    }

    _createNode(key, clusterKey, shot, camera, reference) {
        const projectionType =
            this._convertProjectionType(camera.projection_type);

        const rotation = shot.rotation;
        const translation = shot.translation;

        const vd = createViewingDirection(rotation)
        const rt = createCameraMatrix(rotation, translation);
        const upVector = createUpVector(shot.orientation, rt);
        const cca = createBearing(vd, upVector);

        const opticalCenter = createOpticalCenter(rotation, translation);
        const computedGeodetic =
            this._enuToGeodetic(opticalCenter, reference);

        const calt = computedGeodetic[2];
        const cl = { lat: computedGeodetic[0], lon: computedGeodetic[1] };

        const geodetic = this._enuToGeodetic(shot.gps_position, reference);
        const alt = geodetic[2];
        const l = { lat: geodetic[0], lon: geodetic[1] };

        const gpano = projectionType === 'equirectangular' ?
            {
                CroppedAreaLeftPixels: 0,
                CroppedAreaTopPixels: 0,
                CroppedAreaImageHeightPixels: camera.height,
                CroppedAreaImageWidthPixels: camera.width,
                FullPanoHeightPixels: camera.height,
                FullPanoWidthPixels: camera.width,
            } : null;

        let imagesUrl = new URL(
            this._options.imagesPath,
            this._options.endpoint).href;
        imagesUrl += imagesUrl.endsWith('/') ? '' : '/';
        const thumbUrl = new URL(key, imagesUrl).href;

        const meshUrl = key;
        const clusterUrl = clusterKey;
        const sequenceKey = !!shot.sequence_key ?
            shot.sequence_key : key;

        return {
            altitude: alt,
            atomic_scale: shot.scale,
            c_rotation: shot.rotation,
            ca: cca,
            camera_projection_type: projectionType,
            captured_at: shot.capture_time,
            captured_with_camera_uuid: shot.camera,
            cca,
            calt,
            cfocal: camera.focal,
            ck1: camera.k1,
            ck2: camera.k2,
            cl,
            cluster_key: clusterKey,
            cluster_url: clusterUrl,
            gpano,
            height: camera.height,
            key: key,
            l,
            merge_cc: shot.merge_cc,
            merge_version: 1,
            mesh_url: meshUrl,
            organization_key: null,
            orientation: shot.orientation,
            private: null,
            project: null,
            quality_score: 1,
            sequence_key: sequenceKey,
            thumb320_url: thumbUrl,
            thumb640_url: thumbUrl,
            thumb1024_url: thumbUrl,
            thumb2048_url: thumbUrl,
            user: {
                key: null,
                username: null,
            },
            width: camera.width,
        }
    }

    _createReference(reference) {
        return !!reference ?
            reference :
            {
                latitude: 0,
                longitude: 0,
                altitud: 0,
            };
    }

    _convertProjectionType(projectionType) {
        return projectionType === 'spherical' ?
            'equirectangular' : projectionType;
    }

    _enuToGeodetic(enu, reference) {
        return this._geoCoords.enuToGeodetic(
            enu[0],
            enu[1],
            enu[2],
            reference.latitude,
            reference.longitude,
            reference.altitude);
    }

    _filterItems(keys, items) {
        const filtered = {};
        for (const key of keys) {
            filtered[key] = items[key]
        }
        return filtered;
    }

    _flatten(a) {
        return a.reduce(
            (accumulator, currentValue) => accumulator.concat(currentValue),
            []);
    }

    _fetchReconstruction(url) {
        return fetch(
            url,
            {
                method: 'GET',
                headers: {
                    'Accept': 'application/json',
                    'Content-Type': 'application/json',
                },
            })
            .then(r => r.json());
    }

    _getDataLoadedPromise() {
        return this._loaderPromise != null ?
            this._loaderPromise : Promise.resolve();
    }

    _getFallbackBuffer() {
        if (!!this._fallbackBuffer) {
            return Promise.resolve(this._fallbackBuffer);
        }

        if (!this._bufferPromise) {
            this._bufferPromise = this._createFallbackBuffer();
        }

        return this._bufferPromise;
    }
}
