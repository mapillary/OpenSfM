(function(global, factory) {
  typeof exports === 'object' && typeof module !== 'undefined'
    ? factory(exports)
    : typeof define === 'function' && define.amd
    ? define(['exports'], factory)
    : ((global = global || self), factory((global.Potree = {})));
})(this, function(exports) {
  'use strict';

  class LRUItem {
    constructor(node) {
      this.previous = null;
      this.next = null;
      this.node = node;
    }
  }

  /**
   * @class A doubly-linked-list of the least recently used elements.
   */
  class LRU {
    constructor() {
      // the least recently used item
      this.first = null;
      // the most recently used item
      this.last = null;
      // a list of all items in the lru list
      this.items = {};
      this.elements = 0;
      this.numPoints = 0;
    }

    size() {
      return this.elements;
    }

    contains(node) {
      return this.items[node.id] == null;
    }

    touch(node) {
      if (!node.loaded) {
        return;
      }

      var item;

      if (this.items[node.id] == null) {
        // add to list
        item = new LRUItem(node);
        item.previous = this.last;
        this.last = item;
        if (item.previous !== null) {
          item.previous.next = item;
        }

        this.items[node.id] = item;
        this.elements++;

        if (this.first === null) {
          this.first = item;
        }
        this.numPoints += node.numPoints;
      } else {
        // update in list
        item = this.items[node.id];

        if (item.previous === null) {
          // handle touch on first element
          if (item.next !== null) {
            this.first = item.next;
            this.first.previous = null;
            item.previous = this.last;
            item.next = null;
            this.last = item;
            item.previous.next = item;
          }
        } else if (item.next !== null) {
          // handle touch on any other element
          item.previous.next = item.next;
          item.next.previous = item.previous;
          item.previous = this.last;
          item.next = null;
          this.last = item;
          item.previous.next = item;
        }
      }
    }

    remove(node) {
      var lruItem = this.items[node.id];
      if (lruItem) {
        if (this.elements === 1) {
          this.first = null;
          this.last = null;
        } else {
          if (!lruItem.previous) {
            this.first = lruItem.next;
            this.first.previous = null;
          }
          if (!lruItem.next) {
            this.last = lruItem.previous;
            this.last.next = null;
          }
          if (lruItem.previous && lruItem.next) {
            lruItem.previous.next = lruItem.next;
            lruItem.next.previous = lruItem.previous;
          }
        }

        delete this.items[node.id];
        this.elements--;
        this.numPoints -= node.numPoints;
      }
    }

    getLRUItem() {
      if (this.first === null) {
        return null;
      }
      var lru = this.first;

      return lru.node;
    }

    toString() {
      var string = '{ ';
      var curr = this.first;

      while (curr !== null) {
        string += curr.node.id;
        if (curr.next !== null) {
          string += ', ';
        }
        curr = curr.next;
      }

      string += '}';
      string += '(' + this.size() + ')';
      return string;
    }

    freeMemory() {
      if (this.elements <= 1) {
        return;
      }

      while (this.numPoints > Global.pointLoadLimit) {
        var element = this.first;
        var node = element.node;
        this.disposeDescendants(node);
      }
    }

    disposeDescendants(node) {
      var stack = [];
      stack.push(node);

      while (stack.length > 0) {
        var current = stack.pop();

        current.dispose();
        this.remove(current);

        for (var key in current.children) {
          if (current.children.hasOwnProperty(key)) {
            var child = current.children[key];
            if (child.loaded) {
              stack.push(current.children[key]);
            }
          }
        }
      }
    }
  }

  /**
   * The worker manager is responsible for creating and managing worker instances.
   */
  class WorkerManager {
    constructor() {
      this.workers = [];

      for (var i = 0; i < WorkerManager.URLS.length; i++) {
        this.workers.push([]);
      }
    }

    /**
     * Get a worker from the pool, if none available one will be created.
     */
    getWorker(type) {
      if (this.workers[type].length > 0) {
        return this.workers[type].pop();
      }

      return new Worker(Global.workerPath + WorkerManager.URLS[type]);
    }

    /**
     * Return (reinsert) the worker into the pool.
     */
    returnWorker(type, worker) {
      this.workers[type].push(worker);
    }

    /**
     * Run a task immediatly.
     */
    runTask(type, onMessage, message, transfer) {
      var self = this;

      var worker = this.getWorker(type);
      worker.onmessage = function(event) {
        onMessage(event);
        self.returnWorker(type, worker);
      };

      if (transfer !== undefined) {
        worker.postMessage(message, transfer);
      } else {
        worker.postMessage(message);
      }
    }
  }
  WorkerManager.BINARY_DECODER = 0;
  WorkerManager.LAS_LAZ = 1;
  WorkerManager.LAS_DECODER = 2;
  WorkerManager.GREYHOUND = 3;
  WorkerManager.DEM = 4;
  WorkerManager.EPT_LAS_ZIP_DECODER = 5;
  WorkerManager.EPT_BINARY_DECODER = 6;

  WorkerManager.URLS = [
    '/workers/BinaryDecoderWorker.js',
    '/workers/LASLAZWorker.js',
    '/workers/LASDecoderWorker.js',
    '/workers/GreyhoundBinaryDecoderWorker.js',
    '/workers/DEMWorker.js',
    '/workers/EptLaszipDecoderWorker.js',
    '/workers/EptBinaryDecoderWorker.js',
  ];

  function getBasePath() {
    if (document.currentScript.src) {
      var scriptPath = new URL(document.currentScript.src + '/..').href;

      if (scriptPath.slice(-1) === '/') {
        scriptPath = scriptPath.slice(0, -1);
      }

      return scriptPath;
    } else {
      console.error(
        'Potree: Was unable to find its script path using document.currentScript.'
      );
    }

    return '';
  }

  var Global = {
    debug: {},
    workerPath: getBasePath(),
    maxNodesLoadGPUFrame: 20,
    maxDEMLevel: 0,
    maxNodesLoading:
      navigator.hardwareConcurrency !== undefined
        ? navigator.hardwareConcurrency
        : 4,
    pointLoadLimit: 1e10,
    numNodesLoading: 0,
    measureTimings: false,
    workerPool: new WorkerManager(),
    lru: new LRU(),
    pointcloudTransformVersion: undefined,
  };

  var PointAttributeNames = {
    POSITION_CARTESIAN: 0, //float x, y, z,
    COLOR_PACKED: 1, //byte r, g, b, a, I: [0,1]
    COLOR_FLOATS_1: 2, //float r, g, b, I: [0,1]
    COLOR_FLOATS_255: 3, //float r, g, b, I: [0,255]
    NORMAL_FLOATS: 4, //float x, y, z,
    FILLER: 5,
    INTENSITY: 6,
    CLASSIFICATION: 7,
    NORMAL_SPHEREMAPPED: 8,
    NORMAL_OCT16: 9,
    NORMAL: 10,
    RETURN_NUMBER: 11,
    NUMBER_OF_RETURNS: 12,
    SOURCE_ID: 13,
    INDICES: 14,
    SPACING: 15,
  };

  /**
   * Some types of possible point attribute data formats
   *
   * @class
   */
  var PointAttributeTypes = {
    DATA_TYPE_DOUBLE: {
      ordinal: 0,
      size: 8,
    },
    DATA_TYPE_FLOAT: {
      ordinal: 1,
      size: 4,
    },
    DATA_TYPE_INT8: {
      ordinal: 2,
      size: 1,
    },
    DATA_TYPE_UINT8: {
      ordinal: 3,
      size: 1,
    },
    DATA_TYPE_INT16: {
      ordinal: 4,
      size: 2,
    },
    DATA_TYPE_UINT16: {
      ordinal: 5,
      size: 2,
    },
    DATA_TYPE_INT32: {
      ordinal: 6,
      size: 4,
    },
    DATA_TYPE_UINT32: {
      ordinal: 7,
      size: 4,
    },
    DATA_TYPE_INT64: {
      ordinal: 8,
      size: 8,
    },
    DATA_TYPE_UINT64: {
      ordinal: 9,
      size: 8,
    },
  };

  var i = 0;
  for (var obj in PointAttributeTypes) {
    PointAttributeTypes[i] = PointAttributeTypes[obj];
    i++;
  }

  /**
   * A single point attribute such as color/normal/.. and its data format/number of elements/...
   */
  function PointAttribute(name, type, numElements) {
    this.name = name;
    this.type = type;
    this.numElements = numElements;
    this.byteSize = this.numElements * this.type.size;
  }
  PointAttribute.POSITION_CARTESIAN = new PointAttribute(
    PointAttributeNames.POSITION_CARTESIAN,
    PointAttributeTypes.DATA_TYPE_FLOAT,
    3
  );
  PointAttribute.RGBA_PACKED = new PointAttribute(
    PointAttributeNames.COLOR_PACKED,
    PointAttributeTypes.DATA_TYPE_INT8,
    4
  );
  PointAttribute.COLOR_PACKED = PointAttribute.RGBA_PACKED;
  PointAttribute.RGB_PACKED = new PointAttribute(
    PointAttributeNames.COLOR_PACKED,
    PointAttributeTypes.DATA_TYPE_INT8,
    3
  );
  PointAttribute.NORMAL_FLOATS = new PointAttribute(
    PointAttributeNames.NORMAL_FLOATS,
    PointAttributeTypes.DATA_TYPE_FLOAT,
    3
  );
  PointAttribute.FILLER_1B = new PointAttribute(
    PointAttributeNames.FILLER,
    PointAttributeTypes.DATA_TYPE_UINT8,
    1
  );
  PointAttribute.INTENSITY = new PointAttribute(
    PointAttributeNames.INTENSITY,
    PointAttributeTypes.DATA_TYPE_UINT16,
    1
  );
  PointAttribute.CLASSIFICATION = new PointAttribute(
    PointAttributeNames.CLASSIFICATION,
    PointAttributeTypes.DATA_TYPE_UINT8,
    1
  );
  PointAttribute.NORMAL_SPHEREMAPPED = new PointAttribute(
    PointAttributeNames.NORMAL_SPHEREMAPPED,
    PointAttributeTypes.DATA_TYPE_UINT8,
    2
  );
  PointAttribute.NORMAL_OCT16 = new PointAttribute(
    PointAttributeNames.NORMAL_OCT16,
    PointAttributeTypes.DATA_TYPE_UINT8,
    2
  );
  PointAttribute.NORMAL = new PointAttribute(
    PointAttributeNames.NORMAL,
    PointAttributeTypes.DATA_TYPE_FLOAT,
    3
  );
  PointAttribute.RETURN_NUMBER = new PointAttribute(
    PointAttributeNames.RETURN_NUMBER,
    PointAttributeTypes.DATA_TYPE_UINT8,
    1
  );
  PointAttribute.NUMBER_OF_RETURNS = new PointAttribute(
    PointAttributeNames.NUMBER_OF_RETURNS,
    PointAttributeTypes.DATA_TYPE_UINT8,
    1
  );
  PointAttribute.SOURCE_ID = new PointAttribute(
    PointAttributeNames.SOURCE_ID,
    PointAttributeTypes.DATA_TYPE_UINT8,
    1
  );
  PointAttribute.INDICES = new PointAttribute(
    PointAttributeNames.INDICES,
    PointAttributeTypes.DATA_TYPE_UINT32,
    1
  );
  PointAttribute.SPACING = new PointAttribute(
    PointAttributeNames.SPACING,
    PointAttributeTypes.DATA_TYPE_FLOAT,
    1
  );

  /**
   * Ordered list of PointAttributes used to identify how points are aligned in a buffer.
   */
  function PointAttributes(pointAttributes) {
    this.attributes = [];
    this.byteSize = 0;
    this.size = 0;

    if (pointAttributes != null) {
      for (var i = 0; i < pointAttributes.length; i++) {
        var pointAttributeName = pointAttributes[i];
        var pointAttribute = PointAttribute[pointAttributeName];
        this.attributes.push(pointAttribute);
        this.byteSize += pointAttribute.byteSize;
        this.size++;
      }
    }
  }
  PointAttributes.prototype.add = function(pointAttribute) {
    this.attributes.push(pointAttribute);
    this.byteSize += pointAttribute.byteSize;
    this.size++;
  };

  PointAttributes.prototype.hasColors = function() {
    for (var name in this.attributes) {
      var pointAttribute = this.attributes[name];
      if (pointAttribute.name === PointAttributeNames.COLOR_PACKED) {
        return true;
      }
    }

    return false;
  };

  PointAttributes.prototype.hasNormals = function() {
    for (var name in this.attributes) {
      var pointAttribute = this.attributes[name];
      if (
        pointAttribute === PointAttribute.NORMAL_SPHEREMAPPED ||
        pointAttribute === PointAttribute.NORMAL_FLOATS ||
        pointAttribute === PointAttribute.NORMAL ||
        pointAttribute === PointAttribute.NORMAL_OCT16
      ) {
        return true;
      }
    }

    return false;
  };

  //
  //index is in order xyzxyzxyz
  //
  class DEMNode {
    constructor(name, box, tileSize) {
      this.name = name;
      this.box = box;
      this.tileSize = tileSize;
      this.level = this.name.length - 1;
      this.data = new Float32Array(tileSize * tileSize);
      this.data.fill(-Infinity);
      this.children = [];

      this.mipMap = [this.data];
      this.mipMapNeedsUpdate = true;
    }

    createMipMap() {
      this.mipMap = [this.data];

      var sourceSize = this.tileSize;
      var mipSize = parseInt(sourceSize / 2);
      var mipSource = this.data;
      while (mipSize > 1) {
        var mipData = new Float32Array(mipSize * mipSize);

        for (var i = 0; i < mipSize; i++) {
          for (var j = 0; j < mipSize; j++) {
            var h00 = mipSource[2 * i + 0 + 2 * j * sourceSize];
            var h01 = mipSource[2 * i + 0 + 2 * j * sourceSize + sourceSize];
            var h10 = mipSource[2 * i + 1 + 2 * j * sourceSize];
            var h11 = mipSource[2 * i + 1 + 2 * j * sourceSize + sourceSize];

            var [height, weight] = [0, 0];

            if (isFinite(h00)) {
              height += h00;
              weight += 1;
            }
            if (isFinite(h01)) {
              height += h01;
              weight += 1;
            }
            if (isFinite(h10)) {
              height += h10;
              weight += 1;
            }
            if (isFinite(h11)) {
              height += h11;
              weight += 1;
            }
            height = height / weight;

            //var hs = [h00, h01, h10, h11].filter(h => isFinite(h));
            //var height = hs.reduce((a, v, i) => a + v, 0) / hs.length;

            mipData[i + j * mipSize] = height;
          }
        }

        this.mipMap.push(mipData);

        mipSource = mipData;
        sourceSize = mipSize;
        mipSize = parseInt(mipSize / 2);
      }

      this.mipMapNeedsUpdate = false;
    }

    uv(position) {
      var boxSize = this.box.getSize(new THREE.Vector3());

      var u = (position.x - this.box.min.x) / boxSize.x;
      var v = (position.y - this.box.min.y) / boxSize.y;

      return [u, v];
    }

    heightAtMipMapLevel(position, mipMapLevel) {
      var uv = this.uv(position);

      var tileSize = parseInt(this.tileSize / parseInt(2 ** mipMapLevel));
      var data = this.mipMap[mipMapLevel];

      var i = Math.min(uv[0] * tileSize, tileSize - 1);
      var j = Math.min(uv[1] * tileSize, tileSize - 1);

      var a = i % 1;
      var b = j % 1;

      var [i0, i1] = [Math.floor(i), Math.ceil(i)];
      var [j0, j1] = [Math.floor(j), Math.ceil(j)];

      var h00 = data[i0 + tileSize * j0];
      var h01 = data[i0 + tileSize * j1];
      var h10 = data[i1 + tileSize * j0];
      var h11 = data[i1 + tileSize * j1];

      var wh00 = isFinite(h00) ? (1 - a) * (1 - b) : 0;
      var wh01 = isFinite(h01) ? (1 - a) * b : 0;
      var wh10 = isFinite(h10) ? a * (1 - b) : 0;
      var wh11 = isFinite(h11) ? a * b : 0;

      var wsum = wh00 + wh01 + wh10 + wh11;
      wh00 = wh00 / wsum;
      wh01 = wh01 / wsum;
      wh10 = wh10 / wsum;
      wh11 = wh11 / wsum;

      if (wsum === 0) {
        return null;
      }

      var h = 0;

      if (isFinite(h00)) h += h00 * wh00;
      if (isFinite(h01)) h += h01 * wh01;
      if (isFinite(h10)) h += h10 * wh10;
      if (isFinite(h11)) h += h11 * wh11;

      return h;
    }

    height(position) {
      var h = null;

      for (var i = 0; i < this.mipMap.length; i++) {
        h = this.heightAtMipMapLevel(position, i);

        if (h !== null) {
          return h;
        }
      }

      return h;
    }

    traverse(handler, level = 0) {
      handler(this, level);

      for (var child of this.children.filter(c => c !== undefined)) {
        child.traverse(handler, level + 1);
      }
    }
  }

  class DEM$1 {
    constructor(pointcloud) {
      this.pointcloud = pointcloud;
      this.matrix = null;
      this.boundingBox = null;
      this.tileSize = 64;
      this.root = null;
      this.version = 0;
    }

    //expands the tree to all nodes that intersect <box> at <level> returns the intersecting nodes at <level>
    expandAndFindByBox(box, level) {
      if (level === 0) {
        return [this.root];
      }

      var result = [];
      var stack = [this.root];

      while (stack.length > 0) {
        var node = stack.pop();
        var nodeBoxSize = node.box.getSize(new THREE.Vector3());

        //check which children intersect by transforming min/max to quadrants
        var min = {
          x: (box.min.x - node.box.min.x) / nodeBoxSize.x,
          y: (box.min.y - node.box.min.y) / nodeBoxSize.y,
        };
        var max = {
          x: (box.max.x - node.box.max.x) / nodeBoxSize.x,
          y: (box.max.y - node.box.max.y) / nodeBoxSize.y,
        };

        min.x = min.x < 0.5 ? 0 : 1;
        min.y = min.y < 0.5 ? 0 : 1;
        max.x = max.x < 0.5 ? 0 : 1;
        max.y = max.y < 0.5 ? 0 : 1;

        var childIndices;
        if (min.x === 0 && min.y === 0 && max.x === 1 && max.y === 1) {
          childIndices = [0, 1, 2, 3];
        } else if (min.x === max.x && min.y === max.y) {
          childIndices = [(min.x << 1) | min.y];
        } else {
          childIndices = [(min.x << 1) | min.y, (max.x << 1) | max.y];
        }

        for (var index of childIndices) {
          if (node.children[index] === undefined) {
            var childBox = node.box.clone();

            if ((index & 2) > 0) {
              childBox.min.x += nodeBoxSize.x / 2.0;
            } else {
              childBox.max.x -= nodeBoxSize.x / 2.0;
            }

            if ((index & 1) > 0) {
              childBox.min.y += nodeBoxSize.y / 2.0;
            } else {
              childBox.max.y -= nodeBoxSize.y / 2.0;
            }

            var child = new DEMNode(node.name + index, childBox, this.tileSize);
            node.children[index] = child;
          }

          var child = node.children[index];

          if (child.level < level) {
            stack.push(child);
          } else {
            result.push(child);
          }
        }
      }

      return result;
    }

    childIndex(uv) {
      var [x, y] = uv.map(n => (n < 0.5 ? 0 : 1));

      var index = (x << 1) | y;

      return index;
    }

    height(position) {
      if (!this.root) {
        return 0;
      }

      var height = null;
      var list = [this.root];
      while (true) {
        var node = list[list.length - 1];

        var currentHeight = node.height(position);

        if (currentHeight !== null) {
          height = currentHeight;
        }

        var uv = node.uv(position);
        var childIndex = this.childIndex(uv);

        if (node.children[childIndex]) {
          list.push(node.children[childIndex]);
        } else {
          break;
        }
      }

      return height + this.pointcloud.position.z;
    }

    update(visibleNodes) {
      //check if point cloud transformation changed
      if (
        this.matrix === null ||
        !this.matrix.equals(this.pointcloud.matrixWorld)
      ) {
        this.matrix = this.pointcloud.matrixWorld.clone();
        this.boundingBox = this.pointcloud.boundingBox
          .clone()
          .applyMatrix4(this.matrix);
        this.root = new DEMNode('r', this.boundingBox, this.tileSize);
        this.version++;
      }

      //find node to update
      var node = null;
      for (var vn of visibleNodes) {
        if (vn.demVersion === undefined || vn.demVersion < this.version) {
          node = vn;
          break;
        }
      }
      if (node === null) {
        return;
      }

      //update node
      var projectedBox = node
        .getBoundingBox()
        .clone()
        .applyMatrix4(this.matrix);
      var projectedBoxSize = projectedBox.getSize(new THREE.Vector3());

      var targetNodes = this.expandAndFindByBox(projectedBox, node.getLevel());
      node.demVersion = this.version;

      var position = node.geometryNode.geometry.attributes.position.array;
      var message = {
        boundingBox: {
          min: node.getBoundingBox().min.toArray(),
          max: node.getBoundingBox().max.toArray(),
        },
        position: new Float32Array(position).buffer,
      };
      var transferables = [message.position];

      var self = this;

      Global.workerPool.runTask(
        WorkerManager.DEM,
        function(e) {
          var data = new Float32Array(e.data.dem.data);

          for (var demNode of targetNodes) {
            var boxSize = demNode.box.getSize(new THREE.Vector3());

            for (var i = 0; i < self.tileSize; i++) {
              for (var j = 0; j < self.tileSize; j++) {
                var u = i / (self.tileSize - 1);
                var v = j / (self.tileSize - 1);

                var x = demNode.box.min.x + u * boxSize.x;
                var y = demNode.box.min.y + v * boxSize.y;

                var ix =
                  (self.tileSize * (x - projectedBox.min.x)) /
                  projectedBoxSize.x;
                var iy =
                  (self.tileSize * (y - projectedBox.min.y)) /
                  projectedBoxSize.y;

                if (ix < 0 || ix > self.tileSize) {
                  continue;
                }

                if (iy < 0 || iy > self.tileSize) {
                  continue;
                }

                ix = Math.min(Math.floor(ix), self.tileSize - 1);
                iy = Math.min(Math.floor(iy), self.tileSize - 1);

                demNode.data[i + self.tileSize * j] =
                  data[ix + self.tileSize * iy];
              }
            }

            demNode.createMipMap();
            demNode.mipMapNeedsUpdate = true;
          }
        },
        message,
        transferables
      );
    }
  }

  class PointCloudTreeNode {
    constructor() {
      this.needsTransformUpdate = true;
    }

    getChildren() {}

    getBoundingBox() {}

    isLoaded() {}

    isGeometryNode() {}

    isTreeNode() {}

    getLevel() {}

    getBoundingSphere() {}
  }
  class PointCloudTree extends THREE.Object3D {
    constructor() {
      super();

      this.dem = new DEM$1(this);
    }

    initialized() {
      return this.root !== null;
    }
  }

  class PointCloudGreyhoundGeometry {
    constructor() {
      this.spacing = 0;
      this.boundingBox = null;
      this.root = null;
      this.nodes = null;
      this.pointAttributes = {};
      this.hierarchyStepSize = -1;
      this.loader = null;
      this.schema = null;

      this.baseDepth = null;
      this.offset = null;
      this.projection = null;

      this.boundingSphere = null;

      // the serverURL will contain the base URL of the greyhound server. f.e. http://dev.greyhound.io/resource/autzen/
      this.serverURL = null;
      this.normalize = { color: false, intensity: false };
    }
  }

  function PointCloudGreyhoundGeometryNode(
    name,
    pcoGeometry,
    boundingBox,
    scale,
    offset
  ) {
    this.id = PointCloudGreyhoundGeometryNode.IDCount++;
    this.name = name;
    this.index = parseInt(name.charAt(name.length - 1));
    this.pcoGeometry = pcoGeometry;
    this.geometry = null;
    this.boundingBox = boundingBox;
    this.boundingSphere = boundingBox.getBoundingSphere(new THREE.Sphere());
    this.scale = scale;
    this.offset = offset;
    this.children = {};
    this.numPoints = 0;
    this.level = null;
    this.loaded = false;
    this.oneTimeDisposeHandlers = [];
    this.baseLoaded = false;

    var center = new THREE.Vector3();

    var bounds = this.boundingBox.clone();
    bounds.min.sub(this.pcoGeometry.boundingBox.getCenter(center));
    bounds.max.sub(this.pcoGeometry.boundingBox.getCenter(center));

    if (this.scale) {
      bounds.min.multiplyScalar(1 / this.scale);
      bounds.max.multiplyScalar(1 / this.scale);
    }

    //This represents the bounds for this node in the reference frame of the
    //global bounds from `info`, centered around the origin, and then scaled
    //by our selected scale.
    this.greyhoundBounds = bounds;

    //This represents the offset between the coordinate system described above
    //and our pcoGeometry bounds.
    this.greyhoundOffset = this.pcoGeometry.offset
      .clone()
      .add(
        this.pcoGeometry.boundingBox
          .getSize(new THREE.Vector3())
          .multiplyScalar(0.5)
      );
  }
  PointCloudGreyhoundGeometryNode.IDCount = 0;

  PointCloudGreyhoundGeometryNode.prototype = Object.create(
    PointCloudTreeNode.prototype
  );

  PointCloudGreyhoundGeometryNode.prototype.isGeometryNode = function() {
    return true;
  };

  PointCloudGreyhoundGeometryNode.prototype.isTreeNode = function() {
    return false;
  };

  PointCloudGreyhoundGeometryNode.prototype.isLoaded = function() {
    return this.loaded;
  };

  PointCloudGreyhoundGeometryNode.prototype.getBoundingSphere = function() {
    return this.boundingSphere;
  };

  PointCloudGreyhoundGeometryNode.prototype.getBoundingBox = function() {
    return this.boundingBox;
  };

  PointCloudGreyhoundGeometryNode.prototype.getLevel = function() {
    return this.level;
  };

  PointCloudGreyhoundGeometryNode.prototype.getChildren = function() {
    var children = [];

    for (var i = 0; i < 8; ++i) {
      if (this.children[i]) {
        children.push(this.children[i]);
      }
    }

    return children;
  };

  PointCloudGreyhoundGeometryNode.prototype.getURL = function() {
    var schema = this.pcoGeometry.schema;
    var bounds = this.greyhoundBounds;

    var boundsString =
      bounds.min.x +
      ',' +
      bounds.min.y +
      ',' +
      bounds.min.z +
      ',' +
      bounds.max.x +
      ',' +
      bounds.max.y +
      ',' +
      bounds.max.z;

    var url =
      '' +
      this.pcoGeometry.serverURL +
      'read?depthBegin=' +
      (this.baseLoaded ? this.level + this.pcoGeometry.baseDepth : 0) +
      '&depthEnd=' +
      (this.level + this.pcoGeometry.baseDepth + 1) +
      '&bounds=[' +
      boundsString +
      ']' +
      '&schema=' +
      JSON.stringify(schema) +
      '&compress=true';

    if (this.scale) {
      url += '&scale=' + this.scale;
    }

    if (this.greyhoundOffset) {
      var offset = this.greyhoundOffset;
      url += '&offset=[' + offset.x + ',' + offset.y + ',' + offset.z + ']';
    }

    if (!this.baseLoaded) this.baseLoaded = true;

    return url;
  };

  PointCloudGreyhoundGeometryNode.prototype.addChild = function(child) {
    this.children[child.index] = child;
    child.parent = this;
  };

  PointCloudGreyhoundGeometryNode.prototype.load = function() {
    if (
      this.loading === true ||
      this.loaded === true ||
      Global.numNodesLoading >= Global.maxNodesLoading
    ) {
      return;
    }

    this.loading = true;
    Global.numNodesLoading++;

    if (
      this.level % this.pcoGeometry.hierarchyStepSize === 0 &&
      this.hasChildren
    ) {
      this.loadHierarchyThenPoints();
    } else {
      this.loadPoints();
    }
  };

  PointCloudGreyhoundGeometryNode.prototype.loadPoints = function() {
    this.pcoGeometry.loader.load(this);
  };

  PointCloudGreyhoundGeometryNode.prototype.loadHierarchyThenPoints = function() {
    //From Greyhound (Cartesian) ordering for the octree to Potree-default
    var transform = [0, 2, 1, 3, 4, 6, 5, 7];

    var makeBitMask = function(node) {
      var mask = 0;
      Object.keys(node).forEach(function(key) {
        if (key === 'swd') mask += 1 << transform[0];
        else if (key === 'nwd') mask += 1 << transform[1];
        else if (key === 'swu') mask += 1 << transform[2];
        else if (key === 'nwu') mask += 1 << transform[3];
        else if (key === 'sed') mask += 1 << transform[4];
        else if (key === 'ned') mask += 1 << transform[5];
        else if (key === 'seu') mask += 1 << transform[6];
        else if (key === 'neu') mask += 1 << transform[7];
      });
      return mask;
    };

    var parseChildrenCounts = function(base, parentName, stack) {
      var keys = Object.keys(base);
      var child;
      var childName;

      keys.forEach(function(key) {
        if (key === 'n') return;
        switch (key) {
          case 'swd':
            child = base.swd;
            childName = parentName + transform[0];
            break;
          case 'nwd':
            child = base.nwd;
            childName = parentName + transform[1];
            break;
          case 'swu':
            child = base.swu;
            childName = parentName + transform[2];
            break;
          case 'nwu':
            child = base.nwu;
            childName = parentName + transform[3];
            break;
          case 'sed':
            child = base.sed;
            childName = parentName + transform[4];
            break;
          case 'ned':
            child = base.ned;
            childName = parentName + transform[5];
            break;
          case 'seu':
            child = base.seu;
            childName = parentName + transform[6];
            break;
          case 'neu':
            child = base.neu;
            childName = parentName + transform[7];
            break;
          default:
            break;
        }

        stack.push({
          children: makeBitMask(child),
          numPoints: child.n,
          name: childName,
        });

        parseChildrenCounts(child, childName, stack);
      });
    };

    //Load hierarchy.
    var callback = function(node, greyhoundHierarchy) {
      var decoded = [];
      node.numPoints = greyhoundHierarchy.n;
      parseChildrenCounts(greyhoundHierarchy, node.name, decoded);

      var nodes = {};
      nodes[node.name] = node;
      var pgg = node.pcoGeometry;

      for (var i = 0; i < decoded.length; i++) {
        var name = decoded[i].name;
        var numPoints = decoded[i].numPoints;
        var index = parseInt(name.charAt(name.length - 1));
        var parentName = name.substring(0, name.length - 1);
        var parentNode = nodes[parentName];
        var level = name.length - 1;
        var boundingBox = GreyhoundLoader.createChildAABB(
          parentNode.boundingBox,
          index
        );

        var currentNode = new PointCloudGreyhoundGeometryNode(
          name,
          pgg,
          boundingBox,
          node.scale,
          node.offset
        );
        currentNode.level = level;
        currentNode.numPoints = numPoints;
        currentNode.hasChildren = decoded[i].children > 0;
        currentNode.spacing = pgg.spacing / Math.pow(2, level);

        parentNode.addChild(currentNode);
        nodes[name] = currentNode;
      }

      node.loadPoints();
    };

    if (this.level % this.pcoGeometry.hierarchyStepSize === 0) {
      var depthBegin = this.level + this.pcoGeometry.baseDepth;
      var depthEnd = depthBegin + this.pcoGeometry.hierarchyStepSize + 2;

      var bounds = this.greyhoundBounds;

      var boundsString =
        bounds.min.x +
        ',' +
        bounds.min.y +
        ',' +
        bounds.min.z +
        ',' +
        bounds.max.x +
        ',' +
        bounds.max.y +
        ',' +
        bounds.max.z;

      var hurl =
        '' +
        this.pcoGeometry.serverURL +
        'hierarchy?bounds=[' +
        boundsString +
        ']' +
        '&depthBegin=' +
        depthBegin +
        '&depthEnd=' +
        depthEnd;

      if (this.scale) {
        hurl += '&scale=' + this.scale;
      }

      if (this.greyhoundOffset) {
        var offset = this.greyhoundOffset;
        hurl += '&offset=[' + offset.x + ',' + offset.y + ',' + offset.z + ']';
      }

      var self = this;
      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', hurl, true);
      xhr.onload = function(event) {
        var greyhoundHierarchy = JSON.parse(xhr.responseText) || {};
        callback(self, greyhoundHierarchy);
      };
      xhr.onerror = function(event) {
        console.log(
          'Potree: Failed to load file! HTTP status ' +
            xhr.status +
            ', file:' +
            hurl,
          event
        );
      };
      xhr.send(null);
    }
  };

  PointCloudGreyhoundGeometryNode.prototype.getNumPoints = function() {
    return this.numPoints;
  };

  PointCloudGreyhoundGeometryNode.prototype.dispose = function() {
    if (this.geometry && this.parent != null) {
      this.geometry.dispose();
      this.geometry = null;
      this.loaded = false;

      for (var i = 0; i < this.oneTimeDisposeHandlers.length; i++) {
        var handler = this.oneTimeDisposeHandlers[i];
        handler();
      }

      this.oneTimeDisposeHandlers = [];
    }
  };

  Object.assign(
    PointCloudGreyhoundGeometryNode.prototype,
    THREE.EventDispatcher.prototype
  );

  function VersionUtils(version) {
    this.version = version;
    var vmLength =
      version.indexOf('.') === -1 ? version.length : version.indexOf('.');
    this.versionMajor = parseInt(version.substr(0, vmLength));
    this.versionMinor = parseInt(version.substr(vmLength + 1));

    if (this.versionMinor.length === 0) {
      this.versionMinor = 0;
    }
  }
  VersionUtils.prototype.newerThan = function(version) {
    var v = new VersionUtils(version);

    if (
      this.versionMajor > v.versionMajor ||
      (this.versionMajor === v.versionMajor &&
        this.versionMinor > v.versionMinor)
    ) {
      return true;
    }

    return false;
  };

  VersionUtils.prototype.equalOrHigher = function(version) {
    var v = new VersionUtils(version);

    if (
      this.versionMajor > v.versionMajor ||
      (this.versionMajor === v.versionMajor &&
        this.versionMinor >= v.versionMinor)
    ) {
      return true;
    }

    return false;
  };

  VersionUtils.prototype.upTo = function(version) {
    return !this.newerThan(version);
  };

  class GreyhoundBinaryLoader {
    constructor(version, boundingBox, scale) {
      if (typeof version === 'string') {
        this.version = new VersionUtils(version);
      } else {
        this.version = version;
      }

      this.boundingBox = boundingBox;
      this.scale = scale;
    }

    load(node) {
      if (node.loaded) return;

      var self = this;
      var url = node.getURL();

      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';
      xhr.overrideMimeType('text/plain; charset=x-user-defined');
      xhr.onreadystatechange = function() {
        if (xhr.readyState === 4) {
          if (xhr.status === 200 || xhr.status === 0) {
            var buffer = xhr.response;
            self.parse(node, buffer);
          } else {
            console.log('Potree: Failed to load file.', xhr, url);
          }
        }
      };

      xhr.send(null);
    }

    parse(node, buffer) {
      var NUM_POINTS_BYTES = 4;
      var view = new DataView(
        buffer,
        buffer.byteLength - NUM_POINTS_BYTES,
        NUM_POINTS_BYTES
      );
      var numPoints = view.getUint32(0, true);
      var pointAttributes = node.pcoGeometry.pointAttributes;

      node.numPoints = numPoints;

      var bb = node.boundingBox;
      var center = new THREE.Vector3();
      var nodeOffset = node.pcoGeometry.boundingBox
        .getCenter(center)
        .sub(node.boundingBox.min);

      var message = {
        buffer: buffer,
        pointAttributes: pointAttributes,
        version: this.version.version,
        schema: node.pcoGeometry.schema,
        min: [bb.min.x, bb.min.y, bb.min.z],
        max: [bb.max.x, bb.max.y, bb.max.z],
        offset: nodeOffset.toArray(),
        scale: this.scale,
        normalize: node.pcoGeometry.normalize,
      };

      Global.workerPool.runTask(
        WorkerManager.GREYHOUND,
        function(e) {
          var data = e.data;
          var buffers = data.attributeBuffers;

          var tightBoundingBox = new THREE.Box3(
            new THREE.Vector3().fromArray(data.tightBoundingBox.min),
            new THREE.Vector3().fromArray(data.tightBoundingBox.max)
          );

          var geometry = new THREE.BufferGeometry();

          for (var property in buffers) {
            var buffer = buffers[property].buffer;

            if (parseInt(property) === PointAttributeNames.POSITION_CARTESIAN) {
              geometry.addAttribute(
                'position',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (
              parseInt(property) === PointAttributeNames.COLOR_PACKED
            ) {
              geometry.addAttribute(
                'color',
                new THREE.BufferAttribute(new Uint8Array(buffer), 4, true)
              );
            } else if (parseInt(property) === PointAttributeNames.INTENSITY) {
              geometry.addAttribute(
                'intensity',
                new THREE.BufferAttribute(new Float32Array(buffer), 1)
              );
            } else if (
              parseInt(property) === PointAttributeNames.CLASSIFICATION
            ) {
              geometry.addAttribute(
                'classification',
                new THREE.BufferAttribute(new Uint8Array(buffer), 1)
              );
            } else if (
              parseInt(property) === PointAttributeNames.NORMAL_SPHEREMAPPED
            ) {
              geometry.addAttribute(
                'normal',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (
              parseInt(property) === PointAttributeNames.NORMAL_OCT16
            ) {
              geometry.addAttribute(
                'normal',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (parseInt(property) === PointAttributeNames.NORMAL) {
              geometry.addAttribute(
                'normal',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (parseInt(property) === PointAttributeNames.INDICES) {
              var bufferAttribute = new THREE.BufferAttribute(
                new Uint8Array(buffer),
                4
              );
              bufferAttribute.normalized = true;
              geometry.addAttribute('indices', bufferAttribute);
            } else if (parseInt(property) === PointAttributeNames.SPACING) {
              var bufferAttribute = new THREE.BufferAttribute(
                new Float32Array(buffer),
                1
              );
              geometry.addAttribute('spacing', bufferAttribute);
            }
          }

          tightBoundingBox.max.sub(tightBoundingBox.min);
          tightBoundingBox.min.set(0, 0, 0);

          node.numPoints = data.numPoints;
          node.geometry = geometry;
          node.mean = new THREE.Vector3(...data.mean);
          node.tightBoundingBox = tightBoundingBox;
          node.loaded = true;
          node.loading = false;
          Global.numNodesLoading--;
        },
        message,
        [message.buffer]
      );
    }
  }

  /**
   * @class Loads greyhound metadata and returns a PointcloudOctree
   *
   * @author Maarten van Meersbergen
   * @author Oscar Martinez Rubi
   * @author Connor Manning
   */
  class GreyhoundUtils {
    static getQueryParam(name) {
      name = name.replace(/[[\]]/g, '\\$&');
      var regex = new RegExp('[?&]' + name + '(=([^&#]*)|&|#|$)');
      var results = regex.exec(window.location.href);
      if (!results) return null;
      if (!results[2]) return '';
      return decodeURIComponent(results[2].replace(/\+/g, ' '));
    }

    static createSchema(attributes) {
      var schema = [
        {
          name: 'X',
          size: 4,
          type: 'signed',
        },
        {
          name: 'Y',
          size: 4,
          type: 'signed',
        },
        {
          name: 'Z',
          size: 4,
          type: 'signed',
        },
      ];

      //Once we include options in the UI to load a dynamic list of available
      //attributes for visualization (f.e. Classification, Intensity etc.)
      //we will be able to ask for that specific attribute from the server,
      //where we are now requesting all attributes for all points all the time.
      //If we do that though, we also need to tell Potree to redraw the points
      //that are already loaded (with different attributes).
      //This is not default behaviour.
      attributes.forEach(function(item) {
        if (item === 'COLOR_PACKED') {
          schema.push({
            name: 'Red',
            size: 2,
            type: 'unsigned',
          });
          schema.push({
            name: 'Green',
            size: 2,
            type: 'unsigned',
          });
          schema.push({
            name: 'Blue',
            size: 2,
            type: 'unsigned',
          });
        } else if (item === 'INTENSITY') {
          schema.push({
            name: 'Intensity',
            size: 2,
            type: 'unsigned',
          });
        } else if (item === 'CLASSIFICATION') {
          schema.push({
            name: 'Classification',
            size: 1,
            type: 'unsigned',
          });
        }
      });

      return schema;
    }

    static fetch(url, cb) {
      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', url, true);
      xhr.onreadystatechange = function() {
        if (xhr.readyState === 4) {
          if (xhr.status === 200 || xhr.status === 0) {
            cb(null, xhr.responseText);
          } else {
            cb(xhr.responseText);
          }
        }
      };
      xhr.send(null);
    }

    static fetchBinary(url, cb) {
      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';
      xhr.onreadystatechange = function() {
        if (xhr.readyState === 4) {
          if (xhr.status === 200 || xhr.status === 0) {
            cb(null, xhr.response);
          } else {
            cb(xhr.responseText);
          }
        }
      };
      xhr.send(null);
    }

    static pointSizeFrom(schema) {
      return schema.reduce((p, c) => p + c.size, 0);
    }

    static getNormalization(serverURL, baseDepth, cb) {
      var s = [
        {
          name: 'X',
          size: 4,
          type: 'floating',
        },
        {
          name: 'Y',
          size: 4,
          type: 'floating',
        },
        {
          name: 'Z',
          size: 4,
          type: 'floating',
        },
        {
          name: 'Red',
          size: 2,
          type: 'unsigned',
        },
        {
          name: 'Green',
          size: 2,
          type: 'unsigned',
        },
        {
          name: 'Blue',
          size: 2,
          type: 'unsigned',
        },
        {
          name: 'Intensity',
          size: 2,
          type: 'unsigned',
        },
      ];

      var url =
        serverURL + 'read?depth=' + baseDepth + '&schema=' + JSON.stringify(s);

      GreyhoundUtils.fetchBinary(url, function(err, buffer) {
        if (err) throw new Error(err);

        var view = new DataView(buffer);
        var numBytes = buffer.byteLength - 4;
        var pointSize = GreyhoundUtils.pointSizeFrom(s);

        var colorNorm = false;
        var intensityNorm = false;

        for (var offset = 0; offset < numBytes; offset += pointSize) {
          if (
            view.getUint16(offset + 12, true) > 255 ||
            view.getUint16(offset + 14, true) > 255 ||
            view.getUint16(offset + 16, true) > 255
          ) {
            colorNorm = true;
          }

          if (view.getUint16(offset + 18, true) > 255) {
            intensityNorm = true;
          }

          if (colorNorm && intensityNorm) break;
        }

        cb(null, {
          color: colorNorm,
          intensity: intensityNorm,
        });
      });
    }
  }

  function GreyhoundLoader() {}

  GreyhoundLoader.loadInfoJSON = function(url, callback) {};

  /**
   * @return a point cloud octree with the root node data loaded.
   * loading of descendants happens asynchronously when they"re needed
   *
   * @param url
   * @param loadingFinishedListener executed after loading the binary has been
   * finished
   */
  GreyhoundLoader.load = function(url, callback) {
    var HIERARCHY_STEP_SIZE = 5;

    try {
      //We assume everything ater the string "greyhound://" is the server url
      var serverURL = url.split('greyhound://')[1];
      if (
        serverURL.split('http://').length === 1 &&
        serverURL.split('https://').length === 1
      ) {
        serverURL = 'http://' + serverURL;
      }

      GreyhoundUtils.fetch(serverURL + 'info', function(err, data) {
        if (err) throw new Error(err);

        /* We parse the result of the info query, which should be a JSON datastructure somewhat like:
        {
          "bounds": [635577, 848882, -1000, 639004, 853538, 2000],
          "numPoints": 10653336,
          "schema": [
              { "name": "X", "size": 8, "type": "floating" },
              { "name": "Y", "size": 8, "type": "floating" },
              { "name": "Z", "size": 8, "type": "floating" },
              { "name": "Intensity", "size": 2, "type": "unsigned" },
              { "name": "OriginId", "size": 4, "type": "unsigned" },
              { "name": "Red", "size": 2, "type": "unsigned" },
              { "name": "Green", "size": 2, "type": "unsigned" },
              { "name": "Blue", "size": 2, "type": "unsigned" }
          ],
          "srs": "<omitted for brevity>",
          "type": "octree"
        }
        */
        var greyhoundInfo = JSON.parse(data);
        var version = new VersionUtils('1.4');

        var bounds = greyhoundInfo.bounds;

        //TODO Unused: var boundsConforming = greyhoundInfo.boundsConforming;
        //TODO Unused: var width = bounds[3] - bounds[0];
        //TODO Unused: var depth = bounds[4] - bounds[1];
        //TODO Unused: var height = bounds[5] - bounds[2];
        //TODO Unused: var radius = width / 2;

        var scale = greyhoundInfo.scale || 0.01;
        if (Array.isArray(scale)) {
          scale = Math.min(scale[0], scale[1], scale[2]);
        }

        if (GreyhoundUtils.getQueryParam('scale')) {
          scale = parseFloat(GreyhoundUtils.getQueryParam('scale'));
        }

        var baseDepth = Math.max(8, greyhoundInfo.baseDepth);

        //Ideally we want to change this bit completely, since
        //greyhound"s options are wider than the default options for
        //visualizing pointclouds. If someone ever has time to build a
        //custom ui element for greyhound, the schema options from
        //this info request should be given to the UI, so the user can
        //choose between them. The selected option can then be
        //directly requested from the server in the
        //PointCloudGreyhoundGeometryNode without asking for
        //attributes that we are not currently visualizing.  We assume
        //XYZ are always available.
        var attributes = ['POSITION_CARTESIAN'];

        //To be careful, we only add COLOR_PACKED as an option if all
        //colors are actually found.
        var red = false;
        var green = false;
        var blue = false;

        greyhoundInfo.schema.forEach(function(entry) {
          //Intensity and Classification are optional.
          if (entry.name === 'Intensity') {
            attributes.push('INTENSITY');
          }
          if (entry.name === 'Classification') {
            attributes.push('CLASSIFICATION');
          }

          if (entry.name === 'Red') red = true;
          else if (entry.name === 'Green') green = true;
          else if (entry.name === 'Blue') blue = true;
        });

        if (red && green && blue) attributes.push('COLOR_PACKED');

        //Fill in geometry fields.
        var pgg = new PointCloudGreyhoundGeometry();
        pgg.serverURL = serverURL;
        pgg.spacing = (bounds[3] - bounds[0]) / Math.pow(2, baseDepth);
        pgg.baseDepth = baseDepth;
        pgg.hierarchyStepSize = HIERARCHY_STEP_SIZE;

        pgg.schema = GreyhoundUtils.createSchema(attributes);
        var pointSize = GreyhoundUtils.pointSizeFrom(pgg.schema);

        pgg.pointAttributes = new PointAttributes(attributes);
        pgg.pointAttributes.byteSize = pointSize;

        var boundingBox = new THREE.Box3(
          new THREE.Vector3().fromArray(bounds, 0),
          new THREE.Vector3().fromArray(bounds, 3)
        );

        var offset = boundingBox.min.clone();

        boundingBox.max.sub(boundingBox.min);
        boundingBox.min.set(0, 0, 0);

        pgg.projection = greyhoundInfo.srs;
        pgg.boundingBox = boundingBox;
        pgg.boundingSphere = boundingBox.getBoundingSphere(new THREE.Sphere());

        pgg.scale = scale;
        pgg.offset = offset;
        pgg.loader = new GreyhoundBinaryLoader(version, boundingBox, pgg.scale);

        var nodes = {};

        //load root
        var name = 'r';

        var root = new PointCloudGreyhoundGeometryNode(
          name,
          pgg,
          boundingBox,
          scale,
          offset
        );
        root.level = 0;
        root.hasChildren = true;
        root.numPoints = greyhoundInfo.numPoints;
        root.spacing = pgg.spacing;
        pgg.root = root;
        pgg.root.load();
        nodes[name] = root;

        pgg.nodes = nodes;

        GreyhoundUtils.getNormalization(
          serverURL,
          greyhoundInfo.baseDepth,
          function(_, normalize) {
            if (normalize.color) pgg.normalize.color = true;
            if (normalize.intensity) pgg.normalize.intensity = true;

            callback(pgg);
          }
        );
      });
    } catch (e) {
      console.log('Potree: Loading failed.', url, e);
      callback();
    }
  };

  GreyhoundLoader.loadPointAttributes = function(mno) {
    var fpa = mno.pointAttributes;
    var pa = new PointAttributes();

    for (var i = 0; i < fpa.length; i++) {
      var pointAttribute = PointAttribute[fpa[i]];
      pa.add(pointAttribute);
    }

    return pa;
  };

  GreyhoundLoader.createChildAABB = function(aabb, childIndex) {
    var min = aabb.min;
    var max = aabb.max;
    var dHalfLength = new THREE.Vector3()
      .copy(max)
      .sub(min)
      .multiplyScalar(0.5);
    var xHalfLength = new THREE.Vector3(dHalfLength.x, 0, 0);
    var yHalfLength = new THREE.Vector3(0, dHalfLength.y, 0);
    var zHalfLength = new THREE.Vector3(0, 0, dHalfLength.z);

    var cmin = min;
    var cmax = new THREE.Vector3().add(min).add(dHalfLength);

    if (childIndex === 1) {
      min = new THREE.Vector3().copy(cmin).add(zHalfLength);
      max = new THREE.Vector3().copy(cmax).add(zHalfLength);
    } else if (childIndex === 3) {
      min = new THREE.Vector3()
        .copy(cmin)
        .add(zHalfLength)
        .add(yHalfLength);
      max = new THREE.Vector3()
        .copy(cmax)
        .add(zHalfLength)
        .add(yHalfLength);
    } else if (childIndex === 0) {
      min = cmin;
      max = cmax;
    } else if (childIndex === 2) {
      min = new THREE.Vector3().copy(cmin).add(yHalfLength);
      max = new THREE.Vector3().copy(cmax).add(yHalfLength);
    } else if (childIndex === 5) {
      min = new THREE.Vector3()
        .copy(cmin)
        .add(zHalfLength)
        .add(xHalfLength);
      max = new THREE.Vector3()
        .copy(cmax)
        .add(zHalfLength)
        .add(xHalfLength);
    } else if (childIndex === 7) {
      min = new THREE.Vector3().copy(cmin).add(dHalfLength);
      max = new THREE.Vector3().copy(cmax).add(dHalfLength);
    } else if (childIndex === 4) {
      min = new THREE.Vector3().copy(cmin).add(xHalfLength);
      max = new THREE.Vector3().copy(cmax).add(xHalfLength);
    } else if (childIndex === 6) {
      min = new THREE.Vector3()
        .copy(cmin)
        .add(xHalfLength)
        .add(yHalfLength);
      max = new THREE.Vector3()
        .copy(cmax)
        .add(xHalfLength)
        .add(yHalfLength);
    }

    return new THREE.Box3(min, max);
  };

  class BinaryLoader {
    constructor(version, boundingBox, scale) {
      if (typeof version === 'string') {
        this.version = new VersionUtils(version);
      } else {
        this.version = version;
      }

      this.boundingBox = boundingBox;
      this.scale = scale;
    }

    load(node) {
      if (node.loaded) {
        return;
      }

      var url = node.getURL();

      if (this.version.equalOrHigher('1.4')) {
        url += '.bin';
      }

      var self = this;
      var xhr = new XMLHttpRequest();
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';
      xhr.overrideMimeType('text/plain; charset=x-user-defined');
      xhr.onreadystatechange = function() {
        if (xhr.readyState === 4) {
          if (
            (xhr.status === 200 || xhr.status === 0) &&
            xhr.response !== null
          ) {
            var buffer = xhr.response;
            self.parse(node, buffer);
          } else {
            throw new Error(
              'Potree: Failed to load file, HTTP status ' + xhr.status
            );
          }
        }
      };
      xhr.send(null);
    }

    parse(node, buffer) {
      var pointAttributes = node.pcoGeometry.pointAttributes;
      var numPoints =
        buffer.byteLength / node.pcoGeometry.pointAttributes.byteSize;

      if (this.version.upTo('1.5')) {
        node.numPoints = numPoints;
      }

      var message = {
        buffer: buffer,
        pointAttributes: pointAttributes,
        version: this.version.version,
        min: [
          node.boundingBox.min.x,
          node.boundingBox.min.y,
          node.boundingBox.min.z,
        ],
        offset: [
          node.pcoGeometry.offset.x,
          node.pcoGeometry.offset.y,
          node.pcoGeometry.offset.z,
        ],
        scale: this.scale,
        spacing: node.spacing,
        hasChildren: node.hasChildren,
        name: node.name,
      };

      Global.workerPool.runTask(
        WorkerManager.BINARY_DECODER,
        function(e) {
          var data = e.data;
          var buffers = data.attributeBuffers;
          var tightBoundingBox = new THREE.Box3(
            new THREE.Vector3().fromArray(data.tightBoundingBox.min),
            new THREE.Vector3().fromArray(data.tightBoundingBox.max)
          );

          var geometry = new THREE.BufferGeometry();

          for (var property in buffers) {
            var buffer = buffers[property].buffer;

            if (parseInt(property) === PointAttributeNames.POSITION_CARTESIAN) {
              geometry.addAttribute(
                'position',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (
              parseInt(property) === PointAttributeNames.COLOR_PACKED
            ) {
              geometry.addAttribute(
                'color',
                new THREE.BufferAttribute(new Uint8Array(buffer), 4, true)
              );
            } else if (parseInt(property) === PointAttributeNames.INTENSITY) {
              geometry.addAttribute(
                'intensity',
                new THREE.BufferAttribute(new Float32Array(buffer), 1)
              );
            } else if (
              parseInt(property) === PointAttributeNames.CLASSIFICATION
            ) {
              geometry.addAttribute(
                'classification',
                new THREE.BufferAttribute(new Uint8Array(buffer), 1)
              );
            } else if (
              parseInt(property) === PointAttributeNames.NORMAL_SPHEREMAPPED
            ) {
              geometry.addAttribute(
                'normal',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (
              parseInt(property) === PointAttributeNames.NORMAL_OCT16
            ) {
              geometry.addAttribute(
                'normal',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (parseInt(property) === PointAttributeNames.NORMAL) {
              geometry.addAttribute(
                'normal',
                new THREE.BufferAttribute(new Float32Array(buffer), 3)
              );
            } else if (parseInt(property) === PointAttributeNames.INDICES) {
              var bufferAttribute = new THREE.BufferAttribute(
                new Uint8Array(buffer),
                4
              );
              bufferAttribute.normalized = true;
              geometry.addAttribute('indices', bufferAttribute);
            } else if (parseInt(property) === PointAttributeNames.SPACING) {
              var bufferAttribute = new THREE.BufferAttribute(
                new Float32Array(buffer),
                1
              );
              geometry.addAttribute('spacing', bufferAttribute);
            }
          }

          tightBoundingBox.max.sub(tightBoundingBox.min);
          tightBoundingBox.min.set(0, 0, 0);

          var numPoints = e.data.buffer.byteLength / pointAttributes.byteSize;

          node.numPoints = numPoints;
          node.geometry = geometry;
          node.mean = new THREE.Vector3(...data.mean);
          node.tightBoundingBox = tightBoundingBox;
          node.loaded = true;
          node.loading = false;
          node.estimatedSpacing = data.estimatedSpacing;
          Global.numNodesLoading--;
        },
        message,
        [message.buffer]
      );
    }
  }

  var pointFormatReaders = [
    function(dv) {
      return {
        position: [
          dv.getInt32(0, true),
          dv.getInt32(4, true),
          dv.getInt32(8, true),
        ],
        intensity: dv.getUint16(12, true),
        classification: dv.getUint8(16, true),
      };
    },
    function(dv) {
      return {
        position: [
          dv.getInt32(0, true),
          dv.getInt32(4, true),
          dv.getInt32(8, true),
        ],
        intensity: dv.getUint16(12, true),
        classification: dv.getUint8(16, true),
      };
    },
    function(dv) {
      return {
        position: [
          dv.getInt32(0, true),
          dv.getInt32(4, true),
          dv.getInt32(8, true),
        ],
        intensity: dv.getUint16(12, true),
        classification: dv.getUint8(16, true),
        color: [
          dv.getUint16(20, true),
          dv.getUint16(22, true),
          dv.getUint16(24, true),
        ],
      };
    },
    function(dv) {
      return {
        position: [
          dv.getInt32(0, true),
          dv.getInt32(4, true),
          dv.getInt32(8, true),
        ],
        intensity: dv.getUint16(12, true),
        classification: dv.getUint8(16, true),
        color: [
          dv.getUint16(28, true),
          dv.getUint16(30, true),
          dv.getUint16(32, true),
        ],
      };
    },
  ];

  function readAs(buf, Type, offset, count) {
    count = count === undefined || count === 0 ? 1 : count;
    var sub = buf.slice(offset, offset + Type.BYTES_PER_ELEMENT * count);

    var r = new Type(sub);
    if (count === undefined || count === 1) {
      return r[0];
    }

    var ret = [];
    for (var i = 0; i < count; i++) {
      ret.push(r[i]);
    }

    return ret;
  }

  function parseLASHeader(arraybuffer) {
    var data = {};

    data.pointsOffset = readAs(arraybuffer, Uint32Array, 32 * 3);
    data.pointsFormatId = readAs(arraybuffer, Uint8Array, 32 * 3 + 8);
    data.pointsStructSize = readAs(arraybuffer, Uint16Array, 32 * 3 + 8 + 1);
    data.pointsCount = readAs(arraybuffer, Uint32Array, 32 * 3 + 11);

    var start = 32 * 3 + 35;
    data.scale = readAs(arraybuffer, Float64Array, start, 3);
    start += 24; // 8*3
    data.offset = readAs(arraybuffer, Float64Array, start, 3);
    start += 24;

    var bounds = readAs(arraybuffer, Float64Array, start, 6);
    start += 48; // 8*6;
    data.maxs = [bounds[0], bounds[2], bounds[4]];
    data.mins = [bounds[1], bounds[3], bounds[5]];

    return data;
  }

  // LAS Loader
  // Loads uncompressed files
  //
  function LASLoader(arraybuffer) {
    this.arraybuffer = arraybuffer;
  }
  LASLoader.prototype.open = function() {
    // nothing needs to be done to open this file
    //
    this.readOffset = 0;
    return new Promise(function(res, rej) {
      setTimeout(res, 0);
    });
  };

  LASLoader.prototype.getHeader = function() {
    var self = this;

    return new Promise(function(res, rej) {
      setTimeout(function() {
        self.header = parseLASHeader(self.arraybuffer);
        res(self.header);
      }, 0);
    });
  };

  LASLoader.prototype.readData = function(count, offset, skip) {
    var self = this;

    return new Promise(function(res, rej) {
      setTimeout(function() {
        if (!self.header)
          return rej(
            new Error(
              'Cannot start reading data till a header request is issued'
            )
          );

        var start;
        if (skip <= 1) {
          count = Math.min(count, self.header.pointsCount - self.readOffset);
          start =
            self.header.pointsOffset +
            self.readOffset * self.header.pointsStructSize;
          var end = start + count * self.header.pointsStructSize;
          res({
            buffer: self.arraybuffer.slice(start, end),
            count: count,
            hasMoreData: self.readOffset + count < self.header.pointsCount,
          });
          self.readOffset += count;
        } else {
          var pointsToRead = Math.min(
            count * skip,
            self.header.pointsCount - self.readOffset
          );
          var bufferSize = Math.ceil(pointsToRead / skip);
          var pointsRead = 0;

          var buf = new Uint8Array(bufferSize * self.header.pointsStructSize);

          for (var i = 0; i < pointsToRead; i++) {
            if (i % skip === 0) {
              start =
                self.header.pointsOffset +
                self.readOffset * self.header.pointsStructSize;
              var src = new Uint8Array(
                self.arraybuffer,
                start,
                self.header.pointsStructSize
              );

              buf.set(src, pointsRead * self.header.pointsStructSize);
              pointsRead++;
            }

            self.readOffset++;
          }

          res({
            buffer: buf.buffer,
            count: pointsRead,
            hasMoreData: self.readOffset < self.header.pointsCount,
          });
        }
      }, 0);
    });
  };

  LASLoader.prototype.close = function() {
    var self = this;
    return new Promise(function(res, rej) {
      self.arraybuffer = null;
      setTimeout(res, 0);
    });
  };

  // LAZ Loader
  // Uses NaCL module to load LAZ files
  //
  function LAZLoader(arraybuffer) {
    var self = this;

    this.arraybuffer = arraybuffer;
    this.nextCB = null;

    this.dorr = function(req, cb) {
      self.nextCB = cb;

      Global.workerPool.runTask(
        WorkerManager.LAS_LAZ,
        function(e) {
          if (self.nextCB !== null) {
            self.nextCB(e.data);
            self.nextCB = null;
          }
        },
        req
      );
    };
  }
  LAZLoader.prototype.open = function() {
    // nothing needs to be done to open this file
    var self = this;
    return new Promise(function(res, rej) {
      self.dorr({ type: 'open', arraybuffer: self.arraybuffer }, function(r) {
        if (r.status !== 1) {
          return rej(new Error('Failed to open file'));
        }

        res(true);
      });
    });
  };

  LAZLoader.prototype.getHeader = function() {
    var self = this;

    return new Promise(function(res, rej) {
      self.dorr({ type: 'header' }, function(r) {
        if (r.status !== 1) {
          return rej(new Error('Failed to get header'));
        }

        res(r.header);
      });
    });
  };

  LAZLoader.prototype.readData = function(count, offset, skip) {
    var self = this;

    return new Promise(function(res, rej) {
      self.dorr(
        { type: 'read', count: count, offset: offset, skip: skip },
        function(r) {
          if (r.status !== 1) return rej(new Error('Failed to read data'));
          res({
            buffer: r.buffer,
            count: r.count,
            hasMoreData: r.hasMoreData,
          });
        }
      );
    });
  };

  LAZLoader.prototype.close = function() {
    var self = this;

    return new Promise(function(res, rej) {
      self.dorr({ type: 'close' }, function(r) {
        if (r.status !== 1) {
          return rej(new Error('Failed to close file'));
        }

        res(true);
      });
    });
  };

  // A single consistent interface for loading LAS/LAZ files
  function LASFile(arraybuffer) {
    this.arraybuffer = arraybuffer;

    this.determineVersion();
    if (this.version > 12) {
      throw new Error('Only file versions <= 1.2 are supported at this time');
    }

    this.determineFormat();
    if (pointFormatReaders[this.formatId] === undefined) {
      throw new Error('The point format ID is not supported');
    }

    this.loader = this.isCompressed
      ? new LAZLoader(this.arraybuffer)
      : new LASLoader(this.arraybuffer);
  }
  LASFile.prototype.determineFormat = function() {
    var formatId = readAs(this.arraybuffer, Uint8Array, 32 * 3 + 8);
    var bit_7 = (formatId & 0x80) >> 7;
    var bit_6 = (formatId & 0x40) >> 6;

    if (bit_7 === 1 && bit_6 === 1) {
      throw new Error('Old style compression not supported');
    }

    this.formatId = formatId & 0x3f;
    this.isCompressed = bit_7 === 1 || bit_6 === 1;
  };

  LASFile.prototype.determineVersion = function() {
    var ver = new Int8Array(this.arraybuffer, 24, 2);
    this.version = ver[0] * 10 + ver[1];
    this.versionAsString = ver[0] + '.' + ver[1];
  };

  LASFile.prototype.open = function() {
    return this.loader.open();
  };

  LASFile.prototype.getHeader = function() {
    return this.loader.getHeader();
  };

  LASFile.prototype.readData = function(count, start, skip) {
    return this.loader.readData(count, start, skip);
  };

  LASFile.prototype.close = function() {
    return this.loader.close();
  };

  // Decodes LAS records into points
  function LASDecoder(
    buffer,
    pointFormatID,
    pointSize,
    pointsCount,
    scale,
    offset,
    mins,
    maxs
  ) {
    this.arrayb = buffer;
    this.decoder = pointFormatReaders[pointFormatID];
    this.pointsCount = pointsCount;
    this.pointSize = pointSize;
    this.scale = scale;
    this.offset = offset;
    this.mins = mins;
    this.maxs = maxs;
  }
  LASDecoder.prototype.getPoint = function(index) {
    if (index < 0 || index >= this.pointsCount) {
      throw new Error('Point index out of range');
    }

    return this.decoder(
      new DataView(this.arrayb, index * this.pointSize, this.pointSize)
    );
  };

  /**
   * laslaz code taken and adapted from plas.io js-laslaz
   *	http://plas.io/
   *  https://github.com/verma/plasio
   *
   * Thanks to Uday Verma and Howard Butler
   */
  class LASLAZLoader {
    constructor(version) {
      if (typeof version === 'string') {
        this.version = new VersionUtils(version);
      } else {
        this.version = version;
      }
    }

    load(node) {
      if (node.loaded) {
        return;
      }

      var pointAttributes = node.pcoGeometry.pointAttributes;
      var url = node.getURL();

      if (this.version.equalOrHigher('1.4')) {
        url += '.' + pointAttributes.toLowerCase();
      }

      var xhr = new XMLHttpRequest();
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';
      xhr.overrideMimeType('text/plain; charset=x-user-defined');
      xhr.onload = () => {
        if (xhr.response instanceof ArrayBuffer) {
          this.parse(node, xhr.response);
        } else {
          console.log(
            'Potree: LASLAZLoader xhr response is not a ArrayBuffer.'
          );
        }
      };
      xhr.onerror = function() {
        console.log(
          'Potree: LASLAZLoader failed to load file, ' +
            xhr.status +
            ', file: ' +
            url
        );
      };
      xhr.send(null);
    }

    parse(node, buffer) {
      var lf = new LASFile(buffer);
      var handler = new LASLAZBatcher(node);

      lf.open()
        .then(msg => {
          lf.isOpen = true;
          return lf;
        })
        .catch(msg => {
          console.log('Potree: Failed to open file.');
        })
        .then(lf => {
          return lf.getHeader().then(function(h) {
            return [lf, h];
          });
        })
        .then(v => {
          let lf = v[0];
          let header = v[1];
          let skip = 1;
          let totalRead = 0;
          let totalToRead = header.pointsCount;

          var reader = function() {
            let p = lf.readData(1000000, 0, skip);

            return p.then(function(data) {
              handler.push(
                new LASDecoder(
                  data.buffer,
                  header.pointsFormatId,
                  header.pointsStructSize,
                  data.count,
                  header.scale,
                  header.offset,
                  header.mins,
                  header.maxs
                )
              );

              totalRead += data.count;

              if (data.hasMoreData) {
                return reader();
              } else {
                header.totalRead = totalRead;
                header.versionAsString = lf.versionAsString;
                header.isCompressed = lf.isCompressed;
                return [lf, header, handler];
              }
            });
          };

          return reader();
        })
        .then(v => {
          let lf = v[0];

          //Close it
          return lf
            .close()
            .then(function() {
              lf.isOpen = false;
              return v.slice(1);
            })
            .catch(e => {
              //If there was a cancellation, make sure the file is closed, if the file is open close and then fail
              if (lf.isOpen) {
                return lf.close().then(function() {
                  lf.isOpen = false;
                  throw e;
                });
              }
              throw e;
            });
        });
    }

    handle(node, url) {}
  }
  class LASLAZBatcher {
    constructor(node) {
      this.node = node;
    }

    push(data) {
      var self = this;

      var message = {
        buffer: data.arrayb,
        numPoints: data.pointsCount,
        pointSize: data.pointSize,
        pointFormatID: 2,
        scale: data.scale,
        offset: data.offset,
        mins: data.mins,
        maxs: data.maxs,
      };

      var worker = Global.workerPool.getWorker(WorkerManager.LAS_DECODER);
      worker.onmessage = function(e) {
        var geometry = new THREE.BufferGeometry();
        var numPoints = data.pointsCount;

        var positions = new Float32Array(e.data.position);
        var colors = new Uint8Array(e.data.color);
        var intensities = new Float32Array(e.data.intensity);
        var classifications = new Uint8Array(e.data.classification);
        var returnNumbers = new Uint8Array(e.data.returnNumber);
        var numberOfReturns = new Uint8Array(e.data.numberOfReturns);
        var pointSourceIDs = new Uint16Array(e.data.pointSourceID);
        var indices = new Uint8Array(e.data.indices);

        geometry.addAttribute(
          'position',
          new THREE.BufferAttribute(positions, 3)
        );
        geometry.addAttribute(
          'color',
          new THREE.BufferAttribute(colors, 4, true)
        );
        geometry.addAttribute(
          'intensity',
          new THREE.BufferAttribute(intensities, 1)
        );
        geometry.addAttribute(
          'classification',
          new THREE.BufferAttribute(classifications, 1)
        );
        geometry.addAttribute(
          'returnNumber',
          new THREE.BufferAttribute(returnNumbers, 1)
        );
        geometry.addAttribute(
          'numberOfReturns',
          new THREE.BufferAttribute(numberOfReturns, 1)
        );
        geometry.addAttribute(
          'pointSourceID',
          new THREE.BufferAttribute(pointSourceIDs, 1)
        );
        //geometry.addAttribute("normal", new THREE.BufferAttribute(new Float32Array(numPoints * 3), 3));
        geometry.addAttribute('indices', new THREE.BufferAttribute(indices, 4));
        geometry.attributes.indices.normalized = true;

        var tightBoundingBox = new THREE.Box3(
          new THREE.Vector3().fromArray(e.data.tightBoundingBox.min),
          new THREE.Vector3().fromArray(e.data.tightBoundingBox.max)
        );

        geometry.boundingBox = self.node.boundingBox;
        self.node.tightBoundingBox = tightBoundingBox;

        self.node.geometry = geometry;
        self.node.numPoints = numPoints;
        self.node.loaded = true;
        self.node.loading = false;
        Global.numNodesLoading--;
        self.node.mean = new THREE.Vector3(...e.data.mean);

        Global.workerPool.returnWorker(WorkerManager.LAS_DECODER, worker);
      };

      worker.postMessage(message, [message.buffer]);
    }
  }

  class PointCloudOctreeGeometry {
    constructor() {
      this.url = null;
      this.octreeDir = null;
      this.spacing = 0;
      this.boundingBox = null;
      this.root = null;
      this.nodes = null;
      this.pointAttributes = null;
      this.hierarchyStepSize = -1;
      this.loader = null;
    }
  }
  class PointCloudOctreeGeometryNode extends PointCloudTreeNode {
    constructor(name, pcoGeometry, boundingBox) {
      super();

      this.id = PointCloudOctreeGeometryNode.IDCount++;
      this.name = name;
      this.index = parseInt(name.charAt(name.length - 1));
      this.pcoGeometry = pcoGeometry;
      this.geometry = null;
      this.boundingBox = boundingBox;
      this.boundingSphere = boundingBox.getBoundingSphere(new THREE.Sphere());
      this.children = {};
      this.numPoints = 0;
      this.level = null;
      this.loaded = false;
      this.oneTimeDisposeHandlers = [];
    }

    isGeometryNode() {
      return true;
    }

    getLevel() {
      return this.level;
    }

    isTreeNode() {
      return false;
    }

    isLoaded() {
      return this.loaded;
    }

    getBoundingSphere() {
      return this.boundingSphere;
    }

    getBoundingBox() {
      return this.boundingBox;
    }

    getChildren() {
      var children = [];

      for (var i = 0; i < 8; i++) {
        if (this.children[i]) {
          children.push(this.children[i]);
        }
      }

      return children;
    }

    getURL() {
      var url = '';
      var version = this.pcoGeometry.loader.version;

      if (version.equalOrHigher('1.5')) {
        url =
          this.pcoGeometry.octreeDir +
          '/' +
          this.getHierarchyPath() +
          '/' +
          this.name;
      } else if (version.equalOrHigher('1.4')) {
        url = this.pcoGeometry.octreeDir + '/' + this.name;
      } else if (version.upTo('1.3')) {
        url = this.pcoGeometry.octreeDir + '/' + this.name;
      }

      return url;
    }

    getHierarchyPath() {
      var path = 'r/';
      var hierarchyStepSize = this.pcoGeometry.hierarchyStepSize;
      var indices = this.name.substr(1);

      var numParts = Math.floor(indices.length / hierarchyStepSize);
      for (var i = 0; i < numParts; i++) {
        path += indices.substr(i * hierarchyStepSize, hierarchyStepSize) + '/';
      }

      path = path.slice(0, -1);

      return path;
    }

    addChild(child) {
      this.children[child.index] = child;
      child.parent = this;
    }

    load() {
      if (
        this.loading === true ||
        this.loaded === true ||
        Global.numNodesLoading >= Global.maxNodesLoading
      ) {
        return;
      }

      this.loading = true;
      Global.numNodesLoading++;

      if (this.pcoGeometry.loader.version.equalOrHigher('1.5')) {
        if (
          this.level % this.pcoGeometry.hierarchyStepSize === 0 &&
          this.hasChildren
        ) {
          this.loadHierachyThenPoints();
        } else {
          this.loadPoints();
        }
      } else {
        this.loadPoints();
      }
    }

    loadPoints() {
      this.pcoGeometry.loader.load(this);
    }

    loadHierachyThenPoints() {
      var node = this;

      var callback = function(node, hbuffer) {
        var view = new DataView(hbuffer);

        var stack = [];
        var children = view.getUint8(0);
        var numPoints = view.getUint32(1, true);
        node.numPoints = numPoints;
        stack.push({
          children: children,
          numPoints: numPoints,
          name: node.name,
        });

        var decoded = [];
        var offset = 5;

        while (stack.length > 0) {
          var snode = stack.shift();
          var mask = 1;
          for (var i = 0; i < 8; i++) {
            if ((snode.children & mask) !== 0) {
              var childName = snode.name + i;
              var childChildren = view.getUint8(offset);
              var childNumPoints = view.getUint32(offset + 1, true);

              stack.push({
                children: childChildren,
                numPoints: childNumPoints,
                name: childName,
              });
              decoded.push({
                children: childChildren,
                numPoints: childNumPoints,
                name: childName,
              });

              offset += 5;
            }

            mask = mask * 2;
          }

          if (offset === hbuffer.byteLength) {
            break;
          }
        }

        var nodes = {};
        nodes[node.name] = node;
        var pco = node.pcoGeometry;

        for (var i = 0; i < decoded.length; i++) {
          var name = decoded[i].name;
          var decodedNumPoints = decoded[i].numPoints;
          var index = parseInt(name.charAt(name.length - 1));
          var parentName = name.substring(0, name.length - 1);
          var parentNode = nodes[parentName];
          var level = name.length - 1;
          var boundingBox = POCLoader.createChildAABB(
            parentNode.boundingBox,
            index
          );

          var currentNode = new PointCloudOctreeGeometryNode(
            name,
            pco,
            boundingBox
          );
          currentNode.level = level;
          currentNode.numPoints = decodedNumPoints;
          currentNode.hasChildren = decoded[i].children > 0;
          currentNode.spacing = pco.spacing / Math.pow(2, level);
          parentNode.addChild(currentNode);
          nodes[name] = currentNode;
        }

        node.loadPoints();
      };

      if (node.level % node.pcoGeometry.hierarchyStepSize === 0) {
        var hurl =
          node.pcoGeometry.octreeDir +
          '/' +
          node.getHierarchyPath() +
          '/' +
          node.name +
          '.hrc';
        var xhr = new XMLHttpRequest();
        xhr.open('GET', hurl, true);
        xhr.responseType = 'arraybuffer';
        xhr.overrideMimeType('text/plain; charset=x-user-defined');
        xhr.onload = function(event) {
          callback(node, xhr.response);
        };
        xhr.onerror = function(event) {
          console.log(
            'Potree: Failed to load file! HTTP status: ' +
              xhr.status +
              ', file: ' +
              hurl,
            event
          );
          Global.numNodesLoading--;
        };
        xhr.send(null);
      }
    }

    getNumPoints() {
      return this.numPoints;
    }

    dispose() {
      if (this.geometry && this.parent != null) {
        this.geometry.dispose();
        this.geometry = null;
        this.loaded = false;

        for (var i = 0; i < this.oneTimeDisposeHandlers.length; i++) {
          var handler = this.oneTimeDisposeHandlers[i];
          handler();
        }
        this.oneTimeDisposeHandlers = [];
      }
    }
  }

  PointCloudOctreeGeometryNode.IDCount = 0;

  Object.assign(
    PointCloudOctreeGeometryNode.prototype,
    THREE.EventDispatcher.prototype
  );

  /**
   * @class Loads mno files and returns a PointcloudOctree
   * for a description of the mno binary file format, read mnoFileFormat.txt
   *
   * @author Markus Schuetz
   */
  class POCLoader {
    /**
     * @return a point cloud octree with the root node data loaded.
     * loading of descendants happens asynchronously when they"re needed
     *
     * @param url
     * @param loadingFinishedListener executed after loading the binary has been finished
     */
    static load(url, callback) {
      var pco = new PointCloudOctreeGeometry();
      pco.url = url;

      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', url, true);
      xhr.onload = function() {
        var data = JSON.parse(xhr.responseText);
        var version = new VersionUtils(data.version);

        //Assume dir as absolute if it starts with http
        if (data.octreeDir.indexOf('http') === 0) {
          pco.octreeDir = data.octreeDir;
        } else {
          pco.octreeDir = url + '/../' + data.octreeDir;
        }

        pco.spacing = data.spacing;
        pco.hierarchyStepSize = data.hierarchyStepSize;
        pco.pointAttributes = data.pointAttributes;

        var min = new THREE.Vector3(
          data.boundingBox.lx,
          data.boundingBox.ly,
          data.boundingBox.lz
        );
        var max = new THREE.Vector3(
          data.boundingBox.ux,
          data.boundingBox.uy,
          data.boundingBox.uz
        );
        var boundingBox = new THREE.Box3(min, max);
        var tightBoundingBox = boundingBox.clone();

        if (data.tightBoundingBox) {
          tightBoundingBox.min.copy(
            new THREE.Vector3(
              data.tightBoundingBox.lx,
              data.tightBoundingBox.ly,
              data.tightBoundingBox.lz
            )
          );
          tightBoundingBox.max.copy(
            new THREE.Vector3(
              data.tightBoundingBox.ux,
              data.tightBoundingBox.uy,
              data.tightBoundingBox.uz
            )
          );
        }

        var offset = min.clone();

        boundingBox.min.sub(offset);
        boundingBox.max.sub(offset);

        tightBoundingBox.min.sub(offset);
        tightBoundingBox.max.sub(offset);

        pco.projection = data.projection;
        pco.boundingBox = boundingBox;
        pco.tightBoundingBox = tightBoundingBox;
        pco.boundingSphere = boundingBox.getBoundingSphere(new THREE.Sphere());
        pco.tightBoundingSphere = tightBoundingBox.getBoundingSphere(
          new THREE.Sphere()
        );
        pco.offset = offset;

        //Select the appropiate loader
        if (data.pointAttributes === 'LAS' || data.pointAttributes === 'LAZ') {
          pco.loader = new LASLAZLoader(data.version);
        } else {
          pco.loader = new BinaryLoader(data.version, boundingBox, data.scale);
          pco.pointAttributes = new PointAttributes(pco.pointAttributes);
        }

        var nodes = {};
        var name = 'r';

        var root = new PointCloudOctreeGeometryNode(name, pco, boundingBox);
        root.level = 0;
        root.hasChildren = true;
        root.spacing = pco.spacing;
        root.numPoints = version.upTo('1.5') ? data.hierarchy[0][1] : 0;

        pco.root = root;
        pco.root.load();
        nodes[name] = root;

        //Load remaining hierarchy
        if (version.upTo('1.4')) {
          for (var i = 1; i < data.hierarchy.length; i++) {
            var name = data.hierarchy[i][0];
            var numPoints = data.hierarchy[i][1];
            var index = parseInt(name.charAt(name.length - 1));
            var parentName = name.substring(0, name.length - 1);
            var parentNode = nodes[parentName];
            var level = name.length - 1;
            var boundingBox = POCLoader.createChildAABB(
              parentNode.boundingBox,
              index
            );

            var node = new PointCloudOctreeGeometryNode(name, pco, boundingBox);
            node.level = level;
            node.numPoints = numPoints;
            node.spacing = pco.spacing / Math.pow(2, level);
            parentNode.addChild(node);
            nodes[name] = node;
          }
        }
        pco.nodes = nodes;

        callback(pco);
      };

      xhr.onerror = function(event) {
        console.log('Potree: loading failed: "' + url + '"', event);
        callback();
      };

      xhr.send(null);
    }

    static loadPointAttributes(mno) {
      var fpa = mno.pointAttributes;
      var pa = new PointAttributes();

      for (var i = 0; i < fpa.length; i++) {
        pa.add(PointAttribute[fpa[i]]);
      }

      return pa;
    }

    static createChildAABB(aabb, index) {
      var min = aabb.min.clone();
      var max = aabb.max.clone();
      var size = new THREE.Vector3().subVectors(max, min);

      if ((index & 0b0001) > 0) {
        min.z += size.z / 2;
      } else {
        max.z -= size.z / 2;
      }

      if ((index & 0b0010) > 0) {
        min.y += size.y / 2;
      } else {
        max.y -= size.y / 2;
      }

      if ((index & 0b0100) > 0) {
        min.x += size.x / 2;
      } else {
        max.x -= size.x / 2;
      }

      return new THREE.Box3(min, max);
    }
  }

  class EptBinaryLoader {
    load(node) {
      if (node.loaded) return;

      var url = node.url() + '.bin';

      var xhr = new XMLHttpRequest();
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';
      xhr.overrideMimeType('text/plain; charset=x-user-defined');
      xhr.onreadystatechange = () => {
        if (xhr.readyState === 4) {
          if (xhr.status === 200) {
            var buffer = xhr.response;
            this.parse(node, buffer);
          } else {
            console.log('Failed ' + url + ': ' + xhr.status);
          }
        }
      };

      try {
        xhr.send(null);
      } catch (e) {
        console.log('Failed request: ' + e);
      }
    }

    parse(node, buffer) {
      var worker = Global.workerPool.getWorker(
        WorkerManager.EPT_BINARY_DECODER
      );

      worker.onmessage = function(e) {
        var g = new THREE.BufferGeometry();
        var numPoints = e.data.numPoints;

        var position = new Float32Array(e.data.position);
        g.addAttribute('position', new THREE.BufferAttribute(position, 3));

        var indices = new Uint8Array(e.data.indices);
        g.addAttribute('indices', new THREE.BufferAttribute(indices, 4));

        if (e.data.color) {
          var color = new Uint8Array(e.data.color);
          g.addAttribute('color', new THREE.BufferAttribute(color, 4, true));
        }
        if (e.data.intensity) {
          var intensity = new Float32Array(e.data.intensity);
          g.addAttribute('intensity', new THREE.BufferAttribute(intensity, 1));
        }
        if (e.data.classification) {
          var classification = new Uint8Array(e.data.classification);
          g.addAttribute(
            'classification',
            new THREE.BufferAttribute(classification, 1)
          );
        }
        if (e.data.returnNumber) {
          var returnNumber = new Uint8Array(e.data.returnNumber);
          g.addAttribute(
            'returnNumber',
            new THREE.BufferAttribute(returnNumber, 1)
          );
        }
        if (e.data.numberOfReturns) {
          var numberOfReturns = new Uint8Array(e.data.numberOfReturns);
          g.addAttribute(
            'numberOfReturns',
            new THREE.BufferAttribute(numberOfReturns, 1)
          );
        }
        if (e.data.pointSourceId) {
          var pointSourceId = new Uint16Array(e.data.pointSourceId);
          g.addAttribute(
            'pointSourceID',
            new THREE.BufferAttribute(pointSourceId, 1)
          );
        }

        g.attributes.indices.normalized = true;

        var tightBoundingBox = new THREE.Box3(
          new THREE.Vector3().fromArray(e.data.tightBoundingBox.min),
          new THREE.Vector3().fromArray(e.data.tightBoundingBox.max)
        );

        node.doneLoading(
          g,
          tightBoundingBox,
          numPoints,
          new THREE.Vector3(...e.data.mean)
        );

        Global.workerPool.returnWorker(
          WorkerManager.EPT_BINARY_DECODER,
          worker
        );
      };

      var toArray = v => [v.x, v.y, v.z];
      var message = {
        buffer: buffer,
        schema: node.ept.schema,
        scale: node.ept.eptScale,
        offset: node.ept.eptOffset,
        mins: toArray(node.key.b.min),
      };

      worker.postMessage(message, [message.buffer]);
    }
  }

  /**
   * laslaz code taken and adapted from plas.io js-laslaz
   *	http://plas.io/
   *	https://github.com/verma/plasio
   *
   * Thanks to Uday Verma and Howard Butler
   *
   */
  class EptLaszipLoader {
    load(node) {
      if (node.loaded) {
        return;
      }

      var url = node.url() + '.laz';

      var xhr = new XMLHttpRequest();
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';
      xhr.overrideMimeType('text/plain; charset=x-user-defined');
      xhr.onreadystatechange = () => {
        if (xhr.readyState === 4) {
          if (xhr.status === 200) {
            var buffer = xhr.response;
            this.parse(node, buffer);
          } else {
            console.log('Failed ' + url + ': ' + xhr.status);
          }
        }
      };

      xhr.send(null);
    }

    parse(node, buffer) {
      var lf = new LASFile(buffer);
      var handler = new EptLazBatcher(node);

      lf.open()
        .then(() => {
          lf.isOpen = true;
          return lf.getHeader();
        })
        .then(header => {
          var i = 0;
          var np = header.pointsCount;

          var toArray = v => [v.x, v.y, v.z];
          var mins = toArray(node.key.b.min);
          var maxs = toArray(node.key.b.max);

          var read = () => {
            var p = lf.readData(1000000, 0, 1);
            return p.then(function(data) {
              var d = new LASDecoder(
                data.buffer,
                header.pointsFormatId,
                header.pointsStructSize,
                data.count,
                header.scale,
                header.offset,
                mins,
                maxs
              );
              d.extraBytes = header.extraBytes;
              d.pointsFormatId = header.pointsFormatId;
              handler.push(d);

              i += data.count;

              if (data.hasMoreData) {
                return read();
              } else {
                header.totalRead = i;
                header.versionAsString = lf.versionAsString;
                header.isCompressed = lf.isCompressed;
                return null;
              }
            });
          };

          return read();
        })
        .then(() => lf.close())
        .then(() => (lf.isOpen = false))
        .catch(err => {
          console.log('Error reading LAZ:', err);
          if (lf.isOpen) {
            lf.close().then(() => {
              lf.isOpen = false;
              throw err;
            });
          } else throw err;
        });
    }
  }
  class EptLazBatcher {
    constructor(node) {
      this.node = node;
    }

    push(las) {
      var worker = Global.workerPool.getWorker(
        WorkerManager.EPT_LAS_ZIP_DECODER
      );

      worker.onmessage = e => {
        var g = new THREE.BufferGeometry();
        var numPoints = las.pointsCount;

        var positions = new Float32Array(e.data.position);
        var colors = new Uint8Array(e.data.color);

        var intensities = new Float32Array(e.data.intensity);
        var classifications = new Uint8Array(e.data.classification);
        var returnNumbers = new Uint8Array(e.data.returnNumber);
        var numberOfReturns = new Uint8Array(e.data.numberOfReturns);
        var pointSourceIDs = new Uint16Array(e.data.pointSourceID);
        var indices = new Uint8Array(e.data.indices);

        g.addAttribute('position', new THREE.BufferAttribute(positions, 3));
        g.addAttribute('color', new THREE.BufferAttribute(colors, 4, true));
        g.addAttribute('intensity', new THREE.BufferAttribute(intensities, 1));
        g.addAttribute(
          'classification',
          new THREE.BufferAttribute(classifications, 1)
        );
        g.addAttribute(
          'returnNumber',
          new THREE.BufferAttribute(returnNumbers, 1)
        );
        g.addAttribute(
          'numberOfReturns',
          new THREE.BufferAttribute(numberOfReturns, 1)
        );
        g.addAttribute(
          'pointSourceID',
          new THREE.BufferAttribute(pointSourceIDs, 1)
        );
        g.addAttribute('indices', new THREE.BufferAttribute(indices, 4));
        g.attributes.indices.normalized = true;

        var tightBoundingBox = new THREE.Box3(
          new THREE.Vector3().fromArray(e.data.tightBoundingBox.min),
          new THREE.Vector3().fromArray(e.data.tightBoundingBox.max)
        );

        this.node.doneLoading(
          g,
          tightBoundingBox,
          numPoints,
          new THREE.Vector3(...e.data.mean)
        );

        Global.workerPool.returnWorker(
          WorkerManager.EPT_LAS_ZIP_DECODER,
          worker
        );
      };

      var message = {
        buffer: las.arrayb,
        numPoints: las.pointsCount,
        pointSize: las.pointSize,
        pointFormatID: las.pointsFormatId,
        scale: las.scale,
        offset: las.offset,
        mins: las.mins,
        maxs: las.maxs,
      };

      worker.postMessage(message, [message.buffer]);
    }
  }

  class U {
    static toVector3(v, offset) {
      return new THREE.Vector3().fromArray(v, offset || 0);
    }

    static toBox3(b) {
      return new THREE.Box3(U.toVector3(b), U.toVector3(b, 3));
    }

    static findDim(schema, name) {
      var dim = schema.find(dim => dim.name == name);
      if (!dim) throw new Error('Failed to find ' + name + ' in schema');
      return dim;
    }

    static sphereFrom(b) {
      return b.getBoundingSphere(new THREE.Sphere());
    }
  }
  class PointCloudEptGeometry {
    constructor(url, info) {
      let version = info.version;
      let schema = info.schema;
      let bounds = info.bounds;
      let boundsConforming = info.boundsConforming;

      let xyz = [
        U.findDim(schema, 'X'),
        U.findDim(schema, 'Y'),
        U.findDim(schema, 'Z'),
      ];
      let scale = xyz.map(d => d.scale || 1);
      let offset = xyz.map(d => d.offset || 0);

      this.eptScale = U.toVector3(scale);
      this.eptOffset = U.toVector3(offset);

      this.url = url;
      this.info = info;
      this.type = 'ept';

      this.schema = schema;
      this.span = info.span || info.ticks;
      this.boundingBox = U.toBox3(bounds);
      this.tightBoundingBox = U.toBox3(boundsConforming);
      this.offset = U.toVector3([0, 0, 0]);
      this.boundingSphere = U.sphereFrom(this.boundingBox);
      this.tightBoundingSphere = U.sphereFrom(this.tightBoundingBox);
      this.version = new VersionUtils('1.6');

      this.projection = null;
      this.fallbackProjection = null;

      if (info.srs && info.srs.horizontal) {
        this.projection = info.srs.authority + ':' + info.srs.horizontal;
      }

      if (info.srs.wkt) {
        if (!this.projection) this.projection = info.srs.wkt;
        else this.fallbackProjection = info.srs.wkt;
      }

      this.pointAttributes = 'LAZ';
      this.spacing =
        (this.boundingBox.max.x - this.boundingBox.min.x) / this.span;

      let hierarchyType = info.hierarchyType || 'json';

      let dataType = info.dataType || 'laszip';
      this.loader =
        dataType == 'binary' ? new EptBinaryLoader() : new EptLaszipLoader();
    }
  }
  class EptKey {
    constructor(ept, b, d, x, y, z) {
      this.ept = ept;
      this.b = b;
      this.d = d;
      this.x = x || 0;
      this.y = y || 0;
      this.z = z || 0;
    }

    name() {
      return this.d + '-' + this.x + '-' + this.y + '-' + this.z;
    }

    step(a, b, c) {
      let min = this.b.min.clone();
      let max = this.b.max.clone();
      let dst = new THREE.Vector3().subVectors(max, min);

      if (a) min.x += dst.x / 2;
      else max.x -= dst.x / 2;

      if (b) min.y += dst.y / 2;
      else max.y -= dst.y / 2;

      if (c) min.z += dst.z / 2;
      else max.z -= dst.z / 2;

      return new EptKey(
        this.ept,
        new THREE.Box3(min, max),
        this.d + 1,
        this.x * 2 + a,
        this.y * 2 + b,
        this.z * 2 + c
      );
    }

    children() {
      var result = [];
      for (var a = 0; a < 2; ++a) {
        for (var b = 0; b < 2; ++b) {
          for (var c = 0; c < 2; ++c) {
            var add = this.step(a, b, c).name();
            if (!result.includes(add)) result = result.concat(add);
          }
        }
      }
      return result;
    }
  }

  class PointCloudEptGeometryNode extends PointCloudTreeNode {
    constructor(ept, b, d, x, y, z) {
      super();

      this.ept = ept;
      this.key = new EptKey(
        this.ept,
        b || this.ept.boundingBox,
        d || 0,
        x,
        y,
        z
      );

      this.id = PointCloudEptGeometryNode.IDCount++;
      this.geometry = null;
      this.boundingBox = this.key.b;
      this.tightBoundingBox = this.boundingBox;
      this.spacing = this.ept.spacing / Math.pow(2, this.key.d);
      this.boundingSphere = U.sphereFrom(this.boundingBox);

      // These are set during hierarchy loading.
      this.hasChildren = false;
      this.children = {};
      this.numPoints = -1;

      this.level = this.key.d;
      this.loaded = false;
      this.loading = false;
      this.oneTimeDisposeHandlers = [];

      let k = this.key;
      this.name = this.toPotreeName(k.d, k.x, k.y, k.z);
      this.index = parseInt(this.name.charAt(this.name.length - 1));
    }

    isGeometryNode() {
      return true;
    }
    getLevel() {
      return this.level;
    }
    isTreeNode() {
      return false;
    }
    isLoaded() {
      return this.loaded;
    }
    getBoundingSphere() {
      return this.boundingSphere;
    }
    getBoundingBox() {
      return this.boundingBox;
    }
    url() {
      return this.ept.url + 'ept-data/' + this.filename();
    }
    getNumPoints() {
      return this.numPoints;
    }
    filename() {
      return this.key.name();
    }

    getChildren() {
      let children = [];

      for (let i = 0; i < 8; i++) {
        if (this.children[i]) {
          children.push(this.children[i]);
        }
      }

      return children;
    }

    addChild(child) {
      this.children[child.index] = child;
      child.parent = this;
    }

    load() {
      if (this.loaded || this.loading) return;
      if (Global.numNodesLoading >= Global.maxNodesLoading) return;

      this.loading = true;
      ++Global.numNodesLoading;

      if (this.numPoints == -1) this.loadHierarchy();
      this.loadPoints();
    }

    loadPoints() {
      this.ept.loader.load(this);
    }

    async loadHierarchy() {
      let nodes = {};
      nodes[this.filename()] = this;
      this.hasChildren = false;

      let eptHierarchyFile = `${
        this.ept.url
      }ept-hierarchy/${this.filename()}.json`;

      let response = await fetch(eptHierarchyFile);
      let hier = await response.json();

      // Since we want to traverse top-down, and 10 comes
      // lexicographically before 9 (for example), do a deep sort.
      var keys = Object.keys(hier).sort((a, b) => {
        let [da, xa, ya, za] = a.split('-').map(n => parseInt(n, 10));
        let [db, xb, yb, zb] = b.split('-').map(n => parseInt(n, 10));
        if (da < db) return -1;
        if (da > db) return 1;
        if (xa < xb) return -1;
        if (xa > xb) return 1;
        if (ya < yb) return -1;
        if (ya > yb) return 1;
        if (za < zb) return -1;
        if (za > zb) return 1;
        return 0;
      });

      keys.forEach(v => {
        let [d, x, y, z] = v.split('-').map(n => parseInt(n, 10));
        let a = x & 1,
          b = y & 1,
          c = z & 1;
        let parentName =
          d - 1 + '-' + (x >> 1) + '-' + (y >> 1) + '-' + (z >> 1);

        let parentNode = nodes[parentName];
        if (!parentNode) return;
        parentNode.hasChildren = true;

        let key = parentNode.key.step(a, b, c);

        let node = new PointCloudEptGeometryNode(
          this.ept,
          key.b,
          key.d,
          key.x,
          key.y,
          key.z
        );

        node.level = d;
        node.numPoints = hier[v];

        parentNode.addChild(node);
        nodes[key.name()] = node;
      });
    }

    doneLoading(bufferGeometry, tightBoundingBox, np, mean) {
      bufferGeometry.boundingBox = this.boundingBox;
      this.geometry = bufferGeometry;
      this.tightBoundingBox = tightBoundingBox;
      this.numPoints = np;
      this.mean = mean;
      this.loaded = true;
      this.loading = false;
      --Global.numNodesLoading;
    }

    toPotreeName(d, x, y, z) {
      var name = 'r';

      for (var i = 0; i < d; ++i) {
        var shift = d - i - 1;
        var mask = 1 << shift;
        var step = 0;

        if (x & mask) step += 4;
        if (y & mask) step += 2;
        if (z & mask) step += 1;

        name += step;
      }

      return name;
    }

    dispose() {
      if (this.geometry && this.parent != null) {
        this.geometry.dispose();
        this.geometry = null;
        this.loaded = false;

        // this.dispatchEvent( { type: "dispose" } );
        for (let i = 0; i < this.oneTimeDisposeHandlers.length; i++) {
          let handler = this.oneTimeDisposeHandlers[i];
          handler();
        }

        this.oneTimeDisposeHandlers = [];
      }
    }
  }

  PointCloudEptGeometryNode.IDCount = 0;

  /**
   * @author Connor Manning
   */
  class EptLoader {
    static async load(file, callback) {
      var response = await fetch(file);
      var json = await response.json();
      var url = file.substr(0, file.lastIndexOf('ept.json'));

      var geometry = new PointCloudEptGeometry(url, json);
      var root = new PointCloudEptGeometryNode(geometry);
      geometry.root = root;
      geometry.root.load();

      callback(geometry);
    }
  }

  class HelperUtils {
    /**
     * Craete a new data texture with a solid color.
     */
    static generateDataTexture(width, height, color) {
      var size = width * height;
      var data = new Uint8Array(4 * width * height);

      var r = Math.floor(color.r * 255);
      var g = Math.floor(color.g * 255);
      var b = Math.floor(color.b * 255);

      for (var i = 0; i < size; i++) {
        data[i * 3] = r;
        data[i * 3 + 1] = g;
        data[i * 3 + 2] = b;
      }

      var texture = new THREE.DataTexture(
        data,
        width,
        height,
        THREE.RGBAFormat
      );
      texture.needsUpdate = true;
      texture.magFilter = THREE.NearestFilter;

      return texture;
    }

    /**
     * Compute a transformed bouding box from an original box and a transform matrix.
     */
    static computeTransformedBoundingBox(box, transform) {
      var vertices = [
        new THREE.Vector3(box.min.x, box.min.y, box.min.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.min.x, box.min.y, box.min.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.max.x, box.min.y, box.min.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.min.x, box.max.y, box.min.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.min.x, box.min.y, box.max.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.min.x, box.max.y, box.max.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.max.x, box.max.y, box.min.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.max.x, box.min.y, box.max.z).applyMatrix4(
          transform
        ),
        new THREE.Vector3(box.max.x, box.max.y, box.max.z).applyMatrix4(
          transform
        ),
      ];

      var boundingBox = new THREE.Box3();
      boundingBox.setFromPoints(vertices);

      return boundingBox;
    }
  }

  //
  //to get a ready to use gradient array from a chroma.js gradient:
  //http://gka.github.io/chroma.js/
  //
  //var stops = [];
  //for(var i = 0; i <= 10; i++){
  //	var range = chroma.scale(["yellow", "navy"]).mode("lch").domain([10,0])(i)._rgb
  //		.slice(0, 3)
  //		.map(v => (v / 255).toFixed(4))
  //		.join(", ");
  //
  //	var line = `[${i / 10}, new THREE.Color(${range})],`;
  //
  //	stops.push(line);
  //}
  //stops.join("\n");

  //to get a ready to use gradient array from matplotlib:
  //import matplotlib.pyplot as plt
  //import matplotlib.colors as colors
  //
  //norm = colors.Normalize(vmin=0,vmax=1)
  //cmap = plt.cm.viridis
  //
  //for i in range(0,11):
  //   u = i / 10
  //   rgb = cmap(norm(u))[0:3]
  //   rgb = ["{0:.3f}".format(v) for v in rgb]
  //   rgb = "[" + str(u) + ", new THREE.Color(" +  ", ".join(rgb) + ")],"
  //   print(rgb)

  var Gradients = {
    RAINBOW: [
      [0, new THREE.Color(0.278, 0, 0.714)],
      [1 / 6, new THREE.Color(0, 0, 1)],
      [2 / 6, new THREE.Color(0, 1, 1)],
      [3 / 6, new THREE.Color(0, 1, 0)],
      [4 / 6, new THREE.Color(1, 1, 0)],
      [5 / 6, new THREE.Color(1, 0.64, 0)],
      [1, new THREE.Color(1, 0, 0)],
    ],
    //From chroma spectral http://gka.github.io/chroma.js/
    SPECTRAL: [
      [0, new THREE.Color(0.3686, 0.3098, 0.6353)],
      [0.1, new THREE.Color(0.1961, 0.5333, 0.7412)],
      [0.2, new THREE.Color(0.4, 0.7608, 0.6471)],
      [0.3, new THREE.Color(0.6706, 0.8667, 0.6431)],
      [0.4, new THREE.Color(0.902, 0.9608, 0.5961)],
      [0.5, new THREE.Color(1.0, 1.0, 0.749)],
      [0.6, new THREE.Color(0.9961, 0.8784, 0.5451)],
      [0.7, new THREE.Color(0.9922, 0.6824, 0.3804)],
      [0.8, new THREE.Color(0.9569, 0.4275, 0.2627)],
      [0.9, new THREE.Color(0.8353, 0.2431, 0.3098)],
      [1, new THREE.Color(0.6196, 0.0039, 0.2588)],
    ],
    PLASMA: [
      [0.0, new THREE.Color(0.241, 0.015, 0.61)],
      [0.1, new THREE.Color(0.387, 0.001, 0.654)],
      [0.2, new THREE.Color(0.524, 0.025, 0.653)],
      [0.3, new THREE.Color(0.651, 0.125, 0.596)],
      [0.4, new THREE.Color(0.752, 0.227, 0.513)],
      [0.5, new THREE.Color(0.837, 0.329, 0.431)],
      [0.6, new THREE.Color(0.907, 0.435, 0.353)],
      [0.7, new THREE.Color(0.963, 0.554, 0.272)],
      [0.8, new THREE.Color(0.992, 0.681, 0.195)],
      [0.9, new THREE.Color(0.987, 0.822, 0.144)],
      [1.0, new THREE.Color(0.94, 0.975, 0.131)],
    ],
    YELLOW_GREEN: [
      [0, new THREE.Color(0.1647, 0.2824, 0.3451)],
      [0.1, new THREE.Color(0.1338, 0.3555, 0.4227)],
      [0.2, new THREE.Color(0.061, 0.4319, 0.4864)],
      [0.3, new THREE.Color(0.0, 0.5099, 0.5319)],
      [0.4, new THREE.Color(0.0, 0.5881, 0.5569)],
      [0.5, new THREE.Color(0.137, 0.665, 0.5614)],
      [0.6, new THREE.Color(0.2906, 0.7395, 0.5477)],
      [0.7, new THREE.Color(0.4453, 0.8099, 0.5201)],
      [0.8, new THREE.Color(0.6102, 0.8748, 0.485)],
      [0.9, new THREE.Color(0.7883, 0.9323, 0.4514)],
      [1, new THREE.Color(0.9804, 0.9804, 0.4314)],
    ],
    VIRIDIS: [
      [0.0, new THREE.Color(0.267, 0.005, 0.329)],
      [0.1, new THREE.Color(0.283, 0.141, 0.458)],
      [0.2, new THREE.Color(0.254, 0.265, 0.53)],
      [0.3, new THREE.Color(0.207, 0.372, 0.553)],
      [0.4, new THREE.Color(0.164, 0.471, 0.558)],
      [0.5, new THREE.Color(0.128, 0.567, 0.551)],
      [0.6, new THREE.Color(0.135, 0.659, 0.518)],
      [0.7, new THREE.Color(0.267, 0.749, 0.441)],
      [0.8, new THREE.Color(0.478, 0.821, 0.318)],
      [0.9, new THREE.Color(0.741, 0.873, 0.15)],
      [1.0, new THREE.Color(0.993, 0.906, 0.144)],
    ],
    INFERNO: [
      [0.0, new THREE.Color(0.077, 0.042, 0.206)],
      [0.1, new THREE.Color(0.225, 0.036, 0.388)],
      [0.2, new THREE.Color(0.373, 0.074, 0.432)],
      [0.3, new THREE.Color(0.522, 0.128, 0.42)],
      [0.4, new THREE.Color(0.665, 0.182, 0.37)],
      [0.5, new THREE.Color(0.797, 0.255, 0.287)],
      [0.6, new THREE.Color(0.902, 0.364, 0.184)],
      [0.7, new THREE.Color(0.969, 0.516, 0.063)],
      [0.8, new THREE.Color(0.988, 0.683, 0.072)],
      [0.9, new THREE.Color(0.961, 0.859, 0.298)],
      [1.0, new THREE.Color(0.988, 0.998, 0.645)],
    ],
    GRAYSCALE: [[0, new THREE.Color(0, 0, 0)], [1, new THREE.Color(1, 1, 1)]],
  };

  var Shaders = {};

  //pointcloud.vs
  Shaders.vertex =
    `
precision highp float;
precision highp int;

#define MAX_CLIP_POLYGONS 8
#define PI 3.141592653589793

` +
    THREE.ShaderChunk.logdepthbuf_pars_vertex +
    `

attribute vec3 position;
attribute vec3 color;
attribute float intensity;
attribute float classification;
attribute float returnNumber;
attribute float numberOfReturns;
attribute float pointSourceID;
attribute vec4 indices;
attribute float spacing;

uniform mat4 modelMatrix;
uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;
uniform mat4 uViewInv;

uniform float uScreenWidth;
uniform float uScreenHeight;
uniform float fov;
uniform float near;
uniform float far;

uniform bool uDebug;

uniform bool uUseOrthographicCamera;
uniform float uOrthoWidth;
uniform float uOrthoHeight;

#define CLIPTASK_NONE 0
#define CLIPTASK_HIGHLIGHT 1
#define CLIPTASK_SHOW_INSIDE 2
#define CLIPTASK_SHOW_OUTSIDE 3

#define CLIPMETHOD_INSIDE_ANY 0
#define CLIPMETHOD_INSIDE_ALL 1

uniform int clipTask;
uniform int clipMethod;

#if defined(num_clipboxes) && num_clipboxes > 0
  uniform mat4 clipBoxes[num_clipboxes];
#endif

#if defined(num_clipspheres) && num_clipspheres > 0
  uniform mat4 uClipSpheres[num_clipspheres];
#endif

#if defined(num_clippolygons) && num_clippolygons > 0
  uniform int uClipPolygonVCount[num_clippolygons];
  uniform vec3 uClipPolygonVertices[num_clippolygons * 8];
  uniform mat4 uClipPolygonWVP[num_clippolygons];
#endif

uniform float size;
uniform float minSize;
uniform float maxSize;

uniform float uPCIndex;
uniform float uOctreeSpacing;
uniform float uNodeSpacing;
uniform float uOctreeSize;
uniform vec3 uBBSize;
uniform float uLevel;
uniform float uVNStart;
uniform bool uIsLeafNode;

uniform vec3 uColor;
uniform float uOpacity;
uniform float uTime;

uniform vec2 elevationRange;
uniform vec2 intensityRange;
uniform float intensityGamma;
uniform float intensityContrast;
uniform float intensityBrightness;
uniform float rgbGamma;
uniform float rgbContrast;
uniform float rgbBrightness;
uniform float uTransition;
uniform float wRGB;
uniform float wIntensity;
uniform float wElevation;
uniform float wClassification;
uniform float wReturnNumber;
uniform float wSourceID;

uniform vec3 uShadowColor;

uniform sampler2D visibleNodes;
uniform sampler2D gradient;
uniform sampler2D classificationLUT;

#if defined(num_shadowmaps) && num_shadowmaps > 0
  uniform sampler2D uShadowMap[num_shadowmaps];
  uniform mat4 uShadowWorldView[num_shadowmaps];
  uniform mat4 uShadowProj[num_shadowmaps];
#endif

varying vec3 vColor;
varying float vLogDepth;
varying vec3 vViewPosition;
varying float vRadius;
varying float vPointSize;

float round(float number)
{
  return floor(number + 0.5);
}

//---------------------
//OCTREE
//---------------------

#if (defined(adaptive_point_size) || defined(color_type_lod)) && defined(tree_type_octree)

  /**
   * number of 1-bits up to inclusive index position
   * number is treated as if it were an integer in the range 0-255
   */
  int numberOfOnes(int number, int index)
  {
    int numOnes = 0;
    int tmp = 128;

    for(int i = 7; i >= 0; i--)
    {
      if(number >= tmp)
      {
        number = number - tmp;

        if(i <= index)
        {
          numOnes++;
        }
      }
      
      tmp = tmp / 2;
    }

    return numOnes;
  }

  /**
   * checks whether the bit at index is 1
   * number is treated as if it were an integer in the range 0-255
   */
  bool isBitSet(int number, int index)
  {
    //weird multi else if due to lack of proper array, int and bitwise support in WebGL 1.0
    int powi = 1;

    if(index == 0)
    {
      powi = 1;
    }
    else if(index == 1)
    {
      powi = 2;
    }
    else if(index == 2)
    {
      powi = 4;
    }
    else if(index == 3)
    {
      powi = 8;
    }
    else if(index == 4)
    {
      powi = 16;
    }
    else if(index == 5)
    {
      powi = 32;
    }
    else if(index == 6)
    {
      powi = 64;
    }
    else if(index == 7)
    {
      powi = 128;
    }
    else
    {
      return false;
    }

    int ndp = number / powi;

    return mod(float(ndp), 2.0) != 0.0;
  }

  /**
   * find the LOD at the point position
   */
  float getLOD()
  {
    vec3 offset = vec3(0.0, 0.0, 0.0);
    int iOffset = int(uVNStart);
    float depth = uLevel;

    for(float i = 0.0; i <= 30.0; i++)
    {
      float nodeSizeAtLevel = uOctreeSize / pow(2.0, i + uLevel + 0.0);
      
      vec3 index3d = (position-offset) / nodeSizeAtLevel;
      index3d = floor(index3d + 0.5);
      int index = int(round(4.0 * index3d.x + 2.0 * index3d.y + index3d.z));
      
      vec4 value = texture2D(visibleNodes, vec2(float(iOffset) / 2048.0, 0.0));
      int mask = int(round(value.r * 255.0));

      if(isBitSet(mask, index))
      {
        //there are more visible child nodes at this position
        int advanceG = int(round(value.g * 255.0)) * 256;
        int advanceB = int(round(value.b * 255.0));
        int advanceChild = numberOfOnes(mask, index - 1);
        int advance = advanceG + advanceB + advanceChild;

        iOffset = iOffset + advance;
        
        depth++;
      }
      else
      {
        //no more visible child nodes at this position
        return value.a * 255.0;
        //return depth;
      }
      
      offset = offset + (vec3(1.0, 1.0, 1.0) * nodeSizeAtLevel * 0.5) * index3d;
    }
      
    return depth;
  }

  float getSpacing()
  {
    vec3 offset = vec3(0.0, 0.0, 0.0);
    int iOffset = int(uVNStart);
    float depth = uLevel;
    float spacing = uNodeSpacing;

    for(float i = 0.0; i <= 30.0; i++)
    {
      float nodeSizeAtLevel = uOctreeSize / pow(2.0, i + uLevel + 0.0);
      
      vec3 index3d = (position-offset) / nodeSizeAtLevel;
      index3d = floor(index3d + 0.5);
      int index = int(round(4.0 * index3d.x + 2.0 * index3d.y + index3d.z));
      
      vec4 value = texture2D(visibleNodes, vec2(float(iOffset) / 2048.0, 0.0));
      int mask = int(round(value.r * 255.0));
      float spacingFactor = value.a;

      if(i > 0.0)
      {
        spacing = spacing / (255.0 * spacingFactor);
      }
      
      if(isBitSet(mask, index))
      {
        //there are more visible child nodes at this position
        int advanceG = int(round(value.g * 255.0)) * 256;
        int advanceB = int(round(value.b * 255.0));
        int advanceChild = numberOfOnes(mask, index - 1);
        int advance = advanceG + advanceB + advanceChild;

        iOffset = iOffset + advance;

        depth++;
      }
      else
      {
        //no more visible child nodes at this position
        return spacing;
      }
      
      offset = offset + (vec3(1.0, 1.0, 1.0) * nodeSizeAtLevel * 0.5) * index3d;
    }
      
    return spacing;
  }

  float getPointSizeAttenuation()
  {
    return pow(2.0, getLOD());
  }
#endif

//---------------------
//KD-TREE
//---------------------
#if (defined(adaptive_point_size) || defined(color_type_lod)) && defined(tree_type_kdtree)
  float getLOD()
  {
    vec3 offset = vec3(0.0, 0.0, 0.0);
    float iOffset = 0.0;
    float depth = 0.0;
      
    vec3 size = uBBSize;	
    vec3 pos = position;
      
    for(float i = 0.0; i <= 1000.0; i++)
    {
      vec4 value = texture2D(visibleNodes, vec2(iOffset / 2048.0, 0.0));
      
      int children = int(value.r * 255.0);
      float next = value.g * 255.0;
      int split = int(value.b * 255.0);
      
      if(next == 0.0)
      {
         return depth;
      }
      
      vec3 splitv = vec3(0.0, 0.0, 0.0);
      if(split == 1)
      {
        splitv.x = 1.0;
      }
      else if(split == 2)
      {
         splitv.y = 1.0;
      }
      else if(split == 4)
      {
         splitv.z = 1.0;
      }
      
      iOffset = iOffset + next;
      
      float factor = length(pos * splitv / size);

      //Left
      if(factor < 0.5)
      {
        if(children == 0 || children == 2)
        {
          return depth;
        }
      }
      //Right
      else
      {
        pos = pos - size * splitv * 0.5;
        if(children == 0 || children == 1)
        {
          return depth;
        }
        if(children == 3)
        {
          iOffset = iOffset + 1.0;
        }
      }

      size = size * ((1.0 - (splitv + 1.0) / 2.0) + 0.5);
      depth++;
    }
      
    return depth;	
  }

  float getPointSizeAttenuation()
  {
    return 0.5 * pow(1.3, getLOD());
  }
#endif

//formula adapted from: http://www.dfstudios.co.uk/articles/programming/image-programming-algorithms/image-processing-algorithms-part-5-contrast-adjustment/
float getContrastFactor(float contrast)
{
  return (1.0158730158730156 * (contrast + 1.0)) / (1.0158730158730156 - contrast);
}

vec3 getRGB()
{
  vec3 rgb = color;
  
  rgb = pow(rgb, vec3(rgbGamma));
  rgb = rgb + rgbBrightness;
  rgb = clamp(rgb, 0.0, 1.0);
  
  return rgb;
}

float getIntensity()
{
  float w = (intensity - intensityRange.x) / (intensityRange.y - intensityRange.x);
  w = pow(w, intensityGamma);
  w = w + intensityBrightness;
  w = (w - 0.5) * getContrastFactor(intensityContrast) + 0.5;
  w = clamp(w, 0.0, 1.0);

  return w;
}

vec3 getElevation()
{
  vec4 world = modelMatrix * vec4( position, 1.0 );
  float w = (world.z - elevationRange.x) / (elevationRange.y - elevationRange.x);
  return texture2D(gradient, vec2(w,1.0-w)).rgb;
}

vec4 getClassification()
{
  vec2 uv = vec2(classification / 255.0, 0.5);
  return texture2D(classificationLUT, uv);
}

vec3 getReturnNumber()
{
  if(numberOfReturns == 1.0)
  {
    return vec3(1.0, 1.0, 0.0);
  }
  else
  {
    if(returnNumber == 1.0)
    {
      return vec3(1.0, 0.0, 0.0);
    }
    else if(returnNumber == numberOfReturns)
    {
      return vec3(0.0, 0.0, 1.0);
    }
    else
    {
      return vec3(0.0, 1.0, 0.0);
    }
  }
}

vec3 getSourceID()
{
  float w = mod(pointSourceID, 10.0) / 10.0;
  return texture2D(gradient, vec2(w,1.0 - w)).rgb;
}

vec3 getCompositeColor()
{
  vec3 c;
  float w;

  c += wRGB * getRGB();
  w += wRGB;
  
  c += wIntensity * getIntensity() * vec3(1.0, 1.0, 1.0);
  w += wIntensity;
  
  c += wElevation * getElevation();
  w += wElevation;
  
  c += wReturnNumber * getReturnNumber();
  w += wReturnNumber;
  
  c += wSourceID * getSourceID();
  w += wSourceID;
  
  vec4 cl = wClassification * getClassification();
    c += cl.a * cl.rgb;
  w += wClassification * cl.a;

  c = c / w;
  
  if(w == 0.0)
  {
    gl_Position = vec4(100.0, 100.0, 100.0, 0.0);
  }
  
  return c;
}

vec3 getColor()
{
  vec3 color;
  
  #ifdef color_type_rgb
    color = getRGB();
  #elif defined color_type_height
    color = getElevation();
  #elif defined color_type_rgb_height
    vec3 cHeight = getElevation();
    color = (1.0 - uTransition) * getRGB() + uTransition * cHeight;
  #elif defined color_type_depth
    float linearDepth = gl_Position.w;
    float expDepth = (gl_Position.z / gl_Position.w) * 0.5 + 0.5;
    color = vec3(linearDepth, expDepth, 0.0);
  #elif defined color_type_intensity
    float w = getIntensity();
    color = vec3(w, w, w);
  #elif defined color_type_intensity_gradient
    float w = getIntensity();
    color = texture2D(gradient, vec2(w,1.0-w)).rgb;
  #elif defined color_type_color
    color = uColor;
  #elif defined color_type_lod
    float depth = getLOD();
    float w = depth / 10.0;
    color = texture2D(gradient, vec2(w,1.0-w)).rgb;
  #elif defined color_type_point_index
    color = indices.rgb;
  #elif defined color_type_classification
    vec4 cl = getClassification(); 
    color = cl.rgb;
  #elif defined color_type_return_number
    color = getReturnNumber();
  #elif defined color_type_source
    color = getSourceID();
  #elif defined color_type_normal
    color = (modelMatrix * vec4(normal, 0.0)).xyz;
  #elif defined color_type_phong
    color = color;
  #elif defined color_type_composite
    color = getCompositeColor();
  #endif
  
  return color;
}

float getPointSize()
{
  float pointSize = 1.0;
  
  float slope = tan(fov / 2.0);
  float projFactor = -0.5 * uScreenHeight / (slope * vViewPosition.z);
  
  float r = uOctreeSpacing * 1.7;
  vRadius = r;

  #if defined fixed_point_size
    pointSize = size;
  #elif defined attenuated_point_size
    if(uUseOrthographicCamera)
    {
      pointSize = size;
    }
    else
    {
      pointSize = size * spacing * projFactor;
    }
  #elif defined adaptive_point_size
    if(uUseOrthographicCamera)
    {
      float worldSpaceSize = 1.0 * size * r / getPointSizeAttenuation();
      pointSize = (worldSpaceSize / uOrthoWidth) * uScreenWidth;
    }
    else
    {
      if(uIsLeafNode && false)
      {
        pointSize = size * spacing * projFactor;
      }
      else
      {
        float worldSpaceSize = 1.0 * size * r / getPointSizeAttenuation();
        pointSize = worldSpaceSize * projFactor;
      }
    }
  #endif

  pointSize = max(minSize, pointSize);
  pointSize = min(maxSize, pointSize);
  
  vRadius = pointSize / projFactor;

  return pointSize;
}

#if defined num_clippolygons && num_clippolygons > 0
  bool pointInClipPolygon(vec3 point, int polyIdx)
  {
    mat4 wvp = uClipPolygonWVP[polyIdx];

    vec4 pointNDC = wvp * vec4(point, 1.0);
    pointNDC.xy = pointNDC.xy / pointNDC.w;

    int j = uClipPolygonVCount[polyIdx] - 1;
    bool c = false;
    for(int i = 0; i < 8; i++)
    {
      if(i == uClipPolygonVCount[polyIdx])
      {
        break;
      }

      vec3 verti = uClipPolygonVertices[polyIdx * 8 + i];
      vec3 vertj = uClipPolygonVertices[polyIdx * 8 + j];

      if(((verti.y > pointNDC.y) != (vertj.y > pointNDC.y)) && (pointNDC.x < (vertj.x-verti.x) * (pointNDC.y-verti.y) / (vertj.y-verti.y) + verti.x))
      {
        c = !c;
      }

      j = i;
    }

    return c;
  }
#endif

void doClipping()
{
  #if !defined color_type_composite
    vec4 cl = getClassification(); 
    if(cl.a == 0.0)
    {
      gl_Position = vec4(100.0, 100.0, 100.0, 0.0);
      
      return;
    }
  #endif

  int clipVolumesCount = 0;
  int insideCount = 0;

  #if defined(num_clipboxes) && num_clipboxes > 0
    for(int i = 0; i < num_clipboxes; i++)
    {
      vec4 clipPosition = clipBoxes[i] * modelMatrix * vec4( position, 1.0 );
      bool inside = -0.5 <= clipPosition.x && clipPosition.x <= 0.5;
      inside = inside && -0.5 <= clipPosition.y && clipPosition.y <= 0.5;
      inside = inside && -0.5 <= clipPosition.z && clipPosition.z <= 0.5;

      insideCount = insideCount + (inside ? 1 : 0);
      clipVolumesCount++;
    }	
  #endif

  #if defined(num_clippolygons) && num_clippolygons > 0
    for(int i = 0; i < num_clippolygons; i++)
    {
      bool inside = pointInClipPolygon(position, i);

      insideCount = insideCount + (inside ? 1 : 0);
      clipVolumesCount++;
    }
  #endif

  bool insideAny = insideCount > 0;
  bool insideAll = (clipVolumesCount > 0) && (clipVolumesCount == insideCount);

  if(clipMethod == CLIPMETHOD_INSIDE_ANY)
  {
    if(insideAny && clipTask == CLIPTASK_HIGHLIGHT)
    {
      vColor.r += 0.5;
    }
    else if(!insideAny && clipTask == CLIPTASK_SHOW_INSIDE)
    {
      gl_Position = vec4(100.0, 100.0, 100.0, 1.0);
    }
    else if(insideAny && clipTask == CLIPTASK_SHOW_OUTSIDE)
    {
      gl_Position = vec4(100.0, 100.0, 100.0, 1.0);
    }
  }
  else if(clipMethod == CLIPMETHOD_INSIDE_ALL)
  {
    if(insideAll && clipTask == CLIPTASK_HIGHLIGHT)
    {
      vColor.r += 0.5;
    }
    else if(!insideAll && clipTask == CLIPTASK_SHOW_INSIDE)
    {
      gl_Position = vec4(100.0, 100.0, 100.0, 1.0);
    }
    else if(insideAll && clipTask == CLIPTASK_SHOW_OUTSIDE)
    {
      gl_Position = vec4(100.0, 100.0, 100.0, 1.0);
    }
  }
}

void main()
{
  // EDITED
  vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
  vViewPosition = mvPosition.xyz;
  gl_Position = projectionMatrix * mvPosition;
                // + vec4(
                //   (cos(uTime + mvPosition.z / 10.0) / 2.0 + 0.5) * 2.0,
                //   (sin(uTime + mvPosition.z / 10.0) / 2.0 + 0.5) * 5.0,
                //   0.0,
                //   0.0
                // );

  vLogDepth = log2(-mvPosition.z);

  //POINT SIZE
  float pointSize = getPointSize();
  gl_PointSize = pointSize;
  vPointSize = pointSize;

  ` +
    THREE.ShaderChunk.logdepthbuf_vertex +
    `

  //COLOR
  vColor = getColor();

  #if defined hq_depth_pass
    float originalDepth = gl_Position.w;
    float adjustedDepth = originalDepth + 2.0 * vRadius;
    float adjust = adjustedDepth / originalDepth;

    mvPosition.xyz = mvPosition.xyz * adjust;
    gl_Position = projectionMatrix * mvPosition;
  #endif

  //CLIPPING
  doClipping();

  #if defined num_clipspheres && num_clipspheres > 0
    for(int i = 0; i < num_clipspheres; i++)
    {
      vec4 sphereLocal = uClipSpheres[i] * mvPosition;

      float distance = length(sphereLocal.xyz);

      if(distance < 1.0)
      {
        float w = distance;
        vec3 cGradient = texture2D(gradient, vec2(w, 1.0 - w)).rgb;
        
        vColor = cGradient;
      }
    }
  #endif

  #if defined num_shadowmaps && num_shadowmaps > 0

    const float sm_near = 0.1;
    const float sm_far = 10000.0;

    for(int i = 0; i < num_shadowmaps; i++)
    {
      vec3 viewPos = (uShadowWorldView[i] * vec4(position, 1.0)).xyz;
      float distanceToLight = abs(viewPos.z);
      
      vec4 projPos = uShadowProj[i] * uShadowWorldView[i] * vec4(position, 1);
      vec3 nc = projPos.xyz / projPos.w;
      
      float u = nc.x * 0.5 + 0.5;
      float v = nc.y * 0.5 + 0.5;

      vec2 sampleStep = vec2(1.0 / (2.0*1024.0), 1.0 / (2.0*1024.0)) * 1.5;
      vec2 sampleLocations[9];

      sampleLocations[0] = vec2(0.0, 0.0);
      sampleLocations[1] = sampleStep;
      sampleLocations[2] = -sampleStep;
      sampleLocations[3] = vec2(sampleStep.x, -sampleStep.y);
      sampleLocations[4] = vec2(-sampleStep.x, sampleStep.y);
      sampleLocations[5] = vec2(0.0, sampleStep.y);
      sampleLocations[6] = vec2(0.0, -sampleStep.y);
      sampleLocations[7] = vec2(sampleStep.x, 0.0);
      sampleLocations[8] = vec2(-sampleStep.x, 0.0);

      float visibleSamples = 0.0;
      float numSamples = 0.0;

      float bias = vRadius * 2.0;

      for(int j = 0; j < 9; j++)
      {
        vec4 depthMapValue = texture2D(uShadowMap[i], vec2(u, v) + sampleLocations[j]);

        float linearDepthFromSM = depthMapValue.x + bias;
        float linearDepthFromViewer = distanceToLight;

        if(linearDepthFromSM > linearDepthFromViewer)
        {
          visibleSamples += 1.0;
        }

        numSamples += 1.0;
      }

      float visibility = visibleSamples / numSamples;

      if(u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 || nc.x < -1.0 || nc.x > 1.0 || nc.y < -1.0 || nc.y > 1.0 || nc.z < -1.0 || nc.z > 1.0)
      {
        //vColor = vec3(0.0, 0.0, 0.2);
      }
      else
      {
        vColor = vColor * visibility + vColor * uShadowColor * (1.0 - visibility);
      }
    }

  #endif
}`;

  //"pointcloud.fs"
  Shaders.fragment =
    `

#if defined USE_LOGDEPTHBUF_EXT || defined paraboloid_point_shape
  #extension GL_EXT_frag_depth : enable
#endif

precision highp float;
precision highp int;

` +
    THREE.ShaderChunk.logdepthbuf_pars_fragment +
    `

uniform mat4 viewMatrix;
uniform mat4 uViewInv;
uniform mat4 uProjInv;
uniform vec3 cameraPosition;

uniform mat4 projectionMatrix;
uniform float uOpacity;

uniform float blendHardness;
uniform float blendDepthSupplement;
uniform float fov;
uniform float uSpacing;
uniform float near;
uniform float far;
uniform float uPCIndex;
uniform float uScreenWidth;
uniform float uScreenHeight;

uniform float uTime;

varying vec3 vColor;
varying float vLogDepth;
varying vec3 vViewPosition;
varying float vRadius;
varying float vPointSize;
varying vec3 vPosition;

void main()
{
  vec3 color = vColor;
  float depth = gl_FragCoord.z;

  #if defined circle_point_shape || defined paraboloid_point_shape
    float u = (2.0 * gl_PointCoord.x) - 1.0;
    float v = (2.0 * gl_PointCoord.y) - 1.0;
  #endif
  
  #if defined circle_point_shape
    float cc = (u*u) + (v*v);
    if(cc > 1.0)
    {
      discard;
    }
  #endif

  #if defined color_type_point_index
    gl_FragColor = vec4(color, uPCIndex / 255.0);
  #else
    // EDITED
    // gl_FragColor = vec4(color * min(1.0, smoothstep(0.0, 1.0, uTime / 5.0)), uOpacity);
    // gl_FragColor = vec4(color * min(1.0, (sin(uTime / 4.0) / 2.0 + 0.5)), uOpacity);
    // gl_FragColor = vec4(color * uTime, uOpacity);
    gl_FragColor = vec4(color * uTime, uOpacity);
  #endif

  #if defined paraboloid_point_shape
    float wi = -( u*u + v*v);
    vec4 pos = vec4(vViewPosition, 1.0);
    pos.z += wi * vRadius;
    float linearDepth = -pos.z;
    pos = projectionMatrix * pos;
    pos = pos / pos.w;
    float expDepth = pos.z;
    depth = (pos.z + 1.0) / 2.0;

    gl_FragDepthEXT = depth;
    
    #if defined color_type_depth
      color.r = linearDepth;
      color.g = expDepth;
    #endif
  #endif
  
  ` +
    THREE.ShaderChunk.logdepthbuf_fragment +
    `

  #if defined weighted_splats
    float distance = 2.0 * length(gl_PointCoord.xy - 0.5);
    float weight = max(0.0, 1.0 - distance);
    weight = pow(weight, 1.5);

    gl_FragColor.a = weight;
    gl_FragColor.xyz = gl_FragColor.xyz * weight;
  #endif
}`;

  class PointCloudMaterial extends THREE.RawShaderMaterial {
    constructor(parameters = {}) {
      super();

      this.visibleNodesTexture = HelperUtils.generateDataTexture(
        2048,
        1,
        new THREE.Color(0xffffff)
      );
      this.visibleNodesTexture.minFilter = THREE.NearestFilter;
      this.visibleNodesTexture.magFilter = THREE.NearestFilter;

      var getValid = function(a, b) {
        if (a !== undefined) {
          return a;
        } else {
          return b;
        }
      };

      var pointSize = getValid(parameters.size, 1.0);
      var minSize = getValid(parameters.minSize, 2.0);
      var maxSize = getValid(parameters.maxSize, 50.0);
      var treeType = getValid(parameters.treeType, TreeType.OCTREE);

      this._pointSizeType = PointSizeType.FIXED;
      this._shape = PointShape.SQUARE;
      this._pointColorType = PointColorType.RGB;
      this._useClipBox = false;
      this._weighted = false;
      this._gradient = Gradients.SPECTRAL;
      this._treeType = treeType;
      this._useEDL = false;
      this._snapEnabled = false;
      this._numSnapshots = 0;
      this._defaultIntensityRangeChanged = false;
      this._defaultElevationRangeChanged = false;

      this.clipBoxes = [];
      this.clipPolygons = [];

      this.gradientTexture = PointCloudMaterial.generateGradientTexture(
        this._gradient
      );
      this.lights = false;
      this.fog = false;
      this.defines = new Map();

      this.attributes = {
        position: { type: 'fv', value: [] },
        color: { type: 'fv', value: [] },
        normal: { type: 'fv', value: [] },
        intensity: { type: 'f', value: [] },
        classification: { type: 'f', value: [] },
        returnNumber: { type: 'f', value: [] },
        numberOfReturns: { type: 'f', value: [] },
        pointSourceID: { type: 'f', value: [] },
        indices: { type: 'fv', value: [] },
      };

      this.uniforms = {
        level: { type: 'f', value: 0.0 },
        vnStart: { type: 'f', value: 0.0 },
        spacing: { type: 'f', value: 1.0 },
        blendHardness: { type: 'f', value: 2.0 },
        blendDepthSupplement: { type: 'f', value: 0.0 },
        fov: { type: 'f', value: 1.0 },
        screenWidth: { type: 'f', value: 1.0 },
        screenHeight: { type: 'f', value: 1.0 },
        near: { type: 'f', value: 0.1 },
        far: { type: 'f', value: 1.0 },
        uColor: { type: 'c', value: new THREE.Color(0xffffff) },
        uOpacity: { type: 'f', value: 1.0 },
        size: { type: 'f', value: pointSize },
        minSize: { type: 'f', value: minSize },
        maxSize: { type: 'f', value: maxSize },
        octreeSize: { type: 'f', value: 0 },
        bbSize: { type: 'fv', value: [0, 0, 0] },
        elevationRange: { type: '2fv', value: [0, 0] },

        clipBoxCount: { type: 'f', value: 0 },
        //clipSphereCount: {type: "f", value: 0},
        clipPolygonCount: { type: 'i', value: 0 },
        clipBoxes: { type: 'Matrix4fv', value: [] },
        //clipSpheres: {type: "Matrix4fv", value: []},
        clipPolygons: { type: '3fv', value: [] },
        clipPolygonVCount: { type: 'iv', value: [] },
        clipPolygonVP: { type: 'Matrix4fv', value: [] },

        // EDITED
        uTime: { type: 'f', value: 0.0 },

        visibleNodes: { type: 't', value: this.visibleNodesTexture },
        pcIndex: { type: 'f', value: 0 },
        gradient: { type: 't', value: this.gradientTexture },
        classificationLUT: { type: 't', value: this.classificationTexture },
        uHQDepthMap: { type: 't', value: null },
        toModel: { type: 'Matrix4f', value: [] },
        diffuse: { type: 'fv', value: [1, 1, 1] },
        transition: { type: 'f', value: 0.5 },
        intensityRange: { type: 'fv', value: [0, 65000] },
        intensityGamma: { type: 'f', value: 1 },
        intensityContrast: { type: 'f', value: 0 },
        intensityBrightness: { type: 'f', value: 0 },
        rgbGamma: { type: 'f', value: 1 },
        rgbContrast: { type: 'f', value: 0 },
        rgbBrightness: { type: 'f', value: 0 },
        wRGB: { type: 'f', value: 1 },
        wIntensity: { type: 'f', value: 0 },
        wElevation: { type: 'f', value: 0 },
        wClassification: { type: 'f', value: 0 },
        wReturnNumber: { type: 'f', value: 0 },
        wSourceID: { type: 'f', value: 0 },
        useOrthographicCamera: { type: 'b', value: false },
        clipTask: { type: 'i', value: 1 },
        clipMethod: { type: 'i', value: 1 },
        uSnapshot: { type: 'tv', value: [] },
        uSnapshotDepth: { type: 'tv', value: [] },
        uSnapView: { type: 'Matrix4fv', value: [] },
        uSnapProj: { type: 'Matrix4fv', value: [] },
        uSnapProjInv: { type: 'Matrix4fv', value: [] },
        uSnapViewInv: { type: 'Matrix4fv', value: [] },
        uShadowColor: { type: '3fv', value: [0, 0, 0] },

        uFilterReturnNumberRange: { type: 'fv', value: [0, 7] },
        uFilterNumberOfReturnsRange: { type: 'fv', value: [0, 7] },
        uFilterGPSTimeClipRange: { type: 'fv', value: [0, 7] },
      };

      this.classification = Classification.DEFAULT;
      this.defaultAttributeValues.normal = [0, 0, 0];
      this.defaultAttributeValues.classification = [0, 0, 0];
      this.defaultAttributeValues.indices = [0, 0, 0, 0];

      var defines = this.getDefines();
      this.vertexShader = defines + Shaders.vertex;
      this.fragmentShader = defines + Shaders.fragment;
      this.vertexColors = THREE.VertexColors;
    }

    setDefine(key, value) {
      if (value !== undefined && value !== null) {
        if (this.defines.get(key) !== value) {
          this.defines.set(key, value);
          this.updateShaderSource();
        }
      } else {
        this.removeDefine(key);
      }
    }

    removeDefine(key) {
      this.defines.delete(key);
    }

    updateShaderSource() {
      var defines = this.getDefines();
      this.vertexShader = defines + Shaders.vertex;
      this.fragmentShader = defines + Shaders.fragment;

      if (this.opacity === 1.0) {
        this.blending = THREE.NoBlending;
        this.transparent = false;
        this.depthTest = true;
        this.depthWrite = true;
        this.depthFunc = THREE.LessEqualDepth;
      } else if (this.opacity < 1.0 && !this.useEDL) {
        this.blending = THREE.AdditiveBlending;
        this.transparent = true;
        this.depthTest = false;
        this.depthWrite = true;
        this.depthFunc = THREE.AlwaysDepth;
      }

      if (this.weighted) {
        this.blending = THREE.AdditiveBlending;
        this.transparent = true;
        this.depthTest = true;
        this.depthWrite = false;
      }

      this.needsUpdate = true;
    }

    onBeforeCompile(shader, renderer) {
      if (renderer.capabilities.logarithmicDepthBuffer) {
        var define =
          '#define USE_LOGDEPTHBUF\n#define USE_LOGDEPTHBUF_EXT\n#define EPSILON 1e-6\n';
        shader.fragmentShader = define + shader.fragmentShader;
        shader.vertexShader = define + shader.vertexShader;
      }
    }

    getDefines() {
      var defines = [];

      if (this.pointSizeType === PointSizeType.FIXED) {
        defines.push('#define fixed_point_size');
      } else if (this.pointSizeType === PointSizeType.ATTENUATED) {
        defines.push('#define attenuated_point_size');
      } else if (this.pointSizeType === PointSizeType.ADAPTIVE) {
        defines.push('#define adaptive_point_size');
      }

      if (this.shape === PointShape.SQUARE) {
        defines.push('#define square_point_shape');
      } else if (this.shape === PointShape.CIRCLE) {
        defines.push('#define circle_point_shape');
      } else if (this.shape === PointShape.PARABOLOID) {
        defines.push('#define paraboloid_point_shape');
      }

      if (this._useEDL) {
        defines.push('#define use_edl');
      }

      if (this._snapEnabled) {
        defines.push('#define snap_enabled');
      }

      if (this._pointColorType === PointColorType.RGB) {
        defines.push('#define color_type_rgb');
      } else if (this._pointColorType === PointColorType.COLOR) {
        defines.push('#define color_type_color');
      } else if (this._pointColorType === PointColorType.DEPTH) {
        defines.push('#define color_type_depth');
      } else if (this._pointColorType === PointColorType.HEIGHT) {
        defines.push('#define color_type_height');
      } else if (this._pointColorType === PointColorType.INTENSITY) {
        defines.push('#define color_type_intensity');
      } else if (this._pointColorType === PointColorType.INTENSITY_GRADIENT) {
        defines.push('#define color_type_intensity_gradient');
      } else if (this._pointColorType === PointColorType.LOD) {
        defines.push('#define color_type_lod');
      } else if (this._pointColorType === PointColorType.POINT_INDEX) {
        defines.push('#define color_type_point_index');
      } else if (this._pointColorType === PointColorType.CLASSIFICATION) {
        defines.push('#define color_type_classification');
      } else if (this._pointColorType === PointColorType.RETURN_NUMBER) {
        defines.push('#define color_type_return_number');
      } else if (this._pointColorType === PointColorType.SOURCE) {
        defines.push('#define color_type_source');
      } else if (this._pointColorType === PointColorType.NORMAL) {
        defines.push('#define color_type_normal');
      } else if (this._pointColorType === PointColorType.PHONG) {
        defines.push('#define color_type_phong');
      } else if (this._pointColorType === PointColorType.RGB_HEIGHT) {
        defines.push('#define color_type_rgb_height');
      } else if (this._pointColorType === PointColorType.COMPOSITE) {
        defines.push('#define color_type_composite');
      }

      if (this._treeType === TreeType.OCTREE) {
        defines.push('#define tree_type_octree');
      } else if (this._treeType === TreeType.KDTREE) {
        defines.push('#define tree_type_kdtree');
      }

      if (this.weighted) {
        defines.push('#define weighted_splats');
      }

      for (var [key, value] of this.defines) {
        defines.push(value);
      }

      return defines.join('\n');
    }

    setClipBoxes(clipBoxes) {
      if (!clipBoxes) {
        return;
      }

      var doUpdate =
        this.clipBoxes.length !== clipBoxes.length &&
        (clipBoxes.length === 0 || this.clipBoxes.length === 0);
      this.uniforms.clipBoxCount.value = this.clipBoxes.length;
      this.clipBoxes = clipBoxes;

      if (doUpdate) {
        this.updateShaderSource();
      }

      this.uniforms.clipBoxes.value = new Float32Array(
        this.clipBoxes.length * 16
      );

      for (var i = 0; i < this.clipBoxes.length; i++) {
        var box = clipBoxes[i];
        this.uniforms.clipBoxes.value.set(box.inverse.elements, 16 * i);
      }

      for (var i = 0; i < this.uniforms.clipBoxes.value.length; i++) {
        if (Number.isNaN(this.uniforms.clipBoxes.value[i])) {
          this.uniforms.clipBoxes.value[i] = Infinity;
        }
      }
    }

    setClipPolygons(clipPolygons, maxPolygonVertices) {
      if (!clipPolygons) {
        return;
      }
      this.clipPolygons = clipPolygons;
      var doUpdate = this.clipPolygons.length !== clipPolygons.length;
      if (doUpdate) {
        this.updateShaderSource();
      }
    }

    get gradient() {
      return this._gradient;
    }

    set gradient(value) {
      if (this._gradient !== value) {
        this._gradient = value;
        this.gradientTexture = PointCloudMaterial.generateGradientTexture(
          this._gradient
        );
        this.uniforms.gradient.value = this.gradientTexture;
      }
    }

    get useOrthographicCamera() {
      return this.uniforms.useOrthographicCamera.value;
    }

    set useOrthographicCamera(value) {
      if (this.uniforms.useOrthographicCamera.value !== value) {
        this.uniforms.useOrthographicCamera.value = value;
      }
    }

    get classification() {
      return this._classification;
    }

    set classification(value) {
      var copy = {};
      for (var key of Object.keys(value)) {
        copy[key] = value[key].clone();
      }

      var isEqual = false;
      if (this._classification === undefined) {
        isEqual = false;
      } else {
        isEqual =
          Object.keys(copy).length === Object.keys(this._classification).length;
        for (var key of Object.keys(copy)) {
          isEqual = isEqual && this._classification[key] !== undefined;
          isEqual = isEqual && copy[key].equals(this._classification[key]);
        }
      }

      if (!isEqual) {
        this._classification = copy;
        this.recomputeClassification();
      }
    }

    recomputeClassification() {
      this.classificationTexture = PointCloudMaterial.generateClassificationTexture(
        this._classification
      );
      this.uniforms.classificationLUT.value = this.classificationTexture;
      this.dispatchEvent({
        type: 'material_property_changed',
        target: this,
      });
    }

    get numSnapshots() {
      return this._numSnapshots;
    }

    set numSnapshots(value) {
      this._numSnapshots = value;
    }

    get snapEnabled() {
      return this._snapEnabled;
    }

    set snapEnabled(value) {
      if (this._snapEnabled !== value) {
        this._snapEnabled = value;
        this.updateShaderSource();
      }
    }

    get spacing() {
      return this.uniforms.spacing.value;
    }

    set spacing(value) {
      if (this.uniforms.spacing.value !== value) {
        this.uniforms.spacing.value = value;
      }
    }

    get useClipBox() {
      return this._useClipBox;
    }

    set useClipBox(value) {
      if (this._useClipBox !== value) {
        this._useClipBox = value;
        this.updateShaderSource();
      }
    }

    get clipTask() {
      return this.uniforms.clipTask.value;
    }

    set clipTask(mode) {
      this.uniforms.clipTask.value = mode;
    }

    get clipMethod() {
      return this.uniforms.clipMethod.value;
    }

    set clipMethod(mode) {
      this.uniforms.clipMethod.value = mode;
    }

    get weighted() {
      return this._weighted;
    }

    set weighted(value) {
      if (this._weighted !== value) {
        this._weighted = value;
        this.updateShaderSource();
      }
    }

    get fov() {
      return this.uniforms.fov.value;
    }

    set fov(value) {
      if (this.uniforms.fov.value !== value) {
        this.uniforms.fov.value = value;
        this.updateShaderSource();
      }
    }

    get screenWidth() {
      return this.uniforms.screenWidth.value;
    }

    set screenWidth(value) {
      if (this.uniforms.screenWidth.value !== value) {
        this.uniforms.screenWidth.value = value;
        this.updateShaderSource();
      }
    }

    get screenHeight() {
      return this.uniforms.screenHeight.value;
    }

    set screenHeight(value) {
      if (this.uniforms.screenHeight.value !== value) {
        this.uniforms.screenHeight.value = value;
        this.updateShaderSource();
      }
    }

    get near() {
      return this.uniforms.near.value;
    }

    set near(value) {
      if (this.uniforms.near.value !== value) {
        this.uniforms.near.value = value;
      }
    }

    get far() {
      return this.uniforms.far.value;
    }

    set far(value) {
      if (this.uniforms.far.value !== value) {
        this.uniforms.far.value = value;
      }
    }

    get opacity() {
      return this.uniforms.uOpacity.value;
    }

    set opacity(value) {
      if (this.uniforms && this.uniforms.uOpacity) {
        if (this.uniforms.uOpacity.value !== value) {
          this.uniforms.uOpacity.value = value;
          this.updateShaderSource();
          this.dispatchEvent({
            type: 'opacity_changed',
            target: this,
          });
          this.dispatchEvent({
            type: 'material_property_changed',
            target: this,
          });
        }
      }
    }

    get pointColorType() {
      return this._pointColorType;
    }

    set pointColorType(value) {
      if (this._pointColorType !== value) {
        this._pointColorType = value;
        this.updateShaderSource();
        this.dispatchEvent({
          type: 'point_color_type_changed',
          target: this,
        });
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get pointSizeType() {
      return this._pointSizeType;
    }

    set pointSizeType(value) {
      if (this._pointSizeType !== value) {
        this._pointSizeType = value;
        this.updateShaderSource();
        this.dispatchEvent({
          type: 'point_size_type_changed',
          target: this,
        });
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get useEDL() {
      return this._useEDL;
    }

    set useEDL(value) {
      if (this._useEDL !== value) {
        this._useEDL = value;
        this.updateShaderSource();
      }
    }

    get color() {
      return this.uniforms.uColor.value;
    }

    set color(value) {
      if (!this.uniforms.uColor.value.equals(value)) {
        this.uniforms.uColor.value.copy(value);
        this.dispatchEvent({
          type: 'color_changed',
          target: this,
        });
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get shape() {
      return this._shape;
    }

    set shape(value) {
      if (this._shape !== value) {
        this._shape = value;
        this.updateShaderSource();
        this.dispatchEvent({
          type: 'point_shape_changed',
          target: this,
        });
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get treeType() {
      return this._treeType;
    }

    set treeType(value) {
      if (this._treeType !== value) {
        this._treeType = value;
        this.updateShaderSource();
      }
    }

    get bbSize() {
      return this.uniforms.bbSize.value;
    }

    set bbSize(value) {
      this.uniforms.bbSize.value = value;
    }

    get size() {
      return this.uniforms.size.value;
    }

    set size(value) {
      if (this.uniforms.size.value !== value) {
        this.uniforms.size.value = value;
        this.dispatchEvent({
          type: 'point_size_changed',
          target: this,
        });
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get elevationRange() {
      return this.uniforms.elevationRange.value;
    }

    set elevationRange(value) {
      var changed =
        this.uniforms.elevationRange.value[0] !== value[0] ||
        this.uniforms.elevationRange.value[1] !== value[1];
      if (changed) {
        this.uniforms.elevationRange.value = value;
        this._defaultElevationRangeChanged = true;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get heightMin() {
      return this.uniforms.elevationRange.value[0];
    }

    set heightMin(value) {
      this.elevationRange = [value, this.elevationRange[1]];
    }

    get heightMax() {
      return this.uniforms.elevationRange.value[1];
    }

    set heightMax(value) {
      this.elevationRange = [this.elevationRange[0], value];
    }

    get transition() {
      return this.uniforms.transition.value;
    }

    set transition(value) {
      this.uniforms.transition.value = value;
    }

    get intensityRange() {
      return this.uniforms.intensityRange.value;
    }

    set intensityRange(value) {
      if (!(value instanceof Array && value.length === 2)) {
        return;
      }

      if (
        value[0] === this.uniforms.intensityRange.value[0] &&
        value[1] === this.uniforms.intensityRange.value[1]
      ) {
        return;
      }

      this.uniforms.intensityRange.value = value;
      this._defaultIntensityRangeChanged = true;

      this.dispatchEvent({
        type: 'material_property_changed',
        target: this,
      });
    }

    get intensityGamma() {
      return this.uniforms.intensityGamma.value;
    }

    set intensityGamma(value) {
      if (this.uniforms.intensityGamma.value !== value) {
        this.uniforms.intensityGamma.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get intensityContrast() {
      return this.uniforms.intensityContrast.value;
    }

    set intensityContrast(value) {
      if (this.uniforms.intensityContrast.value !== value) {
        this.uniforms.intensityContrast.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get intensityBrightness() {
      return this.uniforms.intensityBrightness.value;
    }

    set intensityBrightness(value) {
      if (this.uniforms.intensityBrightness.value !== value) {
        this.uniforms.intensityBrightness.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get rgbGamma() {
      return this.uniforms.rgbGamma.value;
    }

    set rgbGamma(value) {
      if (this.uniforms.rgbGamma.value !== value) {
        this.uniforms.rgbGamma.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get rgbContrast() {
      return this.uniforms.rgbContrast.value;
    }

    set rgbContrast(value) {
      if (this.uniforms.rgbContrast.value !== value) {
        this.uniforms.rgbContrast.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get rgbBrightness() {
      return this.uniforms.rgbBrightness.value;
    }

    set rgbBrightness(value) {
      if (this.uniforms.rgbBrightness.value !== value) {
        this.uniforms.rgbBrightness.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    // EDITED
    get time() {
      return this.uniforms.uTime.value;
    }
    set time(value) {
      this.uniforms.uTime.value = value;
    }

    get weightRGB() {
      return this.uniforms.wRGB.value;
    }

    set weightRGB(value) {
      if (this.uniforms.wRGB.value !== value) {
        this.uniforms.wRGB.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get weightIntensity() {
      return this.uniforms.wIntensity.value;
    }

    set weightIntensity(value) {
      if (this.uniforms.wIntensity.value !== value) {
        this.uniforms.wIntensity.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get weightElevation() {
      return this.uniforms.wElevation.value;
    }

    set weightElevation(value) {
      if (this.uniforms.wElevation.value !== value) {
        this.uniforms.wElevation.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get weightClassification() {
      return this.uniforms.wClassification.value;
    }

    set weightClassification(value) {
      if (this.uniforms.wClassification.value !== value) {
        this.uniforms.wClassification.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get weightReturnNumber() {
      return this.uniforms.wReturnNumber.value;
    }

    set weightReturnNumber(value) {
      if (this.uniforms.wReturnNumber.value !== value) {
        this.uniforms.wReturnNumber.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    get weightSourceID() {
      return this.uniforms.wSourceID.value;
    }

    set weightSourceID(value) {
      if (this.uniforms.wSourceID.value !== value) {
        this.uniforms.wSourceID.value = value;
        this.dispatchEvent({
          type: 'material_property_changed',
          target: this,
        });
      }
    }

    static generateGradientTexture(gradient) {
      var size = 64;

      //Create canvas
      var canvas = document.createElement('canvas');
      canvas.width = size;
      canvas.height = size;

      //Get context
      var context = canvas.getContext('2d');

      //Draw gradient
      context.rect(0, 0, size, size);
      var ctxGradient = context.createLinearGradient(0, 0, size, size);
      for (var i = 0; i < gradient.length; i++) {
        var step = gradient[i];
        ctxGradient.addColorStop(step[0], '#' + step[1].getHexString());
      }
      context.fillStyle = ctxGradient;
      context.fill();

      var texture = new THREE.CanvasTexture(canvas);
      texture.needsUpdate = true;
      texture.minFilter = THREE.LinearFilter;

      return texture;
    }

    static generateClassificationTexture(classification) {
      var width = 256;
      var height = 256;
      var size = width * height;
      var data = new Uint8Array(4 * size);
      for (var x = 0; x < width; x++) {
        for (var y = 0; y < height; y++) {
          var i = x + width * y;
          var color;
          if (classification[x]) {
            color = classification[x];
          } else if (classification[x % 32]) {
            color = classification[x % 32];
          } else {
            color = classification.DEFAULT;
          }
          data[4 * i + 0] = 255 * color.x;
          data[4 * i + 1] = 255 * color.y;
          data[4 * i + 2] = 255 * color.z;
          data[4 * i + 3] = 255 * color.w;
        }
      }
      var texture = new THREE.DataTexture(
        data,
        width,
        height,
        THREE.RGBAFormat
      );
      texture.magFilter = THREE.NearestFilter;
      texture.needsUpdate = true;
      return texture;
    }

    disableEvents() {
      if (this._hiddenListeners === undefined) {
        this._hiddenListeners = this._listeners;
        this._listeners = {};
      }
    }

    enableEvents() {
      this._listeners = this._hiddenListeners;
      this._hiddenListeners = undefined;
    }

    copyFrom(from) {
      for (var name of this.uniforms) {
        this.uniforms[name].value = from.uniforms[name].value;
      }
    }
  }

  class PointCloudOctreeNode extends PointCloudTreeNode {
    constructor() {
      super();

      this.children = {};
      this.sceneNode = null;
      this.octree = null;
    }

    getNumPoints() {
      return this.geometryNode.numPoints;
    }

    isLoaded() {
      return true;
    }

    isTreeNode() {
      return true;
    }

    isGeometryNode() {
      return false;
    }

    getLevel() {
      return this.geometryNode.level;
    }

    getBoundingSphere() {
      return this.geometryNode.boundingSphere;
    }

    getBoundingBox() {
      return this.geometryNode.boundingBox;
    }

    getChildren() {
      var children = [];

      for (var i = 0; i < 8; i++) {
        if (this.children[i]) {
          children.push(this.children[i]);
        }
      }

      return children;
    }

    getPointsInBox(boxNode) {
      if (!this.sceneNode) {
        return null;
      }

      var buffer = this.geometryNode.buffer;

      var posOffset = buffer.offset('position');
      var stride = buffer.stride;
      var view = new DataView(buffer.data);

      var worldToBox = new THREE.Matrix4().getInverse(boxNode.matrixWorld);
      var objectToBox = new THREE.Matrix4().multiplyMatrices(
        worldToBox,
        this.sceneNode.matrixWorld
      );

      var inBox = [];

      var pos = new THREE.Vector4();
      for (var i = 0; i < buffer.numElements; i++) {
        var x = view.getFloat32(i * stride + posOffset + 0, true);
        var y = view.getFloat32(i * stride + posOffset + 4, true);
        var z = view.getFloat32(i * stride + posOffset + 8, true);

        pos.set(x, y, z, 1);
        pos.applyMatrix4(objectToBox);

        if (-0.5 < pos.x && pos.x < 0.5) {
          if (-0.5 < pos.y && pos.y < 0.5) {
            if (-0.5 < pos.z && pos.z < 0.5) {
              pos.set(x, y, z, 1).applyMatrix4(this.sceneNode.matrixWorld);
              inBox.push(new THREE.Vector3(pos.x, pos.y, pos.z));
            }
          }
        }
      }

      return inBox;
    }

    get name() {
      return this.geometryNode.name;
    }
  }
  class PointCloudOctree extends PointCloudTree {
    constructor(geometry, material) {
      super();

      this.pointBudget = Infinity;
      this.pcoGeometry = geometry;
      this.boundingBox = this.pcoGeometry.boundingBox;
      this.boundingSphere = this.boundingBox.getBoundingSphere(
        new THREE.Sphere()
      );
      this.material = material || new PointCloudMaterial();
      this.visiblePointsTarget = 2 * 1000 * 1000;
      this.minimumNodePixelSize = 150;
      this.level = 0;
      this.position.copy(geometry.offset);
      this.updateMatrix();

      this.showBoundingBox = false;
      this.boundingBoxNodes = [];
      this.loadQueue = [];
      this.visibleBounds = new THREE.Box3();
      this.visibleNodes = [];
      this.visibleGeometry = [];
      this.generateDEM = false;
      this.profileRequests = [];
      this.name = '';

      this.tempVector3 = new THREE.Vector3();

      var box = [
        this.pcoGeometry.tightBoundingBox,
        this.getBoundingBoxWorld(),
      ].find(v => v !== undefined);

      this.updateMatrixWorld(true);
      box = HelperUtils.computeTransformedBoundingBox(box, this.matrixWorld);

      var bMin = box.min.z;
      var bMax = box.max.z;
      this.material.heightMin = bMin;
      this.material.heightMax = bMax;

      //TODO <read projection from file instead>
      this.projection = geometry.projection;

      this.root = this.pcoGeometry.root;
    }

    setName(name) {
      if (this.name !== name) {
        this.name = name;
        this.dispatchEvent({
          type: 'name_changed',
          name: name,
          pointcloud: this,
        });
      }
    }

    getName() {
      return this.name;
    }

    toTreeNode(geometryNode, parent) {
      var node = new PointCloudOctreeNode();

      var sceneNode = new THREE.Points(geometryNode.geometry, this.material);
      sceneNode.name = geometryNode.name;
      sceneNode.position.copy(geometryNode.boundingBox.min);
      sceneNode.frustumCulled = true;
      sceneNode.onBeforeRender = (
        _this,
        scene,
        camera,
        geometry,
        material,
        group
      ) => {
        if (material.program) {
          _this.getContext().useProgram(material.program.program);

          if (material.program.getUniforms().map.level) {
            var level = geometryNode.getLevel();
            material.uniforms.level.value = level;
            material.program
              .getUniforms()
              .map.level.setValue(_this.getContext(), level);
          }

          if (
            this.visibleNodeTextureOffsets &&
            material.program.getUniforms().map.vnStart
          ) {
            var vnStart = this.visibleNodeTextureOffsets.get(node);
            material.uniforms.vnStart.value = vnStart;
            material.program
              .getUniforms()
              .map.vnStart.setValue(_this.getContext(), vnStart);
          }

          if (material.program.getUniforms().map.pcIndex) {
            var i = node.pcIndex
              ? node.pcIndex
              : this.visibleNodes.indexOf(node);
            material.uniforms.pcIndex.value = i;
            material.program
              .getUniforms()
              .map.pcIndex.setValue(_this.getContext(), i);
          }
        }
      };

      node.geometryNode = geometryNode;
      node.sceneNode = sceneNode;
      node.pointcloud = this;
      node.children = {};
      for (var key in geometryNode.children) {
        node.children[key] = geometryNode.children[key];
      }

      if (!parent) {
        this.root = node;
        this.add(sceneNode);
      } else {
        var childIndex = parseInt(
          geometryNode.name[geometryNode.name.length - 1]
        );
        parent.sceneNode.add(sceneNode);
        parent.children[childIndex] = node;
      }

      var disposeListener = function() {
        // EDITED
        let childIndex = parseInt(
          geometryNode.name[geometryNode.name.length - 1]
        );
        //remove node from scene
        parent.sceneNode.remove(sceneNode);

        //check if has geometry it could be removed also...
        if (sceneNode.geometry) {
          //delete attributes solve a big memory leak...
          let attributes = sceneNode.geometry.attributes;
          for (let key in attributes) {
            if (key == 'position') {
              delete attributes[key].array;
            }
            delete attributes[key];
          }
          //dispose geometry
          sceneNode.geometry.dispose();
          sceneNode.geometry = undefined;
        }

        //check if has material, can be removed...
        if (sceneNode.material) {
          //check if has material map, can be removed...
          if (sceneNode.material.map) {
            sceneNode.material.map.dispose();
            sceneNode.material.map = undefined;
          }
          //dispose material
          sceneNode.material.dispose();
          sceneNode.material = undefined;
        }

        //delete matrix
        delete sceneNode.matrix;
        //delete matrixWorld
        delete sceneNode.matrixWorld;
        //delete position
        delete sceneNode.position.array;
        //delete qa
        delete sceneNode.quaternion.array;
        //delete rotation
        delete sceneNode.rotation.array;
        //delete scale
        delete sceneNode.scale.array;
        //delete up
        delete sceneNode.up.array;
        //delete sceneNode
        sceneNode = undefined;
        parent.children[childIndex] = geometryNode;
      };
      geometryNode.oneTimeDisposeHandlers.push(disposeListener);

      return node;
    }

    updateVisibleBounds() {
      var leafNodes = [];
      for (var i = 0; i < this.visibleNodes.length; i++) {
        var node = this.visibleNodes[i];
        var isLeaf = true;

        for (var j = 0; j < node.children.length; j++) {
          var child = node.children[j];
          if (child instanceof PointCloudOctreeNode) {
            isLeaf = isLeaf && !child.sceneNode.visible;
          } else if (child instanceof PointCloudOctreeGeometryNode) {
            isLeaf = true;
          }
        }

        if (isLeaf) {
          leafNodes.push(node);
        }
      }

      this.visibleBounds.min = new THREE.Vector3(Infinity, Infinity, Infinity);
      this.visibleBounds.max = new THREE.Vector3(
        -Infinity,
        -Infinity,
        -Infinity
      );

      for (var i = 0; i < leafNodes.length; i++) {
        var node = leafNodes[i];
        this.visibleBounds.expandByPoint(node.getBoundingBox().min);
        this.visibleBounds.expandByPoint(node.getBoundingBox().max);
      }
    }

    updateMaterial(material, visibleNodes, camera, renderer) {
      material.fov = camera.fov * (Math.PI / 180);
      material.screenWidth = renderer.domElement.clientWidth;
      material.screenHeight = renderer.domElement.clientHeight;
      material.spacing =
        this.pcoGeometry.spacing *
        Math.max(this.scale.x, this.scale.y, this.scale.z);
      material.near = camera.near;
      material.far = camera.far;
      material.uniforms.octreeSize.value = this.pcoGeometry.boundingBox.getSize(
        new THREE.Vector3()
      ).x;
    }

    computeVisibilityTextureData(nodes, camera) {
      if (Global.measureTimings) {
        performance.mark('computeVisibilityTextureData-start');
      }

      var data = new Uint8Array(nodes.length * 4);
      var visibleNodeTextureOffsets = new Map();

      //copy array
      nodes = nodes.slice();

      //sort by level and index, e.g. r, r0, r3, r4, r01, r07, r30, ...
      var sort = function(a, b) {
        var na = a.geometryNode.name;
        var nb = b.geometryNode.name;
        if (na.length !== nb.length) return na.length - nb.length;
        if (na < nb) return -1;
        if (na > nb) return 1;
        return 0;
      };
      nodes.sort(sort);

      //code sample taken from three.js src/math/Ray.js
      var v1 = new THREE.Vector3();
      var intersectSphereBack = (ray, sphere) => {
        v1.subVectors(sphere.center, ray.origin);
        var tca = v1.dot(ray.direction);
        var d2 = v1.dot(v1) - tca * tca;
        var radius2 = sphere.radius * sphere.radius;

        if (d2 > radius2) {
          return null;
        }

        var thc = Math.sqrt(radius2 - d2);

        //t1 = second intersect point - exit point on back of sphere
        var t1 = tca + thc;

        if (t1 < 0) {
          return null;
        }

        return t1;
      };

      var lodRanges = new Map();
      var leafNodeLodRanges = new Map();

      for (var i = 0; i < nodes.length; i++) {
        var node = nodes[i];

        visibleNodeTextureOffsets.set(node, i);

        var children = [];
        for (var j = 0; j < 8; j++) {
          var child = node.children[j];

          if (
            child &&
            child.constructor === PointCloudOctreeNode &&
            nodes.includes(child, i)
          ) {
            children.push(child);
          }
        }

        var spacing = node.geometryNode.estimatedSpacing;

        data[i * 4 + 0] = 0;
        data[i * 4 + 1] = 0;
        data[i * 4 + 2] = 0;
        data[i * 4 + 3] = node.getLevel();
        for (var j = 0; j < children.length; j++) {
          var child = children[j];
          var index = parseInt(child.geometryNode.name.substr(-1));
          data[i * 4 + 0] += Math.pow(2, index);

          if (j === 0) {
            var vArrayIndex = nodes.indexOf(child, i);

            data[i * 4 + 1] = (vArrayIndex - i) >> 8;
            data[i * 4 + 2] = (vArrayIndex - i) % 256;
          }
        }

        //TODO performance optimization
        //for some reason, this part can be extremely slow in chrome during a debugging session, but not during profiling
        var bBox = node.getBoundingBox().clone();
        //bBox.applyMatrix4(node.sceneNode.matrixWorld);
        //bBox.applyMatrix4(camera.matrixWorldInverse);
        var bSphere = bBox.getBoundingSphere(new THREE.Sphere());
        bSphere.applyMatrix4(node.sceneNode.matrixWorld);
        bSphere.applyMatrix4(camera.matrixWorldInverse);

        var ray = new THREE.Ray(
          camera.position,
          camera.getWorldDirection(this.tempVector3)
        );
        var distance = intersectSphereBack(ray, bSphere);
        var distance2 =
          bSphere.center.distanceTo(camera.position) + bSphere.radius;
        if (distance === null) {
          distance = distance2;
        }
        distance = Math.max(distance, distance2);

        if (!lodRanges.has(node.getLevel())) {
          lodRanges.set(node.getLevel(), distance);
        } else {
          var prevDistance = lodRanges.get(node.getLevel());
          var newDistance = Math.max(prevDistance, distance);
          lodRanges.set(node.getLevel(), newDistance);
        }

        if (!node.geometryNode.hasChildren) {
          var value = {
            distance: distance,
            i: i,
          };
          leafNodeLodRanges.set(node, value);
        }
      }

      for (var [node, value] of leafNodeLodRanges) {
        var level = node.getLevel();
        var distance = value.distance;
        var i = value.i;

        if (level < 4) {
          continue;
        }
        for (var [lod, range] of lodRanges) {
          if (distance < range * 1.2) {
            data[i * 4 + 3] = lod;
          }
        }
      }

      if (Global.measureTimings) {
        performance.mark('computeVisibilityTextureData-end');
        performance.measure(
          'render.computeVisibilityTextureData',
          'computeVisibilityTextureData-start',
          'computeVisibilityTextureData-end'
        );
      }

      return {
        data: data,
        offsets: visibleNodeTextureOffsets,
      };
    }

    nodeIntersectsProfile(node, profile) {
      var bbWorld = node.boundingBox.clone().applyMatrix4(this.matrixWorld);
      var bsWorld = bbWorld.getBoundingSphere(new THREE.Sphere());

      var intersects = false;

      for (var i = 0; i < profile.points.length - 1; i++) {
        var start = new THREE.Vector3(
          profile.points[i + 0].x,
          profile.points[i + 0].y,
          bsWorld.center.z
        );
        var end = new THREE.Vector3(
          profile.points[i + 1].x,
          profile.points[i + 1].y,
          bsWorld.center.z
        );

        var closest = new THREE.Line3(start, end).closestPointToPoint(
          bsWorld.center,
          true
        );
        var distance = closest.distanceTo(bsWorld.center);

        intersects = intersects || distance < bsWorld.radius + profile.width;
      }

      return intersects;
    }

    nodesOnRay(nodes, ray) {
      var nodesOnRay = [];

      var _ray = ray.clone();
      for (var i = 0; i < nodes.length; i++) {
        var node = nodes[i];
        //var inverseWorld = new THREE.Matrix4().getInverse(node.matrixWorld);
        //var sphere = node.getBoundingSphere(new THREE.Sphere()).clone().applyMatrix4(node.sceneNode.matrixWorld);
        var sphere = node
          .getBoundingSphere(new THREE.Sphere())
          .clone()
          .applyMatrix4(this.matrixWorld);

        if (_ray.intersectsSphere(sphere)) {
          nodesOnRay.push(node);
        }
      }

      return nodesOnRay;
    }

    updateMatrixWorld(force) {
      if (this.matrixAutoUpdate === true) this.updateMatrix();

      if (this.matrixWorldNeedsUpdate === true || force === true) {
        if (!this.parent) {
          this.matrixWorld.copy(this.matrix);
        } else {
          this.matrixWorld.multiplyMatrices(
            this.parent.matrixWorld,
            this.matrix
          );
        }

        this.matrixWorldNeedsUpdate = false;

        force = true;
      }
    }

    hideDescendants(object) {
      var stack = [];
      for (var i = 0; i < object.children.length; i++) {
        var child = object.children[i];
        if (child.visible) {
          stack.push(child);
        }
      }

      while (stack.length > 0) {
        var object = stack.shift();

        object.visible = false;

        for (var i = 0; i < object.children.length; i++) {
          var child = object.children[i];
          if (child.visible) {
            stack.push(child);
          }
        }
      }
    }

    moveToOrigin() {
      this.position.set(0, 0, 0);
      this.updateMatrixWorld(true);
      var box = this.boundingBox;
      var transform = this.matrixWorld;
      var tBox = HelperUtils.computeTransformedBoundingBox(box, transform);

      this.position.set(0, 0, 0).sub(tBox.getCenter(new THREE.Vector3()));
    }

    moveToGroundPlane() {
      this.updateMatrixWorld(true);
      var box = this.boundingBox;
      var transform = this.matrixWorld;
      var tBox = HelperUtils.computeTransformedBoundingBox(box, transform);
      this.position.y += -tBox.min.y;
    }

    getBoundingBoxWorld() {
      this.updateMatrixWorld(true);
      var box = this.boundingBox;
      var transform = this.matrixWorld;
      var tBox = HelperUtils.computeTransformedBoundingBox(box, transform);

      return tBox;
    }

    /**
     * returns points inside the profile points
     *
     * maxDepth:		search points up to the given octree depth
     *
     *
     * The return value is an array with all segments of the profile path
     *  var segment = {
     * 		start: 	THREE.Vector3,
     * 		end: 	THREE.Vector3,
     * 		points: {}
     * 		project: function()
     *  };
     *
     * The project() function inside each segment can be used to transform
     * that segments point coordinates to line up along the x-axis.
     *
     *
     */
    getPointsInProfile(profile, maxDepth, callback) {
      var points = {
        segments: [],
        boundingBox: new THREE.Box3(),
        projectedBoundingBox: new THREE.Box2(),
      };

      //evaluate segments
      for (var i = 0; i < profile.points.length - 1; i++) {
        var start = profile.points[i];
        var end = profile.points[i + 1];
        var ps = this.getProfile(start, end, profile.width, maxDepth);

        var segment = {
          start: start,
          end: end,
          points: ps,
          project: null,
        };

        points.segments.push(segment);

        points.boundingBox.expandByPoint(ps.boundingBox.min);
        points.boundingBox.expandByPoint(ps.boundingBox.max);
      }

      //add projection functions to the segments
      var mileage = new THREE.Vector3();
      for (var i = 0; i < points.segments.length; i++) {
        var segment = points.segments[i];
        var start = segment.start;
        var end = segment.end;

        var project = (function(_start, _end, _mileage, _boundingBox) {
          var start = _start;
          var end = _end;
          var mileage = _mileage;
          var boundingBox = _boundingBox;

          var xAxis = new THREE.Vector3(1, 0, 0);
          var dir = new THREE.Vector3().subVectors(end, start);
          dir.y = 0;
          dir.normalize();
          var alpha = Math.acos(xAxis.dot(dir));
          if (dir.z > 0) {
            alpha = -alpha;
          }

          return function(position) {
            var toOrigin = new THREE.Matrix4().makeTranslation(
              -start.x,
              -boundingBox.min.y,
              -start.z
            );
            var alignWithX = new THREE.Matrix4().makeRotationY(-alpha);
            var applyMileage = new THREE.Matrix4().makeTranslation(
              mileage.x,
              0,
              0
            );

            var pos = position.clone();
            pos.applyMatrix4(toOrigin);
            pos.applyMatrix4(alignWithX);
            pos.applyMatrix4(applyMileage);

            return pos;
          };
        })(start, end, mileage.clone(), points.boundingBox.clone());

        segment.project = project;

        mileage.x += new THREE.Vector3(start.x, 0, start.z).distanceTo(
          new THREE.Vector3(end.x, 0, end.z)
        );
        mileage.y += end.y - start.y;
      }

      points.projectedBoundingBox.min.x = 0;
      points.projectedBoundingBox.min.y = points.boundingBox.min.y;
      points.projectedBoundingBox.max.x = mileage.x;
      points.projectedBoundingBox.max.y = points.boundingBox.max.y;

      return points;
    }

    /**
     * returns points inside the given profile bounds.
     *
     * start:
     * end:
     * width:
     * depth:		search points up to the given octree depth
     * callback:	if specified, points are loaded before searching
     *
     *
     */
    getProfile(start, end, width, depth, callback) {
      //var request = new Potree.ProfileRequest(start, end, width, depth, callback);
      //this.profileRequests.push(request);
    }

    getVisibleExtent() {
      return this.visibleBounds.applyMatrix4(this.matrixWorld);
    }

    /**
     *
     *
     *
     * params.pickWindowSize:	Look for points inside a pixel window of this size.
     * 							Use odd values: 1, 3, 5, ...
     *
     *
     * TODO: only draw pixels that are actually read with readPixels().
     *
     */
    pick(viewer, camera, ray, params = {}) {
      var renderer = viewer.renderer;
      var pRenderer = viewer.pRenderer;

      performance.mark('pick-start');

      var getVal = (a, b) => (a !== undefined ? a : b);

      var pickWindowSize = getVal(params.pickWindowSize, 17);
      var pickOutsideClipRegion = getVal(params.pickOutsideClipRegion, false);

      var size = renderer.getSize(new THREE.Vector3());

      var width = Math.ceil(getVal(params.width, size.width));
      var height = Math.ceil(getVal(params.height, size.height));

      var pointSizeType = getVal(
        params.pointSizeType,
        this.material.pointSizeType
      );
      var pointSize = getVal(params.pointSize, this.material.size);

      var nodes = this.nodesOnRay(this.visibleNodes, ray);

      if (nodes.length === 0) {
        return null;
      }

      if (!this.pickState) {
        var scene = new THREE.Scene();

        var material = new PointCloudMaterial();
        material.pointColorType = PointColorType.POINT_INDEX;

        var renderTarget = new THREE.WebGLRenderTarget(1, 1, {
          minFilter: THREE.LinearFilter,
          magFilter: THREE.NearestFilter,
          format: THREE.RGBAFormat,
        });

        this.pickState = {
          renderTarget: renderTarget,
          material: material,
          scene: scene,
        };
      }
      var pickState = this.pickState;
      var pickMaterial = pickState.material;

      //Update pick material
      pickMaterial.pointSizeType = pointSizeType;
      pickMaterial.shape = this.material.shape;

      pickMaterial.size = pointSize;
      pickMaterial.uniforms.minSize.value = this.material.uniforms.minSize.value;
      pickMaterial.uniforms.maxSize.value = this.material.uniforms.maxSize.value;
      pickMaterial.classification = this.material.classification;
      if (params.pickClipped) {
        pickMaterial.clipBoxes = this.material.clipBoxes;
        if (this.material.clipTask === ClipTask.HIGHLIGHT) {
          pickMaterial.clipTask = ClipTask.NONE;
        } else {
          pickMaterial.clipTask = this.material.clipTask;
        }
      } else {
        pickMaterial.clipBoxes = [];
      }

      this.updateMaterial(pickMaterial, nodes, camera, renderer);

      pickState.renderTarget.setSize(width, height);

      var pixelPos = new THREE.Vector2(params.x, params.y);

      var gl = renderer.getContext();
      gl.enable(gl.SCISSOR_TEST);
      gl.scissor(
        parseInt(pixelPos.x - (pickWindowSize - 1) / 2),
        parseInt(pixelPos.y - (pickWindowSize - 1) / 2),
        parseInt(pickWindowSize),
        parseInt(pickWindowSize)
      );

      renderer.state.buffers.depth.setTest(pickMaterial.depthTest);
      renderer.state.buffers.depth.setMask(pickMaterial.depthWrite);
      renderer.state.setBlending(THREE.NoBlending);

      //Render
      renderer.setRenderTarget(pickState.renderTarget);
      gl.clearColor(0, 0, 0, 0);
      renderer.clearTarget(pickState.renderTarget, true, true, true);

      var tmp = this.material;
      this.material = pickMaterial;

      pRenderer.renderOctree(this, nodes, camera, pickState.renderTarget);

      this.material = tmp;

      var clamp = (number, min, max) => Math.min(Math.max(min, number), max);

      var x = parseInt(clamp(pixelPos.x - (pickWindowSize - 1) / 2, 0, width));
      var y = parseInt(clamp(pixelPos.y - (pickWindowSize - 1) / 2, 0, height));
      var w = parseInt(Math.min(x + pickWindowSize, width) - x);
      var h = parseInt(Math.min(y + pickWindowSize, height) - y);

      var pixelCount = w * h;
      var buffer = new Uint8Array(4 * pixelCount);

      gl.readPixels(
        x,
        y,
        pickWindowSize,
        pickWindowSize,
        gl.RGBA,
        gl.UNSIGNED_BYTE,
        buffer
      );

      renderer.setRenderTarget(null);
      renderer.resetGLState();
      renderer.setScissorTest(false);
      gl.disable(gl.SCISSOR_TEST);

      var pixels = buffer;
      var ibuffer = new Uint32Array(buffer.buffer);
      var hits = [];

      for (var u = 0; u < pickWindowSize; u++) {
        for (var v = 0; v < pickWindowSize; v++) {
          var offset = u + v * pickWindowSize;
          var distance =
            Math.pow(u - (pickWindowSize - 1) / 2, 2) +
            Math.pow(v - (pickWindowSize - 1) / 2, 2);

          var pcIndex = pixels[4 * offset + 3];
          pixels[4 * offset + 3] = 0;
          var pIndex = ibuffer[offset];

          if (
            !(pcIndex === 0 && pIndex === 0) &&
            pcIndex !== undefined &&
            pIndex !== undefined
          ) {
            var hit = {
              pIndex: pIndex,
              pcIndex: pcIndex,
              distanceToCenter: distance,
            };

            if (params.all) {
              hits.push(hit);
            } else {
              if (hits.length > 0) {
                if (distance < hits[0].distanceToCenter) {
                  hits[0] = hit;
                }
              } else {
                hits.push(hit);
              }
            }
          }
        }
      }

      for (var hit of hits) {
        var point = {};

        if (!nodes[hit.pcIndex]) {
          return null;
        }

        var node = nodes[hit.pcIndex];
        var pc = node.sceneNode;
        var geometry = node.geometryNode.geometry;

        for (var attributeName in geometry.attributes) {
          var attribute = geometry.attributes[attributeName];

          if (attributeName === 'position') {
            var x = attribute.array[3 * hit.pIndex + 0];
            var y = attribute.array[3 * hit.pIndex + 1];
            var z = attribute.array[3 * hit.pIndex + 2];

            var position = new THREE.Vector3(x, y, z);
            position.applyMatrix4(pc.matrixWorld);

            point[attributeName] = position;
          }

          /*
          else if(attributeName === "indices")
          {

          }
          else
          {
            //if (values.itemSize === 1) {
            //	point[attribute.name] = values.array[hit.pIndex];
            //} else {
            //	var value = [];
            //	for (var j = 0; j < values.itemSize; j++) {
            //		value.push(values.array[values.itemSize * hit.pIndex + j]);
            //	}
            //	point[attribute.name] = value;
            //}
          }
          */
        }

        hit.point = point;
      }

      performance.mark('pick-end');
      performance.measure('pick', 'pick-start', 'pick-end');

      if (params.all) {
        return hits.map(hit => hit.point);
      } else {
        if (hits.length === 0) {
          return null;
        } else {
          return hits[0].point;
          //var sorted = hits.sort((a, b) => a.distanceToCenter - b.distanceToCenter);
          //return sorted[0].point;
        }
      }
    }

    *getFittedBoxGen(boxNode) {
      var shrinkedLocalBounds = new THREE.Box3();
      var worldToBox = new THREE.Matrix4().getInverse(boxNode.matrixWorld);

      for (var node of this.visibleNodes) {
        if (!node.sceneNode) {
          continue;
        }

        var buffer = node.geometryNode.buffer;

        var posOffset = buffer.offset('position');
        var stride = buffer.stride;
        var view = new DataView(buffer.data);

        var objectToBox = new THREE.Matrix4().multiplyMatrices(
          worldToBox,
          node.sceneNode.matrixWorld
        );

        var pos = new THREE.Vector4();
        for (var i = 0; i < buffer.numElements; i++) {
          var x = view.getFloat32(i * stride + posOffset + 0, true);
          var y = view.getFloat32(i * stride + posOffset + 4, true);
          var z = view.getFloat32(i * stride + posOffset + 8, true);

          pos.set(x, y, z, 1);
          pos.applyMatrix4(objectToBox);

          if (-0.5 < pos.x && pos.x < 0.5) {
            if (-0.5 < pos.y && pos.y < 0.5) {
              if (-0.5 < pos.z && pos.z < 0.5) {
                shrinkedLocalBounds.expandByPoint(pos);
              }
            }
          }
        }

        yield;
      }

      var fittedPosition = shrinkedLocalBounds
        .getCenter(new THREE.Vector3())
        .applyMatrix4(boxNode.matrixWorld);

      var fitted = new THREE.Object3D();
      fitted.position.copy(fittedPosition);
      fitted.scale.copy(boxNode.scale);
      fitted.rotation.copy(boxNode.rotation);

      var ds = new THREE.Vector3().subVectors(
        shrinkedLocalBounds.max,
        shrinkedLocalBounds.min
      );
      fitted.scale.multiply(ds);

      yield fitted;
    }

    getFittedBox(boxNode, maxLevel = Infinity) {
      var shrinkedLocalBounds = new THREE.Box3();
      var worldToBox = new THREE.Matrix4().getInverse(boxNode.matrixWorld);

      for (var node of this.visibleNodes) {
        if (!node.sceneNode || node.getLevel() > maxLevel) {
          continue;
        }

        var buffer = node.geometryNode.buffer;

        var posOffset = buffer.offset('position');
        var stride = buffer.stride;
        var view = new DataView(buffer.data);

        var objectToBox = new THREE.Matrix4().multiplyMatrices(
          worldToBox,
          node.sceneNode.matrixWorld
        );

        var pos = new THREE.Vector4();
        for (var i = 0; i < buffer.numElements; i++) {
          var x = view.getFloat32(i * stride + posOffset + 0, true);
          var y = view.getFloat32(i * stride + posOffset + 4, true);
          var z = view.getFloat32(i * stride + posOffset + 8, true);

          pos.set(x, y, z, 1);
          pos.applyMatrix4(objectToBox);

          if (-0.5 < pos.x && pos.x < 0.5) {
            if (-0.5 < pos.y && pos.y < 0.5) {
              if (-0.5 < pos.z && pos.z < 0.5) {
                shrinkedLocalBounds.expandByPoint(pos);
              }
            }
          }
        }
      }

      var fittedPosition = shrinkedLocalBounds
        .getCenter(new THREE.Vector3())
        .applyMatrix4(boxNode.matrixWorld);

      var fitted = new THREE.Object3D();
      fitted.position.copy(fittedPosition);
      fitted.scale.copy(boxNode.scale);
      fitted.rotation.copy(boxNode.rotation);

      var ds = new THREE.Vector3().subVectors(
        shrinkedLocalBounds.max,
        shrinkedLocalBounds.min
      );
      fitted.scale.multiply(ds);

      return fitted;
    }

    get progress() {
      return this.visibleNodes.length / this.visibleGeometry.length;
    }

    find(name) {
      var node = null;
      for (var char of name) {
        if (char === 'r') {
          node = this.root;
        } else {
          node = node.children[char];
        }
      }

      return node;
    }
  }

  class PointCloudArena4DNode extends PointCloudTreeNode {
    constructor() {
      super();

      this.left = null;
      this.right = null;
      this.sceneNode = null;
      this.kdtree = null;
    }

    getNumPoints() {
      return this.geometryNode.numPoints;
    }

    isLoaded() {
      return true;
    }

    isTreeNode() {
      return true;
    }

    isGeometryNode() {
      return false;
    }

    getLevel() {
      return this.geometryNode.level;
    }

    getBoundingSphere() {
      return this.geometryNode.boundingSphere;
    }

    getBoundingBox() {
      return this.geometryNode.boundingBox;
    }

    toTreeNode(child) {
      var geometryNode = null;

      if (this.left === child) {
        geometryNode = this.left;
      } else if (this.right === child) {
        geometryNode = this.right;
      }

      if (!geometryNode.loaded) {
        return;
      }

      var node = new PointCloudArena4DNode();
      var sceneNode = THREE.PointCloud(
        geometryNode.geometry,
        this.kdtree.material
      );
      sceneNode.visible = false;

      node.kdtree = this.kdtree;
      node.geometryNode = geometryNode;
      node.sceneNode = sceneNode;
      node.parent = this;
      node.left = this.geometryNode.left;
      node.right = this.geometryNode.right;
    }

    getChildren() {
      var children = [];

      if (this.left) {
        children.push(this.left);
      }

      if (this.right) {
        children.push(this.right);
      }

      return children;
    }
  }
  class PointCloudArena4D extends PointCloudTree {
    constructor(geometry) {
      super();

      this.root = null;
      if (geometry.root) {
        this.root = geometry.root;
      } else {
        geometry.addEventListener('hierarchy_loaded', () => {
          this.root = geometry.root;
        });
      }

      this.visiblePointsTarget = 2 * 1000 * 1000;
      this.minimumNodePixelSize = 150;

      this.position.sub(geometry.offset);
      this.updateMatrix();

      this.numVisibleNodes = 0;
      this.numVisiblePoints = 0;

      this.boundingBoxNodes = [];
      this.loadQueue = [];
      this.visibleNodes = [];

      this.pcoGeometry = geometry;
      this.boundingBox = this.pcoGeometry.boundingBox;
      this.boundingSphere = this.pcoGeometry.boundingSphere;
      this.material = new PointCloudMaterial({
        vertexColors: THREE.VertexColors,
        size: 0.05,
        treeType: TreeType.KDTREE,
      });
      this.material.sizeType = PointSizeType.ATTENUATED;
      this.material.size = 0.05;
      this.profileRequests = [];
      this.name = '';
    }

    getBoundingBoxWorld() {
      this.updateMatrixWorld(true);
      var box = this.boundingBox;
      var transform = this.matrixWorld;
      var tBox = HelperUtils.computeTransformedBoundingBox(box, transform);

      return tBox;
    }

    setName(name) {
      if (this.name !== name) {
        this.name = name;
        this.dispatchEvent({
          type: 'name_changed',
          name: name,
          pointcloud: this,
        });
      }
    }

    getName() {
      return this.name;
    }

    getLevel() {
      return this.level;
    }

    toTreeNode(geometryNode, parent) {
      var node = new PointCloudArena4DNode();

      var sceneNode = new THREE.Points(geometryNode.geometry, this.material);
      sceneNode.frustumCulled = true;
      sceneNode.onBeforeRender = (
        _this,
        scene,
        camera,
        geometry,
        material,
        group
      ) => {
        if (material.program) {
          _this.getContext().useProgram(material.program.program);

          if (material.program.getUniforms().map.level) {
            var level = geometryNode.getLevel();
            material.uniforms.level.value = level;
            material.program
              .getUniforms()
              .map.level.setValue(_this.getContext(), level);
          }

          if (
            this.visibleNodeTextureOffsets &&
            material.program.getUniforms().map.vnStart
          ) {
            var vnStart = this.visibleNodeTextureOffsets.get(node);
            material.uniforms.vnStart.value = vnStart;
            material.program
              .getUniforms()
              .map.vnStart.setValue(_this.getContext(), vnStart);
          }

          if (material.program.getUniforms().map.pcIndex) {
            var i = node.pcIndex
              ? node.pcIndex
              : this.visibleNodes.indexOf(node);
            material.uniforms.pcIndex.value = i;
            material.program
              .getUniforms()
              .map.pcIndex.setValue(_this.getContext(), i);
          }
        }
      };

      node.geometryNode = geometryNode;
      node.sceneNode = sceneNode;
      node.pointcloud = this;
      node.left = geometryNode.left;
      node.right = geometryNode.right;

      if (!parent) {
        this.root = node;
        this.add(sceneNode);
      } else {
        parent.sceneNode.add(sceneNode);

        if (parent.left === geometryNode) {
          parent.left = node;
        } else if (parent.right === geometryNode) {
          parent.right = node;
        }
      }

      var disposeListener = function() {
        parent.sceneNode.remove(node.sceneNode);

        if (parent.left === node) {
          parent.left = geometryNode;
        } else if (parent.right === node) {
          parent.right = geometryNode;
        }
      };
      geometryNode.oneTimeDisposeHandlers.push(disposeListener);

      return node;
    }

    updateMaterial(material, visibleNodes, camera, renderer) {
      material.fov = camera.fov * (Math.PI / 180);
      material.screenWidth = renderer.domElement.clientWidth;
      material.screenHeight = renderer.domElement.clientHeight;
      material.spacing = this.pcoGeometry.spacing;
      material.near = camera.near;
      material.far = camera.far;

      //reduce shader source updates by setting maxLevel slightly higher than actually necessary
      if (this.maxLevel > material.levels) {
        material.levels = this.maxLevel + 2;
      }

      //material.uniforms.octreeSize.value = this.boundingBox.size().x;
      var bbSize = this.boundingBox.getSize(new THREE.Vector3());
      material.bbSize = [bbSize.x, bbSize.y, bbSize.z];
    }

    updateVisibleBounds() {}

    hideDescendants(object) {
      var stack = [];
      for (var i = 0; i < object.children.length; i++) {
        var child = object.children[i];
        if (child.visible) {
          stack.push(child);
        }
      }

      while (stack.length > 0) {
        var child = stack.shift();

        child.visible = false;
        if (child.boundingBoxNode) {
          child.boundingBoxNode.visible = false;
        }

        for (var i = 0; i < child.children.length; i++) {
          var childOfChild = child.children[i];
          if (childOfChild.visible) {
            stack.push(childOfChild);
          }
        }
      }
    }

    updateMatrixWorld(force) {
      //node.matrixWorld.multiplyMatrices( node.parent.matrixWorld, node.matrix );

      if (this.matrixAutoUpdate === true) this.updateMatrix();

      if (this.matrixWorldNeedsUpdate === true || force === true) {
        if (this.parent === undefined) {
          this.matrixWorld.copy(this.matrix);
        } else {
          this.matrixWorld.multiplyMatrices(
            this.parent.matrixWorld,
            this.matrix
          );
        }

        this.matrixWorldNeedsUpdate = false;

        force = true;
      }
    }

    nodesOnRay(nodes, ray) {
      var nodesOnRay = [];

      var _ray = ray.clone();
      for (var i = 0; i < nodes.length; i++) {
        var node = nodes[i];
        var sphere = node
          .getBoundingSphere(new THREE.Sphere())
          .clone()
          .applyMatrix4(node.sceneNode.matrixWorld);
        //TODO Unused: var box = node.getBoundingBox().clone().applyMatrix4(node.sceneNode.matrixWorld);

        if (_ray.intersectsSphere(sphere)) {
          nodesOnRay.push(node);
        }
        //if(_ray.isIntersectionBox(box)){
        //	nodesOnRay.push(node);
        //}
      }

      return nodesOnRay;
    }

    pick(viewer, camera, ray, params = {}) {
      var renderer = viewer.renderer;
      var pRenderer = viewer.pRenderer;

      performance.mark('pick-start');

      var getVal = (a, b) => (a !== undefined ? a : b);

      var pickWindowSize = getVal(params.pickWindowSize, 17);
      var pickOutsideClipRegion = getVal(params.pickOutsideClipRegion, false);

      var size = renderer.getSize(new THREE.Vector3());

      var width = Math.ceil(getVal(params.width, size.width));
      var height = Math.ceil(getVal(params.height, size.height));

      var pointSizeType = getVal(
        params.pointSizeType,
        this.material.pointSizeType
      );
      var pointSize = getVal(params.pointSize, this.material.size);

      var nodes = this.nodesOnRay(this.visibleNodes, ray);

      if (nodes.length === 0) {
        return null;
      }

      if (!this.pickState) {
        var scene = new THREE.Scene();

        var material = new PointCloudMaterial();
        material.pointColorType = PointColorType.POINT_INDEX;

        var renderTarget = new THREE.WebGLRenderTarget(1, 1, {
          minFilter: THREE.LinearFilter,
          magFilter: THREE.NearestFilter,
          format: THREE.RGBAFormat,
        });

        this.pickState = {
          renderTarget: renderTarget,
          material: material,
          scene: scene,
        };
      }
      var pickState = this.pickState;
      var pickMaterial = pickState.material;
      pickMaterial.pointSizeType = pointSizeType;
      pickMaterial.shape = this.material.shape;

      pickMaterial.size = pointSize;
      pickMaterial.uniforms.minSize.value = this.material.uniforms.minSize.value;
      pickMaterial.uniforms.maxSize.value = this.material.uniforms.maxSize.value;
      pickMaterial.classification = this.material.classification;
      if (params.pickClipped) {
        pickMaterial.clipBoxes = this.material.clipBoxes;
        if (this.material.clipTask === ClipTask.HIGHLIGHT) {
          pickMaterial.clipTask = ClipTask.NONE;
        } else {
          pickMaterial.clipTask = this.material.clipTask;
        }
      } else {
        pickMaterial.clipBoxes = [];
      }

      this.updateMaterial(pickMaterial, nodes, camera, renderer);

      pickState.renderTarget.setSize(width, height);

      var pixelPos = new THREE.Vector2(params.x, params.y);

      var gl = renderer.getContext();
      gl.enable(gl.SCISSOR_TEST);
      gl.scissor(
        parseInt(pixelPos.x - (pickWindowSize - 1) / 2),
        parseInt(pixelPos.y - (pickWindowSize - 1) / 2),
        parseInt(pickWindowSize),
        parseInt(pickWindowSize)
      );

      renderer.state.buffers.depth.setTest(pickMaterial.depthTest);
      renderer.state.buffers.depth.setMask(pickMaterial.depthWrite);
      renderer.state.setBlending(THREE.NoBlending);

      renderer.clearTarget(pickState.renderTarget, true, true, true);
      renderer.setRenderTarget(pickState.renderTarget);

      gl.clearColor(0, 0, 0, 0);
      renderer.clearTarget(pickState.renderTarget, true, true, true);

      var tmp = this.material;
      this.material = pickMaterial;

      pRenderer.renderOctree(this, nodes, camera, pickState.renderTarget);

      this.material = tmp;

      var clamp = (number, min, max) => Math.min(Math.max(min, number), max);

      var x = parseInt(clamp(pixelPos.x - (pickWindowSize - 1) / 2, 0, width));
      var y = parseInt(clamp(pixelPos.y - (pickWindowSize - 1) / 2, 0, height));
      var w = parseInt(Math.min(x + pickWindowSize, width) - x);
      var h = parseInt(Math.min(y + pickWindowSize, height) - y);

      var pixelCount = w * h;
      var buffer = new Uint8Array(4 * pixelCount);

      gl.readPixels(
        x,
        y,
        pickWindowSize,
        pickWindowSize,
        gl.RGBA,
        gl.UNSIGNED_BYTE,
        buffer
      );

      renderer.setRenderTarget(null);
      renderer.resetGLState();
      renderer.setScissorTest(false);
      gl.disable(gl.SCISSOR_TEST);

      var pixels = buffer;
      var ibuffer = new Uint32Array(buffer.buffer);
      var hits = [];
      for (var u = 0; u < pickWindowSize; u++) {
        for (var v = 0; v < pickWindowSize; v++) {
          var offset = u + v * pickWindowSize;
          var distance =
            Math.pow(u - (pickWindowSize - 1) / 2, 2) +
            Math.pow(v - (pickWindowSize - 1) / 2, 2);

          var pcIndex = pixels[4 * offset + 3];
          pixels[4 * offset + 3] = 0;
          var pIndex = ibuffer[offset];

          if (
            !(pcIndex === 0 && pIndex === 0) &&
            pcIndex !== undefined &&
            pIndex !== undefined
          ) {
            var hit = {
              pIndex: pIndex,
              pcIndex: pcIndex,
              distanceToCenter: distance,
            };

            if (params.all) {
              hits.push(hit);
            } else {
              if (hits.length > 0) {
                if (distance < hits[0].distanceToCenter) {
                  hits[0] = hit;
                }
              } else {
                hits.push(hit);
              }
            }
          }
        }
      }

      for (var hit of hits) {
        var point = {};

        if (!nodes[hit.pcIndex]) {
          return null;
        }

        var node = nodes[hit.pcIndex];
        var pc = node.sceneNode;
        var geometry = node.geometryNode.geometry;

        for (var attributeName in geometry.attributes) {
          var attribute = geometry.attributes[attributeName];

          if (attributeName === 'position') {
            var x = attribute.array[3 * hit.pIndex + 0];
            var y = attribute.array[3 * hit.pIndex + 1];
            var z = attribute.array[3 * hit.pIndex + 2];

            var position = new THREE.Vector3(x, y, z);
            position.applyMatrix4(pc.matrixWorld);

            point[attributeName] = position;
          }
        }

        hit.point = point;
      }

      performance.mark('pick-end');
      performance.measure('pick', 'pick-start', 'pick-end');

      if (params.all) {
        return hits.map(hit => hit.point);
      } else {
        if (hits.length === 0) {
          return null;
        } else {
          return hits[0].point;
        }
      }
    }

    computeVisibilityTextureData(nodes) {
      if (Global.measureTimings) {
        performance.mark('computeVisibilityTextureData-start');
      }

      var data = new Uint8Array(nodes.length * 3);
      var visibleNodeTextureOffsets = new Map();

      //copy array
      nodes = nodes.slice();

      //sort by level and number
      var sort = function(a, b) {
        var la = a.geometryNode.level;
        var lb = b.geometryNode.level;
        var na = a.geometryNode.number;
        var nb = b.geometryNode.number;
        if (la !== lb) return la - lb;
        if (na < nb) return -1;
        if (na > nb) return 1;
        return 0;
      };
      nodes.sort(sort);

      var visibleNodeNames = [];
      for (var i = 0; i < nodes.length; i++) {
        visibleNodeNames.push(nodes[i].geometryNode.number);
      }

      for (var i = 0; i < nodes.length; i++) {
        var node = nodes[i];

        visibleNodeTextureOffsets.set(node, i);

        var b1 = 0; //children
        var b2 = 0; //offset to first child
        var b3 = 0; //split

        if (
          node.geometryNode.left &&
          visibleNodeNames.indexOf(node.geometryNode.left.number) > 0
        ) {
          b1 += 1;
          b2 = visibleNodeNames.indexOf(node.geometryNode.left.number) - i;
        }
        if (
          node.geometryNode.right &&
          visibleNodeNames.indexOf(node.geometryNode.right.number) > 0
        ) {
          b1 += 2;
          b2 =
            b2 === 0
              ? visibleNodeNames.indexOf(node.geometryNode.right.number) - i
              : b2;
        }

        if (node.geometryNode.split === 'X') {
          b3 = 1;
        } else if (node.geometryNode.split === 'Y') {
          b3 = 2;
        } else if (node.geometryNode.split === 'Z') {
          b3 = 4;
        }

        data[i * 3 + 0] = b1;
        data[i * 3 + 1] = b2;
        data[i * 3 + 2] = b3;
      }

      if (Global.measureTimings) {
        performance.mark('computeVisibilityTextureData-end');
        performance.measure(
          'render.computeVisibilityTextureData',
          'computeVisibilityTextureData-start',
          'computeVisibilityTextureData-end'
        );
      }

      return {
        data: data,
        offsets: visibleNodeTextureOffsets,
      };
    }

    get progress() {
      if (this.pcoGeometry.root) {
        return Global.numNodesLoading > 0 ? 0 : 1;
      } else {
        return 0;
      }
    }
  }

  class PointCloudArena4DGeometryNode {
    constructor() {
      this.left = null;
      this.right = null;
      this.boundingBox = null;
      this.number = null;
      this.pcoGeometry = null;
      this.loaded = false;
      this.numPoints = 0;
      this.level = 0;
      this.children = [];
      this.oneTimeDisposeHandlers = [];
    }

    isGeometryNode() {
      return true;
    }

    isTreeNode() {
      return false;
    }

    isLoaded() {
      return this.loaded;
    }

    getBoundingSphere() {
      return this.boundingSphere;
    }

    getBoundingBox() {
      return this.boundingBox;
    }

    getChildren() {
      var children = [];

      if (this.left) {
        children.push(this.left);
      }

      if (this.right) {
        children.push(this.right);
      }

      return children;
    }

    getLevel() {
      return this.level;
    }

    load() {
      if (this.loaded || this.loading) {
        return;
      }

      if (Global.numNodesLoading >= Global.maxNodesLoading) {
        return;
      }

      this.loading = true;

      Global.numNodesLoading++;

      var self = this;
      var url = this.pcoGeometry.url + '?node=' + this.number;

      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';
      xhr.onreadystatechange = function() {
        if (!(xhr.readyState === 4 && xhr.status === 200)) {
          return;
        }

        var buffer = xhr.response;
        var sourceView = new DataView(buffer);
        var numPoints = buffer.byteLength / 17;

        var position = new Float32Array(numPoints * 3);
        var color = new Uint8Array(numPoints * 4);
        var intensities = new Float32Array(numPoints);
        var classifications = new Uint8Array(numPoints);
        var indices = new ArrayBuffer(numPoints * 4);
        var u32Indices = new Uint32Array(indices);

        var tightBoundingBox = new THREE.Box3();

        for (var i = 0; i < numPoints; i++) {
          var x =
            sourceView.getFloat32(i * 17 + 0, true) + self.boundingBox.min.x;
          var y =
            sourceView.getFloat32(i * 17 + 4, true) + self.boundingBox.min.y;
          var z =
            sourceView.getFloat32(i * 17 + 8, true) + self.boundingBox.min.z;

          var r = sourceView.getUint8(i * 17 + 12, true);
          var g = sourceView.getUint8(i * 17 + 13, true);
          var b = sourceView.getUint8(i * 17 + 14, true);

          var intensity = sourceView.getUint8(i * 17 + 15, true);

          var classification = sourceView.getUint8(i * 17 + 16, true);

          tightBoundingBox.expandByPoint(new THREE.Vector3(x, y, z));

          position[i * 3 + 0] = x;
          position[i * 3 + 1] = y;
          position[i * 3 + 2] = z;

          color[i * 4 + 0] = r;
          color[i * 4 + 1] = g;
          color[i * 4 + 2] = b;
          color[i * 4 + 3] = 255;

          intensities[i] = intensity;
          classifications[i] = classification;

          u32Indices[i] = i;
        }

        var geometry = new THREE.BufferGeometry();
        geometry.addAttribute(
          'position',
          new THREE.BufferAttribute(position, 3)
        );
        geometry.addAttribute(
          'color',
          new THREE.BufferAttribute(color, 4, true)
        );
        geometry.addAttribute(
          'intensity',
          new THREE.BufferAttribute(intensities, 1)
        );
        geometry.addAttribute(
          'classification',
          new THREE.BufferAttribute(classifications, 1)
        );
        {
          var bufferAttribute = new THREE.BufferAttribute(
            new Uint8Array(indices),
            4,
            true
          );
          geometry.addAttribute('indices', bufferAttribute);
        }

        self.geometry = geometry;
        self.numPoints = numPoints;
        self.loaded = true;
        self.loading = false;
        Global.numNodesLoading--;
      };

      xhr.send(null);
    }

    dispose() {
      if (this.geometry && this.parent != null) {
        this.geometry.dispose();
        this.geometry = null;
        this.loaded = false;

        //this.dispatchEvent( { type: "dispose" } );
        for (var i = 0; i < this.oneTimeDisposeHandlers.length; i++) {
          var handler = this.oneTimeDisposeHandlers[i];
          handler();
        }
        this.oneTimeDisposeHandlers = [];
      }
    }

    getNumPoints() {
      return this.numPoints;
    }
  }
  class PointCloudArena4DGeometry extends THREE.EventDispatcher {
    constructor() {
      super();

      this.numPoints = 0;
      this.version = 0;
      this.boundingBox = null;
      this.numNodes = 0;
      this.name = null;
      this.provider = null;
      this.url = null;
      this.root = null;
      this.levels = 0;
      this._spacing = null;
      this.pointAttributes = new PointAttributes([
        'POSITION_CARTESIAN',
        'COLOR_PACKED',
      ]);
    }

    static load(url, callback) {
      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', url + '?info', true);

      xhr.onreadystatechange = function() {
        try {
          if (xhr.readyState === 4 && xhr.status === 200) {
            var response = JSON.parse(xhr.responseText);

            var geometry = new PointCloudArena4DGeometry();
            geometry.url = url;
            geometry.name = response.Name;
            geometry.provider = response.Provider;
            geometry.numNodes = response.Nodes;
            geometry.numPoints = response.Points;
            geometry.version = response.Version;
            geometry.boundingBox = new THREE.Box3(
              new THREE.Vector3().fromArray(response.BoundingBox.slice(0, 3)),
              new THREE.Vector3().fromArray(response.BoundingBox.slice(3, 6))
            );
            if (response.Spacing) {
              geometry.spacing = response.Spacing;
            }

            var offset = geometry.boundingBox.min.clone().multiplyScalar(-1);

            geometry.boundingBox.min.add(offset);
            geometry.boundingBox.max.add(offset);
            geometry.offset = offset;

            var center = new THREE.Vector3();
            geometry.boundingBox.getCenter(center);
            var radius =
              geometry.boundingBox.getSize(new THREE.Vector3()).length() / 2;
            geometry.boundingSphere = new THREE.Sphere(center, radius);

            geometry.loadHierarchy();

            callback(geometry);
          } else if (xhr.readyState === 4) {
            callback(null);
          }
        } catch (e) {
          console.error(e.message);
          callback(null);
        }
      };

      xhr.send(null);
    }

    loadHierarchy() {
      var url = this.url + '?tree';

      var xhr = new XMLHttpRequest();
      xhr.overrideMimeType('text/plain');
      xhr.open('GET', url, true);
      xhr.responseType = 'arraybuffer';

      xhr.onreadystatechange = () => {
        if (!(xhr.readyState === 4 && xhr.status === 200)) {
          return;
        }

        var buffer = xhr.response;
        var numNodes = buffer.byteLength / 3;
        var view = new DataView(buffer);
        var stack = [];
        var root = null;

        var levels = 0;

        //TODO Debug: var start = new Date().getTime();
        //read hierarchy
        for (var i = 0; i < numNodes; i++) {
          var mask = view.getUint8(i * 3 + 0, true);

          var hasLeft = (mask & 1) > 0;
          var hasRight = (mask & 2) > 0;
          var splitX = (mask & 4) > 0;
          var splitY = (mask & 8) > 0;
          var splitZ = (mask & 16) > 0;
          var split = null;
          if (splitX) {
            split = 'X';
          } else if (splitY) {
            split = 'Y';
          }
          if (splitZ) {
            split = 'Z';
          }

          var node = new PointCloudArena4DGeometryNode();
          node.hasLeft = hasLeft;
          node.hasRight = hasRight;
          node.split = split;
          node.isLeaf = !hasLeft && !hasRight;
          node.number = i;
          node.left = null;
          node.right = null;
          node.pcoGeometry = this;
          node.level = stack.length;
          levels = Math.max(levels, node.level);

          if (stack.length > 0) {
            var parent = stack[stack.length - 1];
            node.boundingBox = parent.boundingBox.clone();
            var parentBBSize = parent.boundingBox.getSize(new THREE.Vector3());

            if (parent.hasLeft && !parent.left) {
              parent.left = node;
              parent.children.push(node);

              if (parent.split === 'X') {
                node.boundingBox.max.x =
                  node.boundingBox.min.x + parentBBSize.x / 2;
              } else if (parent.split === 'Y') {
                node.boundingBox.max.y =
                  node.boundingBox.min.y + parentBBSize.y / 2;
              } else if (parent.split === 'Z') {
                node.boundingBox.max.z =
                  node.boundingBox.min.z + parentBBSize.z / 2;
              }

              var center = new THREE.Vector3();
              node.boundingBox.getCenter(center);
              var radius =
                node.boundingBox.getSize(new THREE.Vector3()).length() / 2;
              node.boundingSphere = new THREE.Sphere(center, radius);
            } else {
              parent.right = node;
              parent.children.push(node);

              if (parent.split === 'X') {
                node.boundingBox.min.x =
                  node.boundingBox.min.x + parentBBSize.x / 2;
              } else if (parent.split === 'Y') {
                node.boundingBox.min.y =
                  node.boundingBox.min.y + parentBBSize.y / 2;
              } else if (parent.split === 'Z') {
                node.boundingBox.min.z =
                  node.boundingBox.min.z + parentBBSize.z / 2;
              }

              var center = new THREE.Vector3();
              node.boundingBox.getCenter(center);
              var radius =
                node.boundingBox.getSize(new THREE.Vector3()).length() / 2;
              node.boundingSphere = new THREE.Sphere(center, radius);
            }
          } else {
            root = node;
            root.boundingBox = this.boundingBox.clone();

            var center = new THREE.Vector3();
            root.boundingBox.getCenter(center);
            var radius =
              root.boundingBox.getSize(new THREE.Vector3()).length() / 2;
            root.boundingSphere = new THREE.Sphere(center, radius);
          }

          var bbSize = node.boundingBox.getSize(new THREE.Vector3());
          node.spacing = (bbSize.x + bbSize.y + bbSize.z) / 3 / 75;
          node.estimatedSpacing = node.spacing;

          stack.push(node);

          if (node.isLeaf) {
            var done = false;
            while (!done && stack.length > 0) {
              stack.pop();

              var top = stack[stack.length - 1];

              done = stack.length > 0 && top.hasRight && top.right == null;
            }
          }
        }

        this.root = root;
        this.levels = levels;

        this.dispatchEvent({
          type: 'hierarchy_loaded',
        });
      };

      xhr.send(null);
    }

    get spacing() {
      if (this._spacing) {
        return this._spacing;
      } else if (this.root) {
        return this.root.spacing;
      }
    }

    set spacing(value) {
      this._spacing = value;
    }
  }

  /*
   ** Binary Heap implementation in Javascript
   ** From: http://eloquentjavascript.net/1st_edition/appendix2.htmlt
   **
   ** Copyright (c) 2007 Marijn Haverbeke, last modified on November 28 2013.
   **
   ** Licensed under a Creative Commons attribution-noncommercial license.
   ** All code in this book may also be considered licensed under an MIT license.
   */

  function BinaryHeap(scoreFunction) {
    this.content = [];
    this.scoreFunction = scoreFunction;
  }

  BinaryHeap.prototype = {
    push: function(element) {
      // Add the new element to the end of the array.
      this.content.push(element);
      // Allow it to bubble up.
      this.bubbleUp(this.content.length - 1);
    },

    pop: function() {
      // Store the first element so we can return it later.
      var result = this.content[0];
      // Get the element at the end of the array.
      var end = this.content.pop();
      // If there are any elements left, put the end element at the
      // start, and let it sink down.
      if (this.content.length > 0) {
        this.content[0] = end;
        this.sinkDown(0);
      }
      return result;
    },

    remove: function(node) {
      var length = this.content.length;
      // To remove a value, we must search through the array to find
      // it.
      for (var i = 0; i < length; i++) {
        if (this.content[i] != node) continue;
        // When it is found, the process seen in 'pop' is repeated
        // to fill up the hole.
        var end = this.content.pop();
        // If the element we popped was the one we needed to remove,
        // we're done.
        if (i == length - 1) break;
        // Otherwise, we replace the removed element with the popped
        // one, and allow it to float up or sink down as appropriate.
        this.content[i] = end;
        this.bubbleUp(i);
        this.sinkDown(i);
        break;
      }
    },

    size: function() {
      return this.content.length;
    },

    bubbleUp: function(n) {
      // Fetch the element that has to be moved.
      var element = this.content[n],
        score = this.scoreFunction(element);
      // When at 0, an element can not go up any further.
      while (n > 0) {
        // Compute the parent element's index, and fetch it.
        var parentN = Math.floor((n + 1) / 2) - 1,
          parent = this.content[parentN];
        // If the parent has a lesser score, things are in order and we
        // are done.
        if (score >= this.scoreFunction(parent)) break;

        // Otherwise, swap the parent with the current element and
        // continue.
        this.content[parentN] = element;
        this.content[n] = parent;
        n = parentN;
      }
    },

    sinkDown: function(n) {
      // Look up the target element and its score.
      var length = this.content.length,
        element = this.content[n],
        elemScore = this.scoreFunction(element);

      while (true) {
        // Compute the indices of the child elements.
        var child2N = (n + 1) * 2,
          child1N = child2N - 1;
        // This is used to store the new position of the element,
        // if any.
        var swap = null;
        // If the first child exists (is inside the array)...
        if (child1N < length) {
          // Look it up and compute its score.
          var child1 = this.content[child1N],
            child1Score = this.scoreFunction(child1);
          // If the score is less than our element's, we need to swap.
          if (child1Score < elemScore) swap = child1N;
        }
        // Do the same checks for the other child.
        if (child2N < length) {
          var child2 = this.content[child2N],
            child2Score = this.scoreFunction(child2);
          if (child2Score < (swap == null ? elemScore : child1Score))
            swap = child2N;
        }

        // No need to swap further, we are done.
        if (swap == null) break;

        // Otherwise, swap and continue.
        this.content[n] = this.content[swap];
        this.content[swap] = element;
        n = swap;
      }
    },
  };

  var AttributeLocations = {
    position: 0,
    color: 1,
    intensity: 2,
    classification: 3,
    returnNumber: 4,
    numberOfReturns: 5,
    pointSourceID: 6,
    indices: 7,
    normal: 8,
    spacing: 9,
  };

  var Classification = {
    DEFAULT: {
      0: new THREE.Vector4(0.5, 0.5, 0.5, 1.0),
      1: new THREE.Vector4(0.5, 0.5, 0.5, 1.0),
      2: new THREE.Vector4(0.63, 0.32, 0.18, 1.0),
      3: new THREE.Vector4(0.0, 1.0, 0.0, 1.0),
      4: new THREE.Vector4(0.0, 0.8, 0.0, 1.0),
      5: new THREE.Vector4(0.0, 0.6, 0.0, 1.0),
      6: new THREE.Vector4(1.0, 0.66, 0.0, 1.0),
      7: new THREE.Vector4(1.0, 0, 1.0, 1.0),
      8: new THREE.Vector4(1.0, 0, 0.0, 1.0),
      9: new THREE.Vector4(0.0, 0.0, 1.0, 1.0),
      12: new THREE.Vector4(1.0, 1.0, 0.0, 1.0),
      DEFAULT: new THREE.Vector4(0.3, 0.6, 0.6, 0.5),
    },
  };

  var ClipTask = {
    NONE: 0,
    HIGHLIGHT: 1,
    SHOW_INSIDE: 2,
    SHOW_OUTSIDE: 3,
  };

  var ClipMethod = {
    INSIDE_ANY: 0,
    INSIDE_ALL: 1,
  };

  var PointSizeType = {
    FIXED: 0,
    ATTENUATED: 1,
    ADAPTIVE: 2,
  };

  var PointShape = {
    SQUARE: 0,
    CIRCLE: 1,
    PARABOLOID: 2,
  };

  var PointColorType = {
    RGB: 0,
    COLOR: 1,
    DEPTH: 2,
    HEIGHT: 3,
    ELEVATION: 3,
    INTENSITY: 4,
    INTENSITY_GRADIENT: 5,
    LOD: 6,
    LEVEL_OF_DETAIL: 6,
    POINT_INDEX: 7,
    CLASSIFICATION: 8,
    RETURN_NUMBER: 9,
    SOURCE: 10,
    NORMAL: 11,
    PHONG: 12,
    RGB_HEIGHT: 13,
    COMPOSITE: 50,
  };

  var TreeType = {
    OCTREE: 0,
    KDTREE: 1,
  };

  function loadPointCloud(path, name, callback) {
    var loaded = function(pointcloud) {
      if (name !== undefined) {
        pointcloud.name = name;
      }

      callback({
        type: 'pointcloud_loaded',
        pointcloud: pointcloud,
      });
    };

    //Greyhound pointcloud server URL.
    if (path.indexOf('greyhound://') === 0) {
      GreyhoundLoader.load(path, function(geometry) {
        if (geometry !== undefined) {
          loaded(new PointCloudOctree(geometry));
        }
      });
    }
    //Potree point cloud
    else if (path.indexOf('cloud.js') > 0) {
      POCLoader.load(path, function(geometry) {
        if (geometry !== undefined) {
          loaded(new PointCloudOctree(geometry));
        }
      });
    } else if (path.indexOf('ept.json') > 0) {
      EptLoader.load(path, function(geometry) {
        if (geometry !== undefined) {
          loaded(new PointCloudOctree(geometry));
        }
      });
    }
    //Arena 4D point cloud
    else if (path.indexOf('.vpc') > 0) {
      PointCloudArena4DGeometry.load(path, function(geometry) {
        if (geometry !== undefined) {
          loaded(new PointCloudArena4D(geometry));
        }
      });
    } else {
      throw new Error('Potree: Failed to load point cloud from URL ' + path);
    }
  }

  function updateVisibility(pointclouds, camera, renderer) {
    var numVisiblePoints = 0;
    var numVisiblePointsInPointclouds = new Map(pointclouds.map(pc => [pc, 0]));
    var visibleNodes = [];
    var unloadedGeometry = [];
    var lowestSpacing = Infinity;

    //Calculate object space frustum and cam pos and setup priority queue
    var structures = updateVisibilityStructures(pointclouds, camera, renderer);
    var frustums = structures.frustums;
    var camObjPositions = structures.camObjPositions;
    var priorityQueue = structures.priorityQueue;

    var loadedToGPUThisFrame = 0;
    var domWidth = renderer.domElement.clientWidth;
    var domHeight = renderer.domElement.clientHeight;

    //Check if pointcloud has been transformed, some code will only be executed if changes have been detected
    if (!Global.pointcloudTransformVersion) {
      Global.pointcloudTransformVersion = new Map();
    }

    var pointcloudTransformVersion = Global.pointcloudTransformVersion;

    for (var i = 0; i < pointclouds.length; i++) {
      var pointcloud = pointclouds[i];

      if (!pointcloud.visible) {
        continue;
      }

      pointcloud.updateMatrixWorld();

      if (!pointcloudTransformVersion.has(pointcloud)) {
        pointcloudTransformVersion.set(pointcloud, {
          number: 0,
          transform: pointcloud.matrixWorld.clone(),
        });
      } else {
        var version = pointcloudTransformVersion.get(pointcloud);
        if (!version.transform.equals(pointcloud.matrixWorld)) {
          version.number++;
          version.transform.copy(pointcloud.matrixWorld);

          pointcloud.dispatchEvent({
            type: 'transformation_changed',
            target: pointcloud,
          });
        }
      }
    }

    //Process priority queue
    while (priorityQueue.size() > 0) {
      var element = priorityQueue.pop();
      var node = element.node;
      var parent = element.parent;
      var pointcloud = pointclouds[element.pointcloud];
      var box = node.getBoundingBox();
      var frustum = frustums[element.pointcloud];
      var camObjPos = camObjPositions[element.pointcloud];

      var insideFrustum = frustum.intersectsBox(box);
      var maxLevel = pointcloud.maxLevel || Infinity;
      var level = node.getLevel();

      var visible = insideFrustum;
      visible =
        visible &&
        !(
          numVisiblePointsInPointclouds.get(pointcloud) + node.getNumPoints() >
          pointcloud.pointBudget
        );
      visible = visible && level < maxLevel;

      //TODO <CLIPPING TASKS>
      /*
      if(false && pointcloud.material.clipBoxes.length > 0)
      {
        var numIntersecting = 0;
        var numIntersectionVolumes = 0;

        for(var clipBox of pointcloud.material.clipBoxes)
        {
          var pcWorldInverse = new THREE.Matrix4().getInverse(pointcloud.matrixWorld);
          var toPCObject = pcWorldInverse.multiply(clipBox.box.matrixWorld);

          var px = new THREE.Vector3(+1, 0, 0).applyMatrix4(toPCObject);
          var nx = new THREE.Vector3(-1, 0, 0).applyMatrix4(toPCObject);
          var py = new THREE.Vector3(0, +1, 0).applyMatrix4(toPCObject);
          var ny = new THREE.Vector3(0, -1, 0).applyMatrix4(toPCObject);
          var pz = new THREE.Vector3(0, 0, +1).applyMatrix4(toPCObject);
          var nz = new THREE.Vector3(0, 0, -1).applyMatrix4(toPCObject);

          var pxN = new THREE.Vector3().subVectors(nx, px).normalize();
          var nxN = pxN.clone().multiplyScalar(-1);
          var pyN = new THREE.Vector3().subVectors(ny, py).normalize();
          var nyN = pyN.clone().multiplyScalar(-1);
          var pzN = new THREE.Vector3().subVectors(nz, pz).normalize();
          var nzN = pzN.clone().multiplyScalar(-1);

          var pxPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(pxN, px);
          var nxPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(nxN, nx);
          var pyPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(pyN, py);
          var nyPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(nyN, ny);
          var pzPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(pzN, pz);
          var nzPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(nzN, nz);

          var frustum = new THREE.Frustum(pxPlane, nxPlane, pyPlane, nyPlane, pzPlane, nzPlane);
          var intersects = frustum.intersectsBox(box);

          if(intersects)
          {
            numIntersecting++;
          }
          numIntersectionVolumes++;
        }

        var insideAny = numIntersecting > 0;
        var insideAll = numIntersecting === numIntersectionVolumes;

        if(pointcloud.material.clipTask === ClipTask.SHOW_INSIDE)
        {
          if(pointcloud.material.clipMethod === ClipMethod.INSIDE_ANY && insideAny)
          {
            //node.debug = true
          }
          else if(pointcloud.material.clipMethod === ClipMethod.INSIDE_ALL && insideAll)
          {
            //node.debug = true;
          }
          else
          {
            visible = false;
          }
        }
      }
      */

      if (node.spacing) {
        lowestSpacing = Math.min(lowestSpacing, node.spacing);
      } else if (node.geometryNode && node.geometryNode.spacing) {
        lowestSpacing = Math.min(lowestSpacing, node.geometryNode.spacing);
      }

      if (!visible) {
        continue;
      }
      numVisiblePoints += node.getNumPoints();

      var numVisiblePointsInPointcloud = numVisiblePointsInPointclouds.get(
        pointcloud
      );
      numVisiblePointsInPointclouds.set(
        pointcloud,
        numVisiblePointsInPointcloud + node.getNumPoints()
      );

      pointcloud.numVisibleNodes++;
      pointcloud.numVisiblePoints += node.getNumPoints();

      if (node.isGeometryNode() && (!parent || parent.isTreeNode())) {
        if (
          node.isLoaded() &&
          loadedToGPUThisFrame < Global.maxNodesLoadGPUFrame
        ) {
          node = pointcloud.toTreeNode(node, parent);
          loadedToGPUThisFrame++;
        } else {
          unloadedGeometry.push(node);
        }
      }

      if (node.isTreeNode()) {
        Global.lru.touch(node.geometryNode);

        node.sceneNode.visible = true;
        node.sceneNode.material = pointcloud.material;

        visibleNodes.push(node);
        pointcloud.visibleNodes.push(node);

        if (node._transformVersion === undefined) {
          node._transformVersion = -1;
        }

        var transformVersion = pointcloudTransformVersion.get(pointcloud);
        if (node._transformVersion !== transformVersion.number) {
          node.sceneNode.updateMatrix();
          node.sceneNode.matrixWorld.multiplyMatrices(
            pointcloud.matrixWorld,
            node.sceneNode.matrix
          );
          node._transformVersion = transformVersion.number;
        }

        if (
          pointcloud.showBoundingBox &&
          !node.boundingBoxNode &&
          node.getBoundingBox
        ) {
          var boxHelper = new THREE.Box3Helper(node.getBoundingBox());
          boxHelper.matrixAutoUpdate = false;
          pointcloud.boundingBoxNodes.push(boxHelper);
          node.boundingBoxNode = boxHelper;
          node.boundingBoxNode.matrix.copy(pointcloud.matrixWorld);
        } else if (pointcloud.showBoundingBox) {
          node.boundingBoxNode.visible = true;
          node.boundingBoxNode.matrix.copy(pointcloud.matrixWorld);
        } else if (!pointcloud.showBoundingBox && node.boundingBoxNode) {
          node.boundingBoxNode.visible = false;
        }
      }

      //Add child nodes to priorityQueue
      var children = node.getChildren();
      for (var i = 0; i < children.length; i++) {
        var child = children[i];
        var weight = 0;

        //Perspective camera
        if (camera.isPerspectiveCamera) {
          var sphere = child.getBoundingSphere(new THREE.Sphere());
          var center = sphere.center;
          var distance = sphere.center.distanceTo(camObjPos);

          var radius = sphere.radius;
          var fov = (camera.fov * Math.PI) / 180;
          var slope = Math.tan(fov / 2);
          var projFactor = (0.5 * domHeight) / (slope * distance);
          var screenPixelRadius = radius * projFactor;

          //If pixel radius bellow minimum discard
          if (screenPixelRadius < pointcloud.minimumNodePixelSize) {
            continue;
          }

          weight = screenPixelRadius;

          //Really close to the camera
          if (distance - radius < 0) {
            weight = Number.MAX_VALUE;
          }
        }
        //Orthographic camera
        else {
          //TODO <IMPROVE VISIBILITY>
          var bb = child.getBoundingBox();
          var distance = child
            .getBoundingSphere(new THREE.Sphere())
            .center.distanceTo(camObjPos);
          var diagonal = bb.max
            .clone()
            .sub(bb.min)
            .length();
          weight = diagonal / distance;
        }

        priorityQueue.push({
          pointcloud: element.pointcloud,
          node: child,
          parent: node,
          weight: weight,
        });
      }
    }

    //Update DEM
    var candidates = pointclouds.filter(
      p => p.generateDEM && p.dem instanceof DEM
    );

    for (var pointcloud of candidates) {
      var updatingNodes = pointcloud.visibleNodes.filter(
        n => n.getLevel() <= Global.maxDEMLevel
      );
      pointcloud.dem.update(updatingNodes);
    }

    for (
      var i = 0;
      i < Math.min(Global.maxNodesLoading, unloadedGeometry.length);
      i++
    ) {
      unloadedGeometry[i].load();
    }

    return {
      visibleNodes: visibleNodes,
      numVisiblePoints: numVisiblePoints,
      lowestSpacing: lowestSpacing,
    };
  }

  function updatePointClouds(pointclouds, camera, renderer) {
    var result = updateVisibility(pointclouds, camera, renderer);

    for (var i = 0; i < pointclouds.length; i++) {
      pointclouds[i].updateMaterial(
        pointclouds[i].material,
        pointclouds[i].visibleNodes,
        camera,
        renderer
      );
      pointclouds[i].updateVisibleBounds();
    }

    Global.lru.freeMemory();

    return result;
  }

  function updateVisibilityStructures(pointclouds, camera, renderer) {
    var frustums = [];
    var camObjPositions = [];
    var priorityQueue = new BinaryHeap(function(x) {
      return 1 / x.weight;
    });

    for (var i = 0; i < pointclouds.length; i++) {
      var pointcloud = pointclouds[i];

      if (!pointcloud.initialized()) {
        continue;
      }

      pointcloud.numVisibleNodes = 0;
      pointcloud.numVisiblePoints = 0;
      pointcloud.deepestVisibleLevel = 0;
      pointcloud.visibleNodes = [];
      pointcloud.visibleGeometry = [];

      //Frustum in object space
      camera.updateMatrixWorld();
      var frustum = new THREE.Frustum();
      var viewI = camera.matrixWorldInverse;
      var world = pointcloud.matrixWorld;

      //Use close near plane for frustum intersection
      var frustumCam = camera.clone();
      frustumCam.near = camera.near; //Math.min(camera.near, 0.1);
      frustumCam.updateProjectionMatrix();
      var proj = camera.projectionMatrix;

      var fm = new THREE.Matrix4()
        .multiply(proj)
        .multiply(viewI)
        .multiply(world);
      frustum.setFromMatrix(fm);
      frustums.push(frustum);

      //Camera position in object space
      var view = camera.matrixWorld;
      var worldI = new THREE.Matrix4().getInverse(world);
      var camMatrixObject = new THREE.Matrix4().multiply(worldI).multiply(view);
      var camObjPos = new THREE.Vector3().setFromMatrixPosition(
        camMatrixObject
      );
      camObjPositions.push(camObjPos);

      if (pointcloud.visible && pointcloud.root !== null) {
        priorityQueue.push({
          pointcloud: i,
          node: pointcloud.root,
          weight: Number.MAX_VALUE,
        });
      }

      //Hide all previously visible nodes
      if (pointcloud.root.isTreeNode()) {
        pointcloud.hideDescendants(pointcloud.root.sceneNode);
      }

      for (var j = 0; j < pointcloud.boundingBoxNodes.length; j++) {
        pointcloud.boundingBoxNodes[j].visible = false;
      }
    }

    return {
      frustums: frustums,
      camObjPositions: camObjPositions,
      priorityQueue: priorityQueue,
    };
  }

  class Points {
    constructor() {
      this.boundingBox = new THREE.Box3();
      this.numPoints = 0;
      this.data = {};
    }

    add(points) {
      var currentSize = this.numPoints;
      var additionalSize = points.numPoints;
      var newSize = currentSize + additionalSize;

      var thisAttributes = Object.keys(this.data);
      var otherAttributes = Object.keys(points.data);
      var attributes = new Set([...thisAttributes, ...otherAttributes]);

      for (var attribute of attributes) {
        if (
          thisAttributes.includes(attribute) &&
          otherAttributes.includes(attribute)
        ) {
          //attribute in both, merge
          var Type = this.data[attribute].constructor;
          var merged = new Type(
            this.data[attribute].length + points.data[attribute].length
          );
          merged.set(this.data[attribute], 0);
          merged.set(points.data[attribute], this.data[attribute].length);
          this.data[attribute] = merged;
        } else if (
          thisAttributes.includes(attribute) &&
          !otherAttributes.includes(attribute)
        ) {
          //attribute only in this; take over this and expand to new size
          var elementsPerPoint = this.data[attribute].length / this.numPoints;
          var Type = this.data[attribute].constructor;
          var expanded = new Type(elementsPerPoint * newSize);
          expanded.set(this.data[attribute], 0);
          this.data[attribute] = expanded;
        } else if (
          !thisAttributes.includes(attribute) &&
          otherAttributes.includes(attribute)
        ) {
          //attribute only in points to be added; take over new points and expand to new size
          var elementsPerPoint =
            points.data[attribute].length / points.numPoints;
          var Type = points.data[attribute].constructor;
          var expanded = new Type(elementsPerPoint * newSize);
          expanded.set(points.data[attribute], elementsPerPoint * currentSize);
          this.data[attribute] = expanded;
        }
      }

      this.numPoints = newSize;

      this.boundingBox.union(points.boundingBox);
    }
  }

  function paramThreeToGL(gl, p) {
    var extension;

    if (p === THREE.RepeatWrapping) return gl.REPEAT;
    if (p === THREE.ClampToEdgeWrapping) return gl.CLAMP_TO_EDGE;
    if (p === THREE.MirroredRepeatWrapping) return gl.MIRRORED_REPEAT;

    if (p === THREE.NearestFilter) return gl.NEAREST;
    if (p === THREE.NearestMipMapNearestFilter)
      return gl.NEAREST_MIPMAP_NEAREST;
    if (p === THREE.NearestMipMapLinearFilter) return gl.NEAREST_MIPMAP_LINEAR;

    if (p === THREE.LinearFilter) return gl.LINEAR;
    if (p === THREE.LinearMipMapNearestFilter) return gl.LINEAR_MIPMAP_NEAREST;
    if (p === THREE.LinearMipMapLinearFilter) return gl.LINEAR_MIPMAP_LINEAR;

    if (p === THREE.UnsignedByteType) return gl.UNSIGNED_BYTE;
    if (p === THREE.UnsignedShort4444Type) return gl.UNSIGNED_SHORT_4_4_4_4;
    if (p === THREE.UnsignedShort5551Type) return gl.UNSIGNED_SHORT_5_5_5_1;
    if (p === THREE.UnsignedShort565Type) return gl.UNSIGNED_SHORT_5_6_5;

    if (p === THREE.ByteType) return gl.BYTE;
    if (p === THREE.ShortType) return gl.SHORT;
    if (p === THREE.UnsignedShortType) return gl.UNSIGNED_SHORT;
    if (p === THREE.IntType) return gl.INT;
    if (p === THREE.UnsignedIntType) return gl.UNSIGNED_INT;
    if (p === THREE.FloatType) return gl.FLOAT;

    if (p === THREE.HalfFloatType) {
      extension = extensions.get('OES_texture_half_float');
      if (extension !== null) return extension.HALF_FLOAT_OES;
    }

    if (p === THREE.AlphaFormat) return gl.ALPHA;
    if (p === THREE.RGBFormat) return gl.RGB;
    if (p === THREE.RGBAFormat) return gl.RGBA;
    if (p === THREE.LuminanceFormat) return gl.LUMINANCE;
    if (p === THREE.LuminanceAlphaFormat) return gl.LUMINANCE_ALPHA;
    if (p === THREE.DepthFormat) return gl.DEPTH_COMPONENT;
    if (p === THREE.DepthStencilFormat) return gl.DEPTH_STENCIL;

    if (p === THREE.AddEquation) return gl.FUNC_ADD;
    if (p === THREE.SubtractEquation) return gl.FUNC_SUBTRACT;
    if (p === THREE.ReverseSubtractEquation) return gl.FUNC_REVERSE_SUBTRACT;

    if (p === THREE.ZeroFactor) return gl.ZERO;
    if (p === THREE.OneFactor) return gl.ONE;
    if (p === THREE.SrcColorFactor) return gl.SRC_COLOR;
    if (p === THREE.OneMinusSrcColorFactor) return gl.ONE_MINUS_SRC_COLOR;
    if (p === THREE.SrcAlphaFactor) return gl.SRC_ALPHA;
    if (p === THREE.OneMinusSrcAlphaFactor) return gl.ONE_MINUS_SRC_ALPHA;
    if (p === THREE.DstAlphaFactor) return gl.DST_ALPHA;
    if (p === THREE.OneMinusDstAlphaFactor) return gl.ONE_MINUS_DST_ALPHA;

    if (p === THREE.DstColorFactor) return gl.DST_COLOR;
    if (p === THREE.OneMinusDstColorFactor) return gl.ONE_MINUS_DST_COLOR;
    if (p === THREE.SrcAlphaSaturateFactor) return gl.SRC_ALPHA_SATURATE;

    if (
      p === THREE.RGB_S3TC_DXT1_Format ||
      p === RGBA_S3TC_DXT1_Format ||
      p === THREE.RGBA_S3TC_DXT3_Format ||
      p === RGBA_S3TC_DXT5_Format
    ) {
      extension = extensions.get('WEBGL_compressed_texture_s3tc');

      if (extension !== null) {
        if (p === THREE.RGB_S3TC_DXT1_Format)
          return extension.COMPRESSED_RGB_S3TC_DXT1_EXT;
        if (p === THREE.RGBA_S3TC_DXT1_Format)
          return extension.COMPRESSED_RGBA_S3TC_DXT1_EXT;
        if (p === THREE.RGBA_S3TC_DXT3_Format)
          return extension.COMPRESSED_RGBA_S3TC_DXT3_EXT;
        if (p === THREE.RGBA_S3TC_DXT5_Format)
          return extension.COMPRESSED_RGBA_S3TC_DXT5_EXT;
      }
    }

    if (
      p === THREE.RGB_PVRTC_4BPPV1_Format ||
      p === THREE.RGB_PVRTC_2BPPV1_Format ||
      p === THREE.RGBA_PVRTC_4BPPV1_Format ||
      p === THREE.RGBA_PVRTC_2BPPV1_Format
    ) {
      extension = extensions.get('WEBGL_compressed_texture_pvrtc');

      if (extension !== null) {
        if (p === THREE.RGB_PVRTC_4BPPV1_Format)
          return extension.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;
        if (p === THREE.RGB_PVRTC_2BPPV1_Format)
          return extension.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;
        if (p === THREE.RGBA_PVRTC_4BPPV1_Format)
          return extension.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;
        if (p === THREE.RGBA_PVRTC_2BPPV1_Format)
          return extension.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG;
      }
    }

    if (p === THREE.RGB_ETC1_Format) {
      extension = extensions.get('WEBGL_compressed_texture_etc1');
      if (extension !== null) return extension.COMPRESSED_RGB_ETC1_WEBGL;
    }

    if (p === THREE.MinEquation || p === THREE.MaxEquation) {
      extension = extensions.get('EXT_blend_minmax');

      if (extension !== null) {
        if (p === THREE.MinEquation) return extension.MIN_EXT;
        if (p === THREE.MaxEquation) return extension.MAX_EXT;
      }
    }

    if (p === UnsignedInt248Type) {
      extension = extensions.get('WEBGL_depth_texture');
      if (extension !== null) return extension.UNSIGNED_INT_24_8_WEBGL;
    }

    return 0;
  }

  class WebGLTexture {
    constructor(gl, texture) {
      this.gl = gl;

      this.texture = texture;
      this.id = gl.createTexture();

      this.target = gl.TEXTURE_2D;
      this.version = -1;

      this.update(texture);
    }

    update() {
      if (!this.texture.image) {
        this.version = this.texture.version;
        return;
      }

      var gl = this.gl;
      var texture = this.texture;

      if (this.version === texture.version) {
        return;
      }

      this.target = gl.TEXTURE_2D;

      gl.bindTexture(this.target, this.id);

      var level = 0;
      var internalFormat = paramThreeToGL(gl, texture.format);
      var width = texture.image.width;
      var height = texture.image.height;
      var border = 0;
      var srcFormat = internalFormat;
      var srcType = paramThreeToGL(gl, texture.type);
      var data;

      gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, texture.flipY);
      gl.pixelStorei(
        gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL,
        texture.premultiplyAlpha
      );
      gl.pixelStorei(gl.UNPACK_ALIGNMENT, texture.unpackAlignment);

      if (texture instanceof THREE.DataTexture) {
        data = texture.image.data;

        gl.texParameteri(this.target, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(this.target, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

        gl.texParameteri(
          this.target,
          gl.TEXTURE_MAG_FILTER,
          paramThreeToGL(gl, texture.magFilter)
        );
        gl.texParameteri(
          this.target,
          gl.TEXTURE_MIN_FILTER,
          paramThreeToGL(gl, texture.minFilter)
        );

        gl.texImage2D(
          this.target,
          level,
          internalFormat,
          width,
          height,
          border,
          srcFormat,
          srcType,
          data
        );
      } else if (texture instanceof THREE.CanvasTexture) {
        data = texture.image;

        gl.texParameteri(
          this.target,
          gl.TEXTURE_WRAP_S,
          paramThreeToGL(gl, texture.wrapS)
        );
        gl.texParameteri(
          this.target,
          gl.TEXTURE_WRAP_T,
          paramThreeToGL(gl, texture.wrapT)
        );

        gl.texParameteri(
          this.target,
          gl.TEXTURE_MAG_FILTER,
          paramThreeToGL(gl, texture.magFilter)
        );
        gl.texParameteri(
          this.target,
          gl.TEXTURE_MIN_FILTER,
          paramThreeToGL(gl, texture.minFilter)
        );

        gl.texImage2D(
          this.target,
          level,
          internalFormat,
          internalFormat,
          srcType,
          data
        );
      }

      gl.bindTexture(this.target, null);

      this.version = texture.version;
    }
  }

  class Shader {
    constructor(gl, name, vsSource, fsSource) {
      this.gl = gl;
      this.name = name;
      this.vsSource = vsSource;
      this.fsSource = fsSource;

      this.cache = new Map();

      this.vs = null;
      this.fs = null;
      this.program = null;

      this.uniformLocations = {};
      this.attributeLocations = {};

      this.update(vsSource, fsSource);
    }

    update(vsSource, fsSource) {
      this.vsSource = vsSource;
      this.fsSource = fsSource;

      this.linkProgram();
    }

    compileShader(shader, source) {
      var gl = this.gl;

      gl.shaderSource(shader, source);

      gl.compileShader(shader);

      var success = gl.getShaderParameter(shader, gl.COMPILE_STATUS);
      if (!success) {
        var info = gl.getShaderInfoLog(shader);
        throw new Error(
          'Potree: Could not compile shader ' + this.name + ', ' + info
        );
      }
    }

    linkProgram() {
      var gl = this.gl;

      this.uniformLocations = {};
      this.attributeLocations = {};

      gl.useProgram(null);

      var cached = this.cache.get(`${this.vsSource}, ${this.fsSource}`);
      if (cached) {
        this.program = cached.program;
        this.vs = cached.vs;
        this.fs = cached.fs;
        this.attributeLocations = cached.attributeLocations;
        this.uniformLocations = cached.uniformLocations;

        return;
      } else {
        this.vs = gl.createShader(gl.VERTEX_SHADER);
        this.fs = gl.createShader(gl.FRAGMENT_SHADER);
        this.program = gl.createProgram();

        for (var name of Object.keys(AttributeLocations)) {
          var location = AttributeLocations[name];
          gl.bindAttribLocation(this.program, location, name);
        }

        this.compileShader(this.vs, this.vsSource);
        this.compileShader(this.fs, this.fsSource);

        var program = this.program;

        gl.attachShader(program, this.vs);
        gl.attachShader(program, this.fs);

        gl.linkProgram(program);

        gl.detachShader(program, this.vs);
        gl.detachShader(program, this.fs);

        var success = gl.getProgramParameter(program, gl.LINK_STATUS);
        if (!success) {
          var info = gl.getProgramInfoLog(program);
          throw new Error(
            'Potree: Could not link program ' + this.name + ', ' + info
          );
        }

        //attribute locations
        var numAttributes = gl.getProgramParameter(
          program,
          gl.ACTIVE_ATTRIBUTES
        );

        for (var i = 0; i < numAttributes; i++) {
          var attribute = gl.getActiveAttrib(program, i);

          var location = gl.getAttribLocation(program, attribute.name);

          this.attributeLocations[attribute.name] = location;
        }

        //uniform locations
        var numUniforms = gl.getProgramParameter(program, gl.ACTIVE_UNIFORMS);

        for (var i = 0; i < numUniforms; i++) {
          var uniform = gl.getActiveUniform(program, i);

          var location = gl.getUniformLocation(program, uniform.name);

          this.uniformLocations[uniform.name] = location;
        }

        var cached = {
          program: this.program,
          vs: this.vs,
          fs: this.fs,
          attributeLocations: this.attributeLocations,
          uniformLocations: this.uniformLocations,
        };

        this.cache.set(`${this.vsSource}, ${this.fsSource}`, cached);
      }
    }

    setUniformMatrix4(name, value) {
      const gl = this.gl;
      const location = this.uniformLocations[name];

      if (location == null) {
        return;
      }

      var tmp = new Float32Array(value.elements);
      gl.uniformMatrix4fv(location, false, tmp);
    }

    setUniform1f(name, value) {
      const gl = this.gl;
      const location = this.uniformLocations[name];

      if (location == null) {
        return;
      }

      gl.uniform1f(location, value);
    }

    setUniformBoolean(name, value) {
      const gl = this.gl;
      const location = this.uniformLocations[name];

      if (location == null) {
        return;
      }

      gl.uniform1i(location, value);
    }

    setUniformTexture(name, value) {
      const gl = this.gl;
      const location = this.uniformLocations[name];

      if (location == null) {
        return;
      }

      gl.uniform1i(location, value);
    }

    setUniform2f(name, value) {
      const gl = this.gl;
      const location = this.uniformLocations[name];

      if (location == null) {
        return;
      }

      gl.uniform2f(location, value[0], value[1]);
    }

    setUniform3f(name, value) {
      const gl = this.gl;
      const location = this.uniformLocations[name];

      if (location == null) {
        return;
      }

      gl.uniform3f(location, value[0], value[1], value[2]);
    }

    setUniform(name, value) {
      if (value.constructor === THREE.Matrix4) {
        this.setUniformMatrix4(name, value);
      } else if (typeof value === 'number') {
        this.setUniform1f(name, value);
      } else if (typeof value === 'boolean') {
        this.setUniformBoolean(name, value);
      } else if (value instanceof WebGLTexture) {
        this.setUniformTexture(name, value);
      } else if (value instanceof Array) {
        if (value.length === 2) {
          this.setUniform2f(name, value);
        } else if (value.length === 3) {
          this.setUniform3f(name, value);
        }
      } else {
        console.error('Potree: Unhandled uniform type: ', name, value);
      }
    }

    setUniform1i(name, value) {
      var gl = this.gl;
      var location = this.uniformLocations[name];

      if (location == null) {
        return;
      }

      gl.uniform1i(location, value);
    }
  }

  class WebGLBuffer {
    constructor() {
      this.numElements = 0;
      this.vao = null;
      this.vbos = new Map();
    }
  }

  /**
   * Potree object is a wrapper to use Potree alongside other THREE based frameworks.
   *
   * The object can be used a normal Object3D.
   *
   * It is based on THREE.Mesh and automatically updates the point cloud based on visibility.
   *
   * Also takes care of geometry ajustments to allow the point clouds to be frustum culled.
   */
  class BasicGroup extends THREE.Mesh {
    constructor() {
      super(
        new THREE.Geometry(),
        new THREE.MeshBasicMaterial({
          opacity: 0.0,
          wireframe: false,
          transparent: true,
        })
      );

      this.rotation.set(-Math.PI / 2, 0, 0);

      this.frustumCulled = true;
      this.pointclouds = [];

      this.nodeSize = 30;
      this.pointBudget = 1e10; //TODO <NOT USED>
      this.nodeLoadRate = 2; //TODO <NOT USED>
    }

    /**
     * Empty raycast method to avoid getting valid collision detection with the box geometry attached.
     */
    raycast(raycaster, intersects) {}

    /**
     * Changes the point budget to be used by potree.
     */
    setPointBudget(budget) {
      this.pointBudget = budget;
    }

    /**
     * Used to update the point cloud visibility relative to a camera.
     *
     * Called automatically before rendering.
     */
    onBeforeRender(renderer, scene, camera, geometry, material, group) {
      for (var i = 0; i < this.pointclouds.length; i++) {
        this.pointclouds[i].minimumNodePixelSize = this.nodeSize;
      }

      updatePointClouds(this.pointclouds, camera, renderer);
    }

    /**
     * Recalculate the box geometry attached to this group.
     *
     * The geometry its not visible and its only used for frustum culling.
     */
    recalculateBoxGeometry() {
      var box = this.getBoundingBox();

      var size = box.getSize(new THREE.Vector3());
      var center = box.getCenter(new THREE.Vector3());

      var matrix = new THREE.Matrix4();
      matrix.makeTranslation(center.x, -center.z, center.y);

      var geometry = new THREE.BoxBufferGeometry(size.x, size.z, size.y);
      geometry.applyMatrix(matrix);

      this.geometry = geometry;
    }

    /**
     * Add an object as children of this scene.
     *
     * Point cloud objects are detected and used to recalculate the geometry box used for frustum culling.
     */
    add(object) {
      THREE.Object3D.prototype.add.call(this, object);

      if (object instanceof PointCloudTree) {
        object.showBoundingBox = false;
        object.generateDEM = false;
        this.pointclouds.push(object);
        this.recalculateBoxGeometry();
      }
    }

    /**
     * Remove object from group.
     *
     * Point cloud objects are detected and used to recalculate the geometry box used for frustum culling
     */
    remove(object) {
      THREE.Object3D.prototype.remove.call(this, object);

      if (object instanceof PointCloudTree) {
        var index = this.pointclouds.indexOf(object);
        if (index !== -1) {
          this.pointclouds.splice(index, 1);
          this.recalculateBoxGeometry();
        }
      }
    }

    /**
     * Get the point cloud bouding box.
     */
    getBoundingBox() {
      var box = new THREE.Box3();

      this.updateMatrixWorld(true);

      for (var i = 0; i < this.pointclouds.length; i++) {
        var pointcloud = this.pointclouds[i];
        pointcloud.updateMatrixWorld(true);
        var pointcloudBox = pointcloud.pcoGeometry.tightBoundingBox
          ? pointcloud.pcoGeometry.tightBoundingBox
          : pointcloud.boundingBox;
        var boxWorld = HelperUtils.computeTransformedBoundingBox(
          pointcloudBox,
          pointcloud.matrixWorld
        );
        box.union(boxWorld);
      }

      return box;
    }

    /**
     * Estimate the point cloud height at a given position.
     */
    estimateHeightAt(position) {
      var height = null;
      var fromSpacing = Infinity;

      for (var pointcloud of this.pointclouds) {
        if (pointcloud.root.geometryNode === undefined) {
          continue;
        }

        var pHeight = null;
        var pFromSpacing = Infinity;

        var lpos = position.clone().sub(pointcloud.position);
        lpos.z = 0;
        var ray = new THREE.Ray(lpos, new THREE.Vector3(0, 0, 1));

        var stack = [pointcloud.root];
        while (stack.length > 0) {
          var node = stack.pop();
          var box = node.getBoundingBox();
          var inside = ray.intersectBox(box);

          if (!inside) {
            continue;
          }

          var h =
            node.geometryNode.mean.z +
            pointcloud.position.z +
            node.geometryNode.boundingBox.min.z;

          if (node.geometryNode.spacing <= pFromSpacing) {
            pHeight = h;
            pFromSpacing = node.geometryNode.spacing;
          }

          for (var index of Object.keys(node.children)) {
            var child = node.children[index];
            if (child.geometryNode) {
              stack.push(node.children[index]);
            }
          }
        }

        if (height === null || pFromSpacing < fromSpacing) {
          height = pHeight;
          fromSpacing = pFromSpacing;
        }
      }

      return height;
    }
  }

  class Group extends BasicGroup {
    constructor() {
      super();

      this.buffers = new Map();
      this.shaders = new Map();
      this.textures = new Map();
      this.types = new Map();
    }

    /**
     * Get WebGL extensions required for the more advanced features.
     */
    getExtensions(gl) {
      this.types.set(Float32Array, gl.FLOAT);
      this.types.set(Uint8Array, gl.UNSIGNED_BYTE);
      this.types.set(Uint16Array, gl.UNSIGNED_SHORT);

      var extVAO = gl.getExtension('OES_vertex_array_object');
      gl.createVertexArray = extVAO.createVertexArrayOES.bind(extVAO);
      gl.bindVertexArray = extVAO.bindVertexArrayOES.bind(extVAO);
    }

    /**
     * Update the potree group before rendering.
     */
    onBeforeRender(renderer, scene, camera, geometry, material, group) {
      super.onBeforeRender(renderer, scene, camera, geometry, material, group);

      var gl = renderer.context;
      if (gl.bindVertexArray === undefined) {
        this.getExtensions(gl);
      }

      var result = this.fetchOctrees();

      for (var octree of result.octrees) {
        var nodes = octree.visibleNodes;
        this.renderOctree(renderer, octree, nodes, camera);
      }

      gl.activeTexture(gl.TEXTURE1);
      gl.bindTexture(gl.TEXTURE_2D, null);

      renderer.state.reset();
    }

    createBuffer(gl, geometry) {
      var webglBuffer = new WebGLBuffer();
      webglBuffer.vao = gl.createVertexArray();
      webglBuffer.numElements = geometry.attributes.position.count;

      gl.bindVertexArray(webglBuffer.vao);

      for (var attributeName in geometry.attributes) {
        var bufferAttribute = geometry.attributes[attributeName];

        var vbo = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
        gl.bufferData(gl.ARRAY_BUFFER, bufferAttribute.array, gl.STATIC_DRAW);

        var attributeLocation = AttributeLocations[attributeName];
        var normalized = bufferAttribute.normalized;
        var type = this.types.get(bufferAttribute.array.constructor);

        if (type !== undefined) {
          gl.vertexAttribPointer(
            attributeLocation,
            bufferAttribute.itemSize,
            type,
            normalized,
            0,
            0
          );
          gl.enableVertexAttribArray(attributeLocation);
        }

        webglBuffer.vbos.set(attributeName, {
          handle: vbo,
          name: attributeName,
          count: bufferAttribute.count,
          itemSize: bufferAttribute.itemSize,
          type: geometry.attributes.position.array.constructor,
          version: 0,
        });
      }

      gl.bindBuffer(gl.ARRAY_BUFFER, null);
      gl.bindVertexArray(null);

      return webglBuffer;
    }

    updateBuffer(gl, geometry) {
      var webglBuffer = this.buffers.get(geometry);

      gl.bindVertexArray(webglBuffer.vao);

      for (var attributeName in geometry.attributes) {
        var bufferAttribute = geometry.attributes[attributeName];

        var attributeLocation = AttributeLocations[attributeName];
        var normalized = bufferAttribute.normalized;
        var type = this.types.get(bufferAttribute.array.constructor);

        var vbo = null;
        if (!webglBuffer.vbos.has(attributeName)) {
          vbo = gl.createBuffer();

          webglBuffer.vbos.set(attributeName, {
            handle: vbo,
            name: attributeName,
            count: bufferAttribute.count,
            itemSize: bufferAttribute.itemSize,
            type: geometry.attributes.position.array.constructor,
            version: bufferAttribute.version,
          });
        } else {
          vbo = webglBuffer.vbos.get(attributeName).handle;
          webglBuffer.vbos.get(attributeName).version = bufferAttribute.version;
        }

        gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
        gl.bufferData(gl.ARRAY_BUFFER, bufferAttribute.array, gl.STATIC_DRAW);
        gl.vertexAttribPointer(
          attributeLocation,
          bufferAttribute.itemSize,
          type,
          normalized,
          0,
          0
        );
        gl.enableVertexAttribArray(attributeLocation);
      }

      gl.bindBuffer(gl.ARRAY_BUFFER, null);
      gl.bindVertexArray(null);
    }

    fetchOctrees() {
      var octrees = [];
      var stack = [this];

      while (stack.length > 0) {
        var node = stack.pop();

        if (node instanceof PointCloudTree) {
          octrees.push(node);
          continue;
        }

        var visibleChildren = node.children.filter(c => c.visible);
        stack.push(...visibleChildren);
      }

      var result = {
        octrees: octrees,
      };

      return result;
    }

    renderNodes(
      renderer,
      octree,
      nodes,
      visibilityTextureData,
      camera,
      shader
    ) {
      var gl = renderer.context;
      var material = octree.material;
      var view = camera.matrixWorldInverse;

      var worldView = new THREE.Matrix4();
      var mat4holder = new Float32Array(16);

      for (var node of nodes) {
        if (Global.debug.allowedNodes !== undefined) {
          if (!Global.debug.allowedNodes.includes(node.name)) {
            continue;
          }
        }

        var world = node.sceneNode.matrixWorld;
        worldView.multiplyMatrices(view, world);

        if (visibilityTextureData) {
          var vnStart = visibilityTextureData.offsets.get(node);
          shader.setUniform1f('uVNStart', vnStart);
        }

        var level = node.getLevel();
        shader.setUniform('uDebug', node.debug === true);

        var isLeaf;
        if (node instanceof PointCloudOctreeNode) {
          isLeaf = Object.keys(node.children).length === 0;
        } else if (node instanceof PointCloudArena4DNode) {
          isLeaf = node.geometryNode.isLeaf;
        }
        shader.setUniform('uIsLeafNode', isLeaf);

        //TODO <consider passing matrices in an array to avoid uniformMatrix4fv overhead>
        var lModel = shader.uniformLocations['modelMatrix'];
        if (lModel) {
          mat4holder.set(world.elements);
          gl.uniformMatrix4fv(lModel, false, mat4holder);
        }

        var lModelView = shader.uniformLocations['modelViewMatrix'];
        mat4holder.set(worldView.elements);
        gl.uniformMatrix4fv(lModelView, false, mat4holder);

        //Clip Polygons
        if (material.clipPolygons && material.clipPolygons.length > 0) {
          var clipPolygonVCount = [];
          var worldViewProjMatrices = [];

          for (var clipPolygon of material.clipPolygons) {
            var view = clipPolygon.viewMatrix;
            var proj = clipPolygon.projMatrix;

            var worldViewProj = proj
              .clone()
              .multiply(view)
              .multiply(world);

            clipPolygonVCount.push(clipPolygon.markers.length);
            worldViewProjMatrices.push(worldViewProj);
          }

          var flattenedMatrices = [].concat(
            ...worldViewProjMatrices.map(m => m.elements)
          );

          var flattenedVertices = new Array(
            8 * 3 * material.clipPolygons.length
          );
          for (var i = 0; i < material.clipPolygons.length; i++) {
            var clipPolygon = material.clipPolygons[i];

            for (var j = 0; j < clipPolygon.markers.length; j++) {
              flattenedVertices[i * 24 + (j * 3 + 0)] =
                clipPolygon.markers[j].position.x;
              flattenedVertices[i * 24 + (j * 3 + 1)] =
                clipPolygon.markers[j].position.y;
              flattenedVertices[i * 24 + (j * 3 + 2)] =
                clipPolygon.markers[j].position.z;
            }
          }

          var lClipPolygonVCount =
            shader.uniformLocations['uClipPolygonVCount[0]'];
          gl.uniform1iv(lClipPolygonVCount, clipPolygonVCount);

          var lClipPolygonVP = shader.uniformLocations['uClipPolygonWVP[0]'];
          gl.uniformMatrix4fv(lClipPolygonVP, false, flattenedMatrices);

          var lClipPolygons =
            shader.uniformLocations['uClipPolygonVertices[0]'];
          gl.uniform3fv(lClipPolygons, flattenedVertices);
        }

        shader.setUniform1f('uLevel', level);
        shader.setUniform1f('uNodeSpacing', node.geometryNode.estimatedSpacing);
        shader.setUniform1f('uPCIndex', i);

        /*
        if(shadowMaps.length > 0)
        {
          var lShadowMap = shader.uniformLocations["uShadowMap[0]"];

          shader.setUniform3f("uShadowColor", material.uniforms.uShadowColor.value);

          var bindingStart = 5;
          var bindingPoints = new Array(shadowMaps.length).fill(bindingStart).map((a, i) => (a + i));
          gl.uniform1iv(lShadowMap, bindingPoints);

          for(var i = 0; i < shadowMaps.length; i++)
          {
            var shadowMap = shadowMaps[i];
            var bindingPoint = bindingPoints[i];
            var glTexture = renderer.properties.get(shadowMap.target.texture).__webglTexture;

            gl.activeTexture(gl[`TEXTURE${bindingPoint}`]);
            gl.bindTexture(gl.TEXTURE_2D, glTexture);
          }

          var worldViewMatrices = shadowMaps.map(sm => sm.camera.matrixWorldInverse).map(view => new THREE.Matrix4().multiplyMatrices(view, world))

          var flattenedMatrices = [].concat(...worldViewMatrices.map(c => c.elements));
          var lWorldView = shader.uniformLocations["uShadowWorldView[0]"];
          gl.uniformMatrix4fv(lWorldView, false, flattenedMatrices);

          flattenedMatrices = [].concat(...shadowMaps.map(sm => sm.camera.projectionMatrix.elements));
          var lProj = shader.uniformLocations["uShadowProj[0]"];
          gl.uniformMatrix4fv(lProj, false, flattenedMatrices);
        }
        */

        var geometry = node.geometryNode.geometry;
        var webglBuffer = null;
        if (!this.buffers.has(geometry)) {
          webglBuffer = this.createBuffer(gl, geometry);
          this.buffers.set(geometry, webglBuffer);
        } else {
          webglBuffer = this.buffers.get(geometry);
          for (var attributeName in geometry.attributes) {
            var attribute = geometry.attributes[attributeName];
            if (
              attribute.version > webglBuffer.vbos.get(attributeName).version
            ) {
              this.updateBuffer(gl, geometry);
            }
          }
        }

        gl.bindVertexArray(webglBuffer.vao);
        gl.drawArrays(gl.POINTS, 0, webglBuffer.numElements);
      }

      gl.bindVertexArray(null);
    }

    renderOctree(renderer, octree, nodes, camera) {
      var gl = renderer.context;
      var material = octree.material;
      var shadowMaps = [];
      var view = camera.matrixWorldInverse;
      var viewInv = camera.matrixWorld;
      var proj = camera.projectionMatrix;
      var projInv = new THREE.Matrix4().getInverse(proj);
      var worldView = new THREE.Matrix4();

      var visibilityTextureData = null;
      var currentTextureBindingPoint = 0;

      if (
        material.pointSizeType === PointSizeType.ADAPTIVE ||
        material.pointColorType === PointColorType.LOD
      ) {
        visibilityTextureData = octree.computeVisibilityTextureData(
          nodes,
          camera
        );

        var vnt = material.visibleNodesTexture;
        vnt.image.data.set(visibilityTextureData.data);
        vnt.needsUpdate = true;
      }

      var shader = null;

      if (!this.shaders.has(material)) {
        shader = new Shader(
          gl,
          'pointcloud',
          material.vertexShader,
          material.fragmentShader
        );
        this.shaders.set(material, shader);
      } else {
        shader = this.shaders.get(material);
      }

      var numSnapshots = material.snapEnabled ? material.numSnapshots : 0;
      var numClipBoxes =
        material.clipBoxes && material.clipBoxes.length
          ? material.clipBoxes.length
          : 0;
      var numClipPolygons =
        material.clipPolygons && material.clipPolygons.length
          ? material.clipPolygons.length
          : 0;
      var numClipSpheres = 0;

      var defines = [
        '#define num_shadowmaps' + shadowMaps.length,
        '#define num_snapshots' + numSnapshots,
        '#define num_clipboxes' + numClipBoxes,
        '#define num_clipspheres' + numClipSpheres,
        '#define num_clippolygons' + numClipPolygons,
      ];

      var definesString = defines.join('\n');
      var vs = definesString + '\n' + material.vertexShader;
      var fs = definesString + '\n' + material.fragmentShader;

      shader.update(vs, fs);

      material.needsUpdate = false;

      for (var uniformName of Object.keys(material.uniforms)) {
        var uniform = material.uniforms[uniformName];

        if (uniform.type == 't') {
          var texture = uniform.value;

          if (!texture) {
            continue;
          }

          if (!this.textures.has(texture)) {
            var webglTexture = new WebGLTexture(gl, texture);
            this.textures.set(texture, webglTexture);
          }

          var webGLTexture = this.textures.get(texture);
          webGLTexture.update();
        }
      }

      gl.useProgram(shader.program);

      if (material.opacity < 1.0) {
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE);
        gl.depthMask(false);
        gl.disable(gl.DEPTH_TEST);
      } else {
        // EDITED: FUCKS WITH TRANSPARENT SHIT
        // gl.disable(gl.BLEND);
        gl.depthMask(true);
        gl.enable(gl.DEPTH_TEST);
      }

      //Update shader uniforms
      shader.setUniformMatrix4('projectionMatrix', proj);
      shader.setUniformMatrix4('viewMatrix', view);
      shader.setUniformMatrix4('uViewInv', viewInv);
      shader.setUniformMatrix4('uProjInv', projInv);

      var screenWidth = material.screenWidth;
      var screenHeight = material.screenHeight;

      shader.setUniform1f('uScreenWidth', screenWidth);
      shader.setUniform1f('uScreenHeight', screenHeight);
      shader.setUniform1f('fov', (Math.PI * camera.fov) / 180);
      shader.setUniform1f('near', camera.near);
      shader.setUniform1f('far', camera.far);

      //Set log
      if (renderer.capabilities.logarithmicDepthBuffer) {
        shader.setUniform(
          'logDepthBufFC',
          2.0 / (Math.log(camera.far + 1.0) / Math.LN2)
        );
      }

      //Camera configuration
      if (camera instanceof THREE.OrthographicCamera) {
        shader.setUniform('uUseOrthographicCamera', true);
        shader.setUniform('uOrthoWidth', camera.right - camera.left);
        shader.setUniform('uOrthoHeight', camera.top - camera.bottom);
      } else {
        shader.setUniform('uUseOrthographicCamera', false);
      }

      //Clip task
      if (material.clipBoxes.length + material.clipPolygons.length === 0) {
        shader.setUniform1i('clipTask', ClipTask.NONE);
      } else {
        shader.setUniform1i('clipTask', material.clipTask);
      }

      shader.setUniform1i('clipMethod', material.clipMethod);

      //Clipboxes
      if (material.clipBoxes && material.clipBoxes.length > 0) {
        var lClipBoxes = shader.uniformLocations['clipBoxes[0]'];
        gl.uniformMatrix4fv(
          lClipBoxes,
          false,
          material.uniforms.clipBoxes.value
        );
      }

      //Clispheres
      /*if(material.clipSpheres && material.clipSpheres.length > 0)
      {
        var clipSpheres = material.clipSpheres;
        var matrices = [];
        for(var clipSphere of clipSpheres)
        {
          var clipToWorld = clipSphere.matrixWorld;
          var viewToWorld = camera.matrixWorld
          var worldToClip = new THREE.Matrix4().getInverse(clipToWorld);

          var viewToClip = new THREE.Matrix4().multiplyMatrices(worldToClip, viewToWorld);

          matrices.push(viewToClip);
        }

        var flattenedMatrices = [].concat(...matrices.map(matrix => matrix.elements));

        var lClipSpheres = shader.uniformLocations["uClipSpheres[0]"];
        gl.uniformMatrix4fv(lClipSpheres, false, flattenedMatrices);
      }*/

      shader.setUniform1f('size', material.size);
      shader.setUniform1f('maxSize', material.uniforms.maxSize.value);
      shader.setUniform1f('minSize', material.uniforms.minSize.value);
      shader.setUniform1f('uOctreeSpacing', material.spacing);
      shader.setUniform('uOctreeSize', material.uniforms.octreeSize.value);
      shader.setUniform3f('uColor', material.color.toArray());
      shader.setUniform1f('uOpacity', material.opacity);
      shader.setUniform1f('uTime', material.uniforms.uTime.value);
      shader.setUniform2f('elevationRange', material.elevationRange);
      shader.setUniform2f('intensityRange', material.intensityRange);
      shader.setUniform1f('intensityGamma', material.intensityGamma);
      shader.setUniform1f('intensityContrast', material.intensityContrast);
      shader.setUniform1f('intensityBrightness', material.intensityBrightness);
      shader.setUniform1f('rgbGamma', material.rgbGamma);
      shader.setUniform1f('rgbContrast', material.rgbContrast);
      shader.setUniform1f('rgbBrightness', material.rgbBrightness);
      shader.setUniform1f('uTransition', material.transition);
      shader.setUniform1f('wRGB', material.weightRGB);
      shader.setUniform1f('wIntensity', material.weightIntensity);
      shader.setUniform1f('wElevation', material.weightElevation);
      shader.setUniform1f('wClassification', material.weightClassification);
      shader.setUniform1f('wReturnNumber', material.weightReturnNumber);
      shader.setUniform1f('wSourceID', material.weightSourceID);

      var vnWebGLTexture = this.textures.get(material.visibleNodesTexture);
      shader.setUniform1i('visibleNodesTexture', currentTextureBindingPoint);
      gl.activeTexture(gl.TEXTURE0 + currentTextureBindingPoint);
      gl.bindTexture(vnWebGLTexture.target, vnWebGLTexture.id);
      currentTextureBindingPoint++;

      var gradientTexture = this.textures.get(material.gradientTexture);
      shader.setUniform1i('gradient', currentTextureBindingPoint);
      gl.activeTexture(gl.TEXTURE0 + currentTextureBindingPoint);
      gl.bindTexture(gradientTexture.target, gradientTexture.id);
      currentTextureBindingPoint++;

      var classificationTexture = this.textures.get(
        material.classificationTexture
      );
      shader.setUniform1i('classificationLUT', currentTextureBindingPoint);
      gl.activeTexture(gl.TEXTURE0 + currentTextureBindingPoint);
      gl.bindTexture(classificationTexture.target, classificationTexture.id);
      currentTextureBindingPoint++;

      if (material.snapEnabled === true) {
        var lSnapshot = shader.uniformLocations['uSnapshot[0]'];
        var lSnapshotDepth = shader.uniformLocations['uSnapshotDepth[0]'];

        var bindingStart = currentTextureBindingPoint;
        var lSnapshotBindingPoints = new Array(5)
          .fill(bindingStart)
          .map((a, i) => a + i);
        var lSnapshotDepthBindingPoints = new Array(5)
          .fill(1 + Math.max(...lSnapshotBindingPoints))
          .map((a, i) => a + i);
        currentTextureBindingPoint =
          1 + Math.max(...lSnapshotDepthBindingPoints);

        gl.uniform1iv(lSnapshot, lSnapshotBindingPoints);
        gl.uniform1iv(lSnapshotDepth, lSnapshotDepthBindingPoints);

        for (var i = 0; i < 5; i++) {
          var texture = material.uniforms['uSnapshot'].value[i];
          var textureDepth = material.uniforms['uSnapshotDepth'].value[i];

          if (!texture) {
            break;
          }

          var snapTexture = renderer.properties.get(texture).__webglTexture;
          var snapTextureDepth = renderer.properties.get(textureDepth)
            .__webglTexture;

          var bindingPoint = lSnapshotBindingPoints[i];
          var depthBindingPoint = lSnapshotDepthBindingPoints[i];

          gl.activeTexture(gl[`TEXTURE${bindingPoint}`]);
          gl.bindTexture(gl.TEXTURE_2D, snapTexture);

          gl.activeTexture(gl[`TEXTURE${depthBindingPoint}`]);
          gl.bindTexture(gl.TEXTURE_2D, snapTextureDepth);
        }

        var flattenedMatrices = [].concat(
          ...material.uniforms.uSnapView.value.map(c => c.elements)
        );
        var lSnapView = shader.uniformLocations['uSnapView[0]'];
        gl.uniformMatrix4fv(lSnapView, false, flattenedMatrices);

        flattenedMatrices = [].concat(
          ...material.uniforms.uSnapProj.value.map(c => c.elements)
        );
        var lSnapProj = shader.uniformLocations['uSnapProj[0]'];
        gl.uniformMatrix4fv(lSnapProj, false, flattenedMatrices);

        flattenedMatrices = [].concat(
          ...material.uniforms.uSnapProjInv.value.map(c => c.elements)
        );
        var lSnapProjInv = shader.uniformLocations['uSnapProjInv[0]'];
        gl.uniformMatrix4fv(lSnapProjInv, false, flattenedMatrices);

        flattenedMatrices = [].concat(
          ...material.uniforms.uSnapViewInv.value.map(c => c.elements)
        );
        var lSnapViewInv = shader.uniformLocations['uSnapViewInv[0]'];
        gl.uniformMatrix4fv(lSnapViewInv, false, flattenedMatrices);
      }

      this.renderNodes(
        renderer,
        octree,
        nodes,
        visibilityTextureData,
        camera,
        shader
      );

      gl.activeTexture(gl.TEXTURE2);
      gl.bindTexture(gl.TEXTURE_2D, null);
      gl.activeTexture(gl.TEXTURE0);
    }
  }

  exports.Global = Global;
  exports.AttributeLocations = AttributeLocations;
  exports.Classification = Classification;
  exports.ClipTask = ClipTask;
  exports.ClipMethod = ClipMethod;
  exports.PointSizeType = PointSizeType;
  exports.PointShape = PointShape;
  exports.PointColorType = PointColorType;
  exports.TreeType = TreeType;
  exports.loadPointCloud = loadPointCloud;
  exports.updateVisibility = updateVisibility;
  exports.updatePointClouds = updatePointClouds;
  exports.updateVisibilityStructures = updateVisibilityStructures;
  exports.BinaryHeap = BinaryHeap;
  exports.LRU = LRU;
  exports.HelperUtils = HelperUtils;
  exports.VersionUtils = VersionUtils;
  exports.WorkerManager = WorkerManager;
  exports.PointAttribute = PointAttribute;
  exports.PointAttributes = PointAttributes;
  exports.PointAttributeNames = PointAttributeNames;
  exports.PointAttributeTypes = PointAttributeTypes;
  exports.Gradients = Gradients;
  exports.Points = Points;
  exports.Shader = Shader;
  exports.WebGLTexture = WebGLTexture;
  exports.WebGLBuffer = WebGLBuffer;
  exports.Shaders = Shaders;
  exports.DEM = DEM$1;
  exports.DEMNode = DEMNode;
  exports.PointCloudTree = PointCloudTree;
  exports.PointCloudArena4D = PointCloudArena4D;
  exports.PointCloudOctree = PointCloudOctree;
  exports.PointCloudOctreeGeometry = PointCloudOctreeGeometry;
  exports.PointCloudArena4DGeometry = PointCloudArena4DGeometry;
  exports.PointCloudGreyhoundGeometry = PointCloudGreyhoundGeometry;
  exports.PointCloudEptGeometry = PointCloudEptGeometry;
  exports.PointCloudMaterial = PointCloudMaterial;
  exports.LASLoader = LASLoader;
  exports.BinaryLoader = BinaryLoader;
  exports.GreyhoundUtils = GreyhoundUtils;
  exports.GreyhoundLoader = GreyhoundLoader;
  exports.GreyhoundBinaryLoader = GreyhoundBinaryLoader;
  exports.POCLoader = POCLoader;
  exports.LASLAZLoader = LASLAZLoader;
  exports.EptLoader = EptLoader;
  exports.EptLaszipLoader = EptLaszipLoader;
  exports.EptBinaryLoader = EptBinaryLoader;
  exports.BasicGroup = BasicGroup;
  exports.Group = Group;

  Object.defineProperty(exports, '__esModule', { value: true });
});
