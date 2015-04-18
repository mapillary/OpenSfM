var Dijkstra = (function () {

     /**
     * A class for calculations on graphs using Dijkstra's algorithm.
     * @constructor
     */
    function Dijkstra() {
    }

    // Private sort delegate for ordering key value pairs arranged
    // as an array of two items like [key, value].
    var keyValueSorter = function (kv1, kv2) {
        return parseFloat(kv1[1]) - parseFloat(kv2[1]);
    }

     /**
     * Private function for running Dijkstra's Algorithm until an evaluation function decides to stop.
     * @param {Object} graph The graph with nodes and weights used for calculation.
     * @param {String} source The name of the source node.
     * @param {String} weight The name of the weight property.
     * @param {Function} evaluationFunc Function taking the current node and current distance as parameters
     *                   and returns true if the algorithm should finish.
     * @return {Object} An object with properties for the visited nodes, the previous nodes and the distances from
     *                  the source node.
     */
    var dijkstra = function(graph, source, weight, evaluationFunc) {
        var touchedNodes = {};
        var previous = {};
        var distances = {};
        var visited = {};

        touchedNodes[source] = 0;
        previous[source] = undefined;
        distances[source] = 0;

        while (true) {
            var touched = [];
            for (var key in touchedNodes) {
                if (Object.prototype.hasOwnProperty.call(touchedNodes, key)) {
                    touched.push([key, touchedNodes[key]])
                }
            }

            // Stop if none of the unvisited nodes can be reached.
            if (!touched.length) {
                break;
            }

            // Select the unvisited node with smallest distance and mark it as current node.
            touched.sort(keyValueSorter);
            var currentNode = touched[0][0];
            var currentDistance = touched[0][1]

            visited[currentNode] = true;
            delete touchedNodes[currentNode];

            // Return if the evaluation of the current position returns true..
            if (evaluationFunc(currentNode, currentDistance)) {
                break;
            }

            var currentEdges = graph.edges[currentNode] || {};

            for (var node in currentEdges) {
                if (Object.prototype.hasOwnProperty.call(currentEdges, node)) {

                    // Do not process already visited nodes.
                    if (Object.prototype.hasOwnProperty.call(visited, node)) {
                        continue;
                    }

                    // Calculate the total distance from the source to the node of
                    // the current edge.
                    var distance = currentEdges[node][weight];
                    var totalDistance = distances[currentNode] + distance;

                    if (!distances[node] || totalDistance < distances[node])
                    {
                        distances[node] = totalDistance;
                        touchedNodes[node] = totalDistance;
                        previous[node] = currentNode;
                    }
                }
            }
        }

        return { visited: visited, distances: distances, previous: previous }
    }

     /**
     * Calculate the shortest path between two nodes in a graph using
     * Dijkstra's Algorithm.
     * @param {Object} graph The graph with nodes and weights used for calculation.
     * @param {String} source The name of the source node.
     * @param {String} target The name of the target node.
     * @param {String} weight The name of the weight property.
     * @return {Array} An array of node names corresponding to the path
     */
    Dijkstra.prototype.shortestPath = function (graph, source, target, weight) {
        if (source === target) {
            return [source];
        }

        var evaluationFunc = function (currentNode, currentDistance) {
            return currentNode === target;
        };

        var result = dijkstra(graph, source, weight, evaluationFunc);

        // No path to the target was found.
        if (result.previous[target] === undefined) {
            return null;
        }

        // Retrieve a path from the dictionary of previous nodes and reverse it.
        var reversePath = [];
        var element = target;
        while (element !== undefined) {
            reversePath.push(element);
            element = result.previous[element];
        }

        return reversePath.reverse();
    }

    /**
     * Retrieve all other nodes within a distance from a source node based on the edge weights.
     * @param {Object} graph The graph with nodes and weights used for calculation.
     * @param {String} source The name of the source node.
     * @param {String} weight The name of the weight property.
     * @param {Number} distance The maximum distance between nodes.
     * @return {Array} An array of node names corresponding to nodes within a distance
                       from the source node.
     */
    Dijkstra.prototype.nodesWithinDistance = function (graph, source, distance, weight) {
        var evaluationFunc = function (currentNode, currentDistance) {
            return currentDistance >= distance;
        };

        var result = dijkstra(graph, source, weight, evaluationFunc);

        var nodes = [];
        for (var node in result.visited) {
            if (Object.prototype.hasOwnProperty.call(result.visited, node)) {
                nodes.push(node);
            }
        }

        return nodes;
    }

    return Dijkstra;
})();

var GraphHelper = (function () {

    /**
     * A class with helper functions for graphs.
     * @constructor
     */
    function GraphHelper() {
        this.dijkstra = new Dijkstra();
    }

    // Private function for getting a graph with edges of a type.
    var getTypeGraph = function (graph, type) {
        var typeGraph = { nodes: graph.nodes, edges: {} };

        for (var k in graph.edges) {
            if (!Object.prototype.hasOwnProperty.call(graph.edges, k)) {
                continue;
            }

            typeGraph.edges[k] = {};
            var edges = graph.edges[k][type];

            for (var m in edges) {
                if (!Object.prototype.hasOwnProperty.call(edges, m)) {
                    continue;
                }

                typeGraph.edges[k][m] = {};

                edge_properties = edges[m];

                for (var ep in edge_properties) {
                    if (!Object.prototype.hasOwnProperty.call(edge_properties, ep)) {
                        continue;
                    }

                    typeGraph.edges[k][m][ep] = edge_properties[ep];
                }
            }
        }

        return typeGraph;
    }

    /**
     * Retrieves a graph with edges of a certain type.
     * @param {Object} graph The graph with nodes and weights used for calculation.
     * @param {String} type The name of the edge type.
     * @return {Array} A graph where all edges are of the specified type.
     */
    GraphHelper.prototype.getTypeGraphs = function (graphs, type) {
        var typeGraphs = [];

        for (var i = 0; i < graphs.length; i++) {
            var typeGraph = getTypeGraph(graphs[i], type)
            typeGraphs.push(typeGraph);
        }

        return typeGraphs;
    }

    /**
     * Retrieves a graph with edges of a default type. The default type edges are overridden by
     * edges of the override type for nodes within a threshold distance from a target node.
     * @param {Object} graph The graph with nodes and weights used for calculation.
     * @param {String} defaultType The name of the default edge type.
     * @param {String} overrideType The name of the override edge type.
     * @param {String} target The name of the override target node from which the other override nodes are retrieved..
     * @return {Array} A graph where all edges are of the specified type.
     */
    GraphHelper.prototype.mergeTypeGraphs = function (graphs, defaultType, overrideType, target, distance) {
        var mergedGraphs = [];

        for (var i = 0; i < graphs.length; i++) {
            var graph = graphs[i];
            var mergedGraph = getTypeGraph(graph, defaultType);

            if (graph.nodes.indexOf(target) > -1) {
                var overrideGraph = getTypeGraph(graph, overrideType);
                var overrideNodes = this.dijkstra.nodesWithinDistance(overrideGraph, target, distance, 'weight');

                for (var i = 0; i < overrideNodes.length; i++) {
                    overrideNode = overrideNodes[i];
                    mergedGraph.edges[overrideNode] = graph.edges[overrideNode][overrideType];
                }
            }

            mergedGraphs.push(mergedGraph);
        }

        return mergedGraphs;
    }

    /**
     * Creates a graph with a penalty for certain properties with certain values.
     * @param {Object} graph The graph with nodes and weights used for calculation.
     * @param {String} type The name of the weight key.
     * @param {String} type The name of the penalty key.
     * @param {Dictionary} penalties Dictionary of penalty keys with respective penalty amount.
     * @return {Object} A graph with edge weights as sum of original weight and specified penalty.
     */
    GraphHelper.prototype.getPenaltyGraph = function (graph, weightKey, penaltyKey, penalties) {

        var penaltyGraph = { edges: {} };

        for (var k in graph.edges) {
            if (!Object.prototype.hasOwnProperty.call(graph.edges, k)) {
                continue;
            }

            penaltyGraph.edges[k] = {};
            var edges = graph.edges[k];

            for (var m in edges) {
                if (!Object.prototype.hasOwnProperty.call(edges, m)) {
                    continue;
                }

                penaltyGraph.edges[k][m] = {};

                // Add penalty to weight if the value of the penalty key corresponds
                // to the specified penalty value.
                if (edges[m][penaltyKey] in penalties) {
                    penaltyGraph.edges[k][m][weightKey] = edges[m][weightKey] + penalties[edges[m][penaltyKey]];
                }
                else {
                    penaltyGraph.edges[k][m][weightKey] = edges[m][weightKey];
                }
            }
        }

        return penaltyGraph;
    }

    /**
     * Retrieves the directed edge between a start node and an adjacent end node.
     * @param {Integer} graph The graph.
     * @param {String} from The name of the node for which the edge starts.
     * @param {String} to The name of the node for which the edge ends.
     * @return {Dictionary} The edge properties.
     */
    GraphHelper.prototype.getEdgeProperties = function(graph, from, to) {
        var nodeEdges = graph.edges[from];
        var edgeProperties = nodeEdges[to];
        return edgeProperties;
    }

    return GraphHelper;
})();

var LinearCurve = THREE.Curve.create(

	function (points) {

		this.points = (points == undefined) ? [] : points;
	},

	function (t) {

		var points = this.points;
		var point = (points.length - 1) * t;

		var intPoint = Math.floor(point);
		var weight = point - intPoint;

		var point1 = points[intPoint];
		var point2 = points[intPoint > points.length - 2 ? points.length - 1 : intPoint + 1];

		var vector = new THREE.Vector3();
		vector.copy(point1).lerp(point2, weight);

		return vector;
	}
);

var JourneyBase = (function () {

    /**
     * A journey base.
     * @constructor
     * @param {String} graphs A list of graphs.
     * @param {String} shots Dictionary of shots with positions and targets.
     * @param {String} intervalTime The interval for navigation.
     * @param {Object} penalty Object specifying a weight key, penalty key and a dictionary of penalty keys values
                        with penalty amounts.
     */
    function JourneyBase(graphs, shots, intervalTime, penalty) {
        this.graphs = graphs;
        this.shots = shots;
        this.intervalTime = intervalTime;
        this.penalty = penalty;

        this.started = false;
        this.preCount = 15;
        this.dijkstra = new Dijkstra();
        this.graphHelper = new GraphHelper();
    }

    /**
     * Sets the interval time.
     * @param {Integer} intervalTime
     */
    JourneyBase.prototype.updateInterval = function (intervalTime) {
        this.intervalTime = intervalTime;
    }

     /**
     * Calculate the shortest path between two nodes in a graph.
     * @param {String} from
     * @param {String} to
     * @return {Array} An array of node names corresponding to the path
     */
    JourneyBase.prototype.shortestPath = function (from, to) {
        var index = null;
        for (var i = 0; i < this.graphs.length; i++) {
            // Ensure that both nodes exist in the graph.
            if (this.graphs[i].nodes.indexOf(from) > -1 &&
                this.graphs[i].nodes.indexOf(to) > -1) {
                index = i;
                break;
            }
        }

        if (index === null) {
            return null;
        }

        var journeyGraph = this.graphs[index];
        if (this.penalty) {
            journeyGraph =
                this.graphHelper.getPenaltyGraph(
                    journeyGraph,
                    this.penalty.weightKey,
                    this.penalty.penaltyKey,
                    this.penalty.values);
        }

        var path = this.dijkstra.shortestPath(journeyGraph, from, to, 'weight');

        return path === null ? null : { path: path, index: index };
    }

    /**
     * Creates a geometry based on an existing property of the nodes in the path.
     * @param {Array} path Nodes to create geometry from.
     * @param {String} property Name of the shot property to use.
     * @return {THREE.Geometry} A geometry for the path.
     */
    JourneyBase.prototype.getGeometry = function (path, property) {
        var geometry = new THREE.Geometry();

        for (var i = 0; i < path.length; i++) {
            var shot_id = path[i];
            var vertex = this.shots[shot_id][property];
            geometry.vertices.push(new THREE.Vector3(vertex.x, vertex.y, vertex.z));
        }

        return geometry;
    }

    /**
     * Creates a geometry based on the positions of the nodes in the
     * shortest path between the specified nodes.
     * @param {String} from Name of the start node.
     * @param {String} to Name of the end node.
     * @return {THREE.Geometry} A geometry for the shortest path between the nodes.
     */
    JourneyBase.prototype.getPathGeometry = function (from, to) {

        var result = this.shortestPath(from, to);
        if (result === null || result.path.length <= 1) {
            return null;
        }

        return this.getGeometry(result.path, 'position');
    }

    /**
     * Retrieves the directed edge between a start node and an adjacent end node.
     * @param {Integer} graphIndex The index of the graph.
     * @param {String} from The name of the node for which the edge starts.
     * @param {String} to The name of the node for which the edge ends.
     * @return {Dictionary} The edge properties.
     */
    JourneyBase.prototype.getEdge = function(graphIndex, from, to) {
        return this.graphHelper.getEdgeProperties(this.graphs[graphIndex], from, to);
    }

    /**
     * Gets a value indicating whether a journey is ongoing.
     * @return {Boolean} A value indicating whether a journey is ongoing.
     */
    JourneyBase.prototype.isStarted = function () {
        return this.started;
    }

    /**
     * Virtual base class method to overwrite and implement in subclasses.
     * @param {Number} from
     * @param {Number} to
     */
    JourneyBase.prototype.start = function (from, to) {
        console.log( "Warning, start() not implemented!" );
    }

    /**
     * Virtual base class method to overwrite and implement in subclasses.
     * @param {Boolean} continuation Indicates if a continuation action should be invoked.
     */
    JourneyBase.prototype.stop = function (continuation) {
        console.log( "Warning, stop() not implemented!" );
    }

    return JourneyBase;
})();

var Journey = (function () {

    /**
     * A journey.
     * @constructor
     * @extends {JourneyBase}
     * @param {Array} graphs A list of graphs.
     * @param {Object} shots Dictionary of shots.
     * @param {Number} intervalTime The maximum time between navigation.
     * @param {Function} navigationAction The action to execute on navigation.
     * @param {Function} startAction The action to run when starting a journey.
     * @param {Function} stopAction The action to run when stopping a journey.
     * @param {Function} preloadAction The action to run when stopping a journey.
     * @param {Object} penalty Object specifying a weight key, penalty key and a dictionary of penalty keys values
                       with penalty amounts.
     */
    function Journey(
        graphs,
        shots,
        intervalTime,
        navigationAction,
        startAction,
        stopAction,
        preloadAction,
        penalty) {

        JourneyBase.apply(this, [graphs, shots, intervalTime, penalty]);

        this.navigationAction = navigationAction;
        this.startAction = startAction;
        this.stopAction = stopAction;
        this.preloadAction = preloadAction;
        this.graphIndex = null;
        this.path = null;
        this.timeoutToken = null;
        this.currentIndex = null;
    }

    // Inheriting from JourneyBase
    Journey.prototype = Object.create(JourneyBase.prototype);
    Journey.prototype.constructor = Journey;

    // Private function for calculating the interval value. The max distance of an edge is
    // 20. The interval is the weight multiplied with the desired interval time for one unit.
    // A smallest value is defined to avoid too fast navigation..
    var getInterval = function (edges, node, intervalTime) {
        var distance = edges[node].weight;
        return Math.max(distance * intervalTime, 0.7 * 1000);
    }

    // Private callback function for setInterval.
    var navigation = function () {
        if (this.started !== true) {
            this.stop();
            return;
        }

        var _this = this;
        if (!isFinite(this.intervalTime)) {
            this.timeoutToken = window.setTimeout(function () { navigation.call(_this); }, 500);
            return;
        }

        var pathLength = this.path.length;
        this.currentIndex++;

        if (this.currentIndex >= pathLength) {
            this.stop();
            return;
        }

        this.navigationAction(this.path[this.currentIndex]);

        if (this.currentIndex === pathLength - 1) {
            this.stop();
            return;
        }

        if (this.currentIndex + this.preCount <= pathLength - 1) {
            this.preloadAction([this.path[this.currentIndex + this.preCount]]);
        }

        var currentInterval =
            getInterval(
                this.graphs[this.graphIndex].edges[this.path[this.currentIndex - 1]],
                this.path[this.currentIndex],
                this.intervalTime);

        this.timeoutToken = window.setTimeout(function () { navigation.call(_this); }, currentInterval);
    }

    /**
     * Starts a journey between two nodes in a graph.
     * @param {Number} from
     * @param {Number} to
     */
    Journey.prototype.start = function (from, to) {
        if (this.started === true) {
            return;
        }

        var result = this.shortestPath(from, to);
        if (result === null || result.path.length <= 1) {
            return;
        }

        this.started = true;
        this.path = result.path;
        this.preloadAction(this.path.slice(1, Math.min(this.preCount, this.path.length)));

        this.graphIndex = result.index;
        this.currentIndex = 0;
        this.startAction();
        this.navigationAction(this.path[this.currentIndex]);

        var _this = this;
        this.timeoutToken = window.setTimeout(function () { navigation.call(_this); }, 500);
    }

    /**
     * Stops an ongoing journey between two nodes in a graph.
     */
    Journey.prototype.stop = function (continuation) {
        if (this.timeoutToken === null || this.started === false) {
            return;
        }

        window.clearTimeout(this.timeoutToken);

        this.graphIndex = null;
        this.path = null;
        this.timeoutToken = null;
        this.currentIndex = null;

        this.stopAction();

        this.started = false;
    }

    return Journey;
})();

var SmoothJourney = (function () {

    /**
     * A smooth journey.
     * @constructor
     * @extends {JourneyBase}
     * @param {Array} graphs A list of graphs.
     * @param {Object} shots Dictionary of shots.
     * @param {Number} intervalTime The maximum time between navigation.
     * @param {Function} navigationAction The action to execute on navigation.
     * @param {Function} nodeAction The action to execute when a node should be displayed.
     * @param {Function} startAction The action to run when starting a journey.
     * @param {Function} stopAction The action to run when stopping a journey.
     * @param {Function} continuationAction The action to execute when the journey is stopped for smooth stopping.
     * @param {Function} preloadAction The action to run when stopping a journey.
     * @param {Function} speedFunction Function returning speed coefficient based on the current position between nodes.
     * @param {Type} curveType The type of the curve used for movement. Must inherit from THREE.Curve.
     * @param {Object} penalty Object specifying a weight key, penalty key and a dictionary of penalty keys values
                       with penalty amounts.
     */
    function SmoothJourney(
        graphs,
        shots,
        intervalTime,
        navigationAction,
        nodeAction,
        startAction,
        stopAction,
        continuationAction,
        preloadAction,
        speedFunction,
        curveType,
        penalty) {

        JourneyBase.apply(this, [graphs, shots, intervalTime, penalty]);

        this.navigationAction = navigationAction;
        this.nodeAction = nodeAction;
        this.startAction = startAction;
        this.stopAction = stopAction;
        this.continuationAction = continuationAction;
        this.preloadAction = preloadAction;
        this.speedFunction = speedFunction;
        this.curveType = curveType;

        this.graphIndex = null;
        this.path = null;
        this.positionCurve = null;
        this.targetCurve = null;
        this.intervalToken = null;
        this.previousTime = null;
        this.currentIndex = null;
        this.u = null;
        this.t = null;
    }

    // Inheriting from JourneyBase
    SmoothJourney.prototype = Object.create(JourneyBase.prototype);
    SmoothJourney.prototype.constructor = SmoothJourney;

    // Private function for calculating the current position and target based
    // on the elapsed time, interval and the curve.
    var move = function () {
        if (this.started !== true) {
            return;
        }

        var currentTime = Date.now();

        // Pause movement if the interval time is infinity.
        if (!isFinite(this.intervalTime)) {
            this.previousTime = currentTime;
            return;
        }

        var elapsed = currentTime - this.previousTime;

        var previousPoint = (this.path.length - 1) * this.t;
        var previousIndex = Math.floor(previousPoint);
        var previousFraction = this.u >= 1 ? 1 : previousPoint - previousIndex;
        var previousEdge = this.getEdge(this.graphIndex, this.path[this.currentIndex], this.path[this.currentIndex + 1]);

        var speedCoefficient = this.speedFunction(previousFraction, previousEdge);
        elapsed = speedCoefficient * elapsed;

        this.previousTime = currentTime;
        var totalTime = this.intervalTime * this.positionCurve.getLength();

        this.u = Math.min(this.u + (elapsed / totalTime), 1);

        // Retrieve t from the position curve to calculate index.
        this.t = this.positionCurve.getUtoTmapping(this.u);
        var point = (this.path.length - 1) * this.t;
        var index = Math.floor(point);

        if (index > this.currentIndex && index < this.path.length - 1) {
            this.currentIndex = index;

            var startIndex = Math.min(2 + this.currentIndex * 3, this.currentIndex + this.preCount);
            var endIndex = Math.min(5 + this.currentIndex * 3, this.currentIndex + this.preCount + 1);

            if (endIndex <= this.path.length) {
                this.preloadAction(this.path.slice(startIndex, endIndex));
            }

            this.nodeAction(this.path[this.currentIndex + 1]);
        }

        var position = this.positionCurve.getPoint(this.t);
        var target = this.targetCurve.getPoint(this.t);

        // Do not reset the weight after reaching the last node.
        var fraction = this.u >= 1 ? 1 : point - index;
        var edge = this.getEdge(this.graphIndex, this.path[this.currentIndex], this.path[this.currentIndex + 1]);

        this.navigationAction(position, target, fraction, edge);

        if (this.u >= 1) {
            this.stop(false);
        }
    }

    /**
     * Starts a smooth journey between two nodes in a graph.
     * @param {Number} from Start node.
     * @param {Number} to End node.
     */
    SmoothJourney.prototype.start = function (from, to) {
        if (this.started === true) {
            return;
        }

        var result = this.shortestPath(from, to);
        if (result === null || result.path.length <= 1) {
            return;
        }

        this.started = true;
        this.path = result.path;
        this.graphIndex = result.index;

        var startIndex = Math.min(2, this.path.length - 1);
        var endIndex = Math.min(5, this.path.length);
        this.preloadAction(this.path.slice(startIndex, endIndex));

        var positions = this.getGeometry(this.path, 'position').vertices;
        var targets = this.getGeometry(this.path, 'target').vertices;

        this.positionCurve = new (Function.prototype.bind.apply(this.curveType, [null, positions]));
        this.targetCurve = new (Function.prototype.bind.apply(this.curveType, [null, targets]));

        this.previousTime = Date.now();
        this.u = 0;
        this.t = 0;
        this.currentIndex = 0;

        this.startAction();

        this.nodeAction(this.path[this.currentIndex + 1]);

        var position = this.positionCurve.getPointAt(0);
        var target = this.targetCurve.getPointAt(0);

        var edge = this.getEdge(this.graphIndex, this.path[this.currentIndex], this.path[this.currentIndex + 1]);
        this.navigationAction(position, target, 0, edge);

        _this = this;
        this.intervalToken = window.setInterval(function () { move.call(_this); }, 1000/60);
    }

    /**
     * Stops a smooth journey.
     * @param {Boolean} continuation Specifying if the continuation action should be invoked.
     */
    SmoothJourney.prototype.stop = function (continuation) {
        if (this.intervalToken === null || this.started === false) {
            return;
        }

        window.clearInterval(this.intervalToken);
        this.intervalToken = null;

        var nextIndex = Math.min(this.currentIndex + 1, this.path.length - 1);
        var nextNode = this.path[nextIndex];

        this.graphIndex = null;
        this.path = null;
        this.positionCurve = null;
        this.targetCurve = null;
        this.previousTime = null;
        this.currentIndex = null;
        this.u = null;
        this.t = null;

        if (continuation === true) {
            this.continuationAction(nextNode);
        }

        this.stopAction();

        this.started = false;
    }

    /**
     * Sets the curve type for a smooth journey.
     * @param {Type} curveType The type of the curve used for movement. Must inherit from THREE.Curve.
     */
    SmoothJourney.prototype.setCurveType = function (curveType) {
        this.curveType = curveType;

        if (this.started === false) {
            return;
        }

        var positions = this.getGeometry(this.path, 'position').vertices;
        var targets = this.getGeometry(this.path, 'target').vertices;

        this.positionCurve = new (Function.prototype.bind.apply(this.curveType, [null, positions]));
        this.targetCurve = new (Function.prototype.bind.apply(this.curveType, [null, targets]));
    }

    return SmoothJourney;
})();

var JourneyWrapper = (function ($) {

    /**
     * A journey wrapper.
     * The journey wrapper uses global objects declared in the reconstruction script.
     * @constructor
     */
    function JourneyWrapper() {
        this.initialized = false;
        this.journey = null;
        this.destination = null;
        this.line = null;
        this.curveType = null;
        this.showPathController = null;

        this.graphHelper = new GraphHelper();
    }

    // Private function for calculating the desired time for moving one unit.
    var getInterval = function () {
        var interval = null;
        if (controls.animationSpeed === 0) {
            interval = Infinity;
        }
        else {
            // Calculate the time it should take to cover the distance of one unit during navigation.
            interval = (-2.4 + 1.7 / Math.sqrt(controls.animationSpeed)) * 1000 / 20;
        }

        return interval;
    }

    // Private function converting shot dictionary with rotations and translations
    // values to shot dictionary with optical centers and viewing directions.
    var convertShots = function () {
        var shots = {};
        for (var r = 0; r < reconstructions.length; ++r) {
            var newShots = reconstructions[r].shots;
            shots = $.extend(shots, newShots);
        }

        var result = {};

        for (var shot_id in shots) {
            if (!Object.prototype.hasOwnProperty.call(shots, shot_id)) {
                continue;
            }

            var shot = shots[shot_id];

            var position = opticalCenter(shot);

            var camera = getCamera(shot_id);
            var cam = camera.reconstruction['cameras'][shot['camera']];
            var target = pixelToVertex(cam, shot, 0, 0, 20);

            result[shot_id] = { 'position': position, 'target': target };
        }

        return result;
    }

    // Private function for start action of journey.
    var start = function () {
        setMovingMode('walk');
        $('#journeyButton').html('X');
    }

    // Private function for stop action of journey.
    var stop = function () {
        $('#journeyButton').html('Go');
    }

    // Private function for preloading images.
    var preload = function (shot_ids) {
        for (var i = 0; i < shot_ids.length; i++) {
            var tempImg = new Image();
            tempImg.src = imageURL(shot_ids[i]);
        }
    }

    // Private function for retrieving a camera based on the id.
    var getCamera = function (shot_id) {
        var camera = null;
        for (var i = 0; i < camera_lines.length; ++i) {
            if (camera_lines[i].shot_id === shot_id) {
                camera = camera_lines[i];
            }
        }

        return camera;
    }

    // Private function for navigation action of journey. Retrieves a camera,
    // creates its image plane and navigates to it.
    var navigation = function (shot_id) {
        var camera = getCamera(shot_id);
        if (camera === null) {
            return;
        }

        setImagePlaneCamera(camera);
        navigateToShot(camera);
    }

    // Private function for setting the position and direction of the orbit controls camera
    // used for the smooth navigation movement as well as controlling the image plane opacity.
    var smoothNavigation = function (position, target, fraction, edge) {
        controls.goto(position, target);
        options.imagePlaneOpacity = 1 - mapFraction(fraction, edge);
    }

    // Private function which retrieves a camera and creates its image plane.
    var smoothNodeAction = function (shot_id) {
        var camera = getCamera(shot_id);
        if (camera === null) {
            return;
        }

        setImagePlaneCamera(camera);
    }

    // Private function for continuing the movement to the next node when a journey is stopped.
    var smoothContinuation = function (shot_id) {
        var camera = getCamera(shot_id);
        navigateToShot(camera);
    }

    // Private function for mapping the fraction in [0, 1] to another fraction in [0, 1] based on the edge.
    var mapFraction = function (fraction, edge) {
        var length = edge.weight;
        var transitionLength = ['step_forward', 'step_backward'].indexOf(edge.direction) > -1 ? 4 : edge.weight;
        var lowerBound = Math.max((length - transitionLength) / (2 * length), 0);
        var upperBound = Math.min((length + transitionLength) / (2 * length), 1);

        var result = (fraction - lowerBound) / (upperBound - lowerBound);

        return Math.min(Math.max(result, 0), 1);
    }

    // Private function for determining the speed for the position between nodes based on the edge.
    var speedFunction = function (fraction, edge) {
        var general = 1;
        var increase = 0;

        switch (edge.direction) {
            case 'turn_left':
            case 'turn_right':
                general = edge.weight / Math.max(edge.weight, 6);
                break;
            case 'turn_u':
                general = edge.weight / Math.max(edge.weight, 8);
                break;
            case 'step_forward':
            case 'step_backward':
                // Speed increase by a maximum of 0.35 multiplied by coefficient based on edge weight.
                var k = Math.min(Math.max(edge.weight - 2, 0), 2) * 3 / 4;
                increase = k * Math.abs(0.35 - Math.min(Math.abs(fraction - 0.65), 0.35));
                break;
            default:
                break;
        }

        return general * (1 + increase);
    }

    /**
     * Initializes a journey wrapper.
     * @param {shots} Dictionary of shots with rotation and translation arrays.
     */
    JourneyWrapper.prototype.initialize = function (shots) {
        if ('nav' in urlParams && 'dest' in urlParams) {

            this.destination = urlParams.dest;
            this.curveType = THREE.SplineCurve3;
            var _this = this;

            var penalty = {
                weightKey: 'weight',
                penaltyKey: 'direction',
                values: {
                    step_backward: 30,
                    turn_u: 15,
                    turn_left: 3,
                    turn_right: 3,
                    step_left: 1,
                    step_right: 1
                }
            };

            $.getJSON(urlParams.nav, function(data) {

                var graphs = _this.graphHelper.mergeTypeGraphs(data, 'pref', 'pos', _this.destination, 10)

                _this.journey =
                    'jou' in urlParams && urlParams.jou === 'basic' ?
                        new Journey(
                            graphs,
                            convertShots(shots),
                            getInterval(),
                            navigation,
                            start,
                            stop,
                            preload,
                            penalty) :
                        new SmoothJourney(
                            graphs,
                            convertShots(shots),
                            getInterval(),
                            smoothNavigation,
                            smoothNodeAction,
                            start,
                            stop,
                            smoothContinuation,
                            preload,
                            speedFunction,
                            _this.curveType,
                            penalty);

                _this.initialized = true;

                options.showPath = false;
                options.curveType = 'Spline';
                f1.add(options, 'curveType', ['Spline', 'Linear'])
                    .onChange(function (value) {
                        var curveType;
                        switch (value) {
                            case 'Spline':
                                curveType = THREE.SplineCurve3;
                                break;
                            case 'Linear':
                                curveType = LinearCurve;
                                break;
                            default:
                                curveType = THREE.SplineCurve3;
                        }

                        _this.setCurveType(curveType);
                    });

                $('#journeyButton').show();

                if ('img' in urlParams && selectedCamera !== undefined) {
                    window.setTimeout(function () { _this.toggle(); }, 700)
                }
                else {
                    _this.addShowPathController();
                }
            });
        }
    }

    /**
     * Gets a value indicating whether a journey is ongoing.
     * @return {Boolean} A value indicating whether a journey is ongoing.
     */
    JourneyWrapper.prototype.isStarted = function () {
        if (this.initialized !== true) {
            return false;
        }

        return this.journey.isStarted();
    }

       /**
     * Gets a value indicating whether a journey type is smooth.
     * @return {Boolean} A value indicating whether a journey type is smooth.
     */
    JourneyWrapper.prototype.isSmooth = function () {
        if (this.initialized !== true) {
            return false;
        }

        return this.journey instanceof SmoothJourney
    }

    /**
     * Updates the interval.
     */
    JourneyWrapper.prototype.updateInterval = function () {
        if (this.initialized !== true) {
            return;
        }

        this.journey.updateInterval(getInterval());
    }

    /**
     * Stops a journey.
     */
    JourneyWrapper.prototype.stop = function () {
        if (this.initialized !== true) {
            return;
        }

        if (this.journey.isStarted() === true) {
            this.journey.stop(false);
        }
    }

    /**
     * Toggles the journey state between started and stopped.
     */
    JourneyWrapper.prototype.toggle = function () {
        if (this.initialized !== true) {
            return;
        }

        if (this.journey.isStarted() === true) {
            this.journey.stop(true);
            return;
        }

        if (selectedCamera === undefined) {
            return;
        }

        this.journey.updateInterval(getInterval());
        this.journey.start(selectedCamera.shot_id, this.destination);
    }

    /**
     * Shows the shortest path in the scene.
     */
    JourneyWrapper.prototype.showPath = function () {
        if (this.initialized !== true
            || selectedCamera === undefined
            || movingMode !== 'orbit'
            || options.showPath !== true){
            return;
        }

        this.hidePath();

        var pathGeometry = this.journey.getPathGeometry(selectedCamera.shot_id, this.destination);
        if (pathGeometry === null) {
            return;
        }

        var curve = new (Function.prototype.bind.apply(this.curveType, [null, pathGeometry.vertices]));
        var length = curve.getLength();
        var nbrOfPoints = length * 5;
        var curveGeometry = new THREE.Geometry();
        curveGeometry.vertices = curve.getPoints(nbrOfPoints);

        var material = new THREE.LineBasicMaterial({
            color: 0xffff88,
            linewidth: 5
        });

        this.line = new THREE.Line(curveGeometry, material);
        this.line.name = 'shortestPath'
        scene.add(this.line);
        render();
    }

    /**
     * Hides the shortest path from the scene.
     */
    JourneyWrapper.prototype.hidePath = function () {
        if (this.initialized !== true || this.line === null){
            return;
        }

        if (this.line !== null) {
            var sceneLine = scene.getObjectByName(this.line.name);
            scene.remove(sceneLine);
            this.line = null;
            render();
        }
    }

    /**
     * Sets the curve type of a journey
     * @param {Type} curveType The type of the curve used for movement. Must inherit from THREE.Curve.
     */
    JourneyWrapper.prototype.setCurveType = function (curveType) {
        if (this.initialized !== true) {
            return;
        }

        this.curveType = curveType;

        if (this.isSmooth()) {
            this.journey.setCurveType(curveType);
        }

        if (options.showPath === true) {
            this.showPath();
        }
    }

    /**
     * Adds a show path checkbox to the options.
     */
    JourneyWrapper.prototype.addShowPathController = function () {
        if (this.initialized !== true || this.showPathController !== null){
            return;
        }

        _this = this;
        this.showPathController = f1.add(options, 'showPath')
            .listen()
            .onChange(function () {
                if (options.showPath === true && selectedCamera !== undefined) {
                    _this.showPath();
                }
                else {
                    _this.hidePath();
                }
            });

        if (options.showPath === true) {
            this.showPath();
        }
    }

    /**
     * Removes the show path checkbox from the options.
     */
    JourneyWrapper.prototype.removeShowPathController = function () {
        if (this.initialized !== true || this.showPathController === null){
            return;
        }

        this.hidePath();
        f1.remove(this.showPathController);
        this.showPathController = null;
    }

    return JourneyWrapper;
})(jQuery);

var journeyWrapper = new JourneyWrapper();