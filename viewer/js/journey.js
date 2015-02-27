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

            visited[currentNode] = true;
            delete touchedNodes[currentNode];

            // Return if we have reached the target.
            if (currentNode === target) {
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

        // No path to the target was found.
        if (previous[target] === undefined) {
            return null;
        }

        // Retrieve a path from the dictionary of previous nodes and reverse it.
        var reversePath = [];
        var element = target;
        while (element !== undefined) {
            reversePath.push(element);
            element = previous[element];
        }

        return reversePath.reverse();
    }

    return Dijkstra;
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
     * @param {Boolean} usePenalty Value indicating if a penalty should be used.
     */
    function JourneyBase(graphs, shots, intervalTime, usePenalty) {
        this.graphs = graphs;
        this.shots = shots;
        this.intervalTime = intervalTime;
        this.usePenalty = usePenalty;

        this.started = false;
        this.dijkstra = new Dijkstra();
    }

    // Private function for creating a graph with a penalty for a certain property with
    // a certain value.
    var getPenaltyGraph = function (graph, weightKey, penaltyKey, penalties) {

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
        var index = undefined;
        for (var i = 0; i < this.graphs.length; i++) {
            // Ensure that both nodes exist in the graph.
            if (this.graphs[i].nodes.indexOf(from) > -1 &&
                this.graphs[i].nodes.indexOf(to) > -1) {
                index = i;
                break;
            }
        }

        if (index === undefined) {
            return null;
        }

        var journeyGraph = this.graphs[index];
        if (this.usePenalty === true) {
            journeyGraph =
                getPenaltyGraph(
                    journeyGraph,
                    'weight',
                    'direction',
                    { step_backward: 30, turn_u: 15 });
        }

        var path = this.dijkstra.shortestPath(journeyGraph, from, to, 'weight');

        return { path: path, index: index };
    }

    /**
     * Creates a geometry based on an existing property of the nodes in the path.
     * @param {Array} path Nodes to create geometry from.
     * @param {String} property Name of the shot property to use.
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
     */
    JourneyBase.prototype.getPathGeometry = function (from, to) {

        var result = this.shortestPath(from, to);
        if (result === null || result.path.length <= 1) {
            return null;
        }

        return this.getGeometry(result.path, 'position');
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
     * @param {Boolean} usePenalty Value indicating if a penalty should be used.
     */
    function Journey(
        graphs,
        shots,
        intervalTime,
        navigationAction,
        startAction,
        stopAction,
        preloadAction,
        usePenalty) {

        JourneyBase.apply(this, [graphs, shots, intervalTime, usePenalty]);

        this.navigationAction = navigationAction;
        this.startAction = startAction;
        this.stopAction = stopAction;
        this.preloadAction = preloadAction;
        this.timeoutToken = undefined;
        this.path = undefined;
        this.graphIndex = undefined;
        this.currentIndex = 0;
    }

    // Inheriting from JourneyBase
    Journey.prototype = Object.create(JourneyBase.prototype);
    Journey.prototype.constructor = Journey;

    // Private function for calculating the interval value.The max distance of an edge is
    // 20. The interval is the fraction of the max distance multiplied by the current
    // interval time. A smallest value is defined to avoid too fast navigation..
    var getInterval = function (edges, node, intervalTime) {
        var distance = edges[node].weight;
        return Math.max((distance / 20) * intervalTime, 0.7 * 1000);
    }

    // Private callback function for setInterval.
    var onNavigation = function (self) {
        var pathLength = self.path.length;
        self.currentIndex++;

        if (self.started !== true || self.currentIndex >= pathLength) {
            self.stop();
            return;
        }

        if (!isFinite(self.intervalTime)) {
            self.timeoutToken = window.setTimeout(function () { onNavigation(self); }, 1000);
            return;
        }

        self.navigationAction(self.path[self.currentIndex]);

        if (self.currentIndex === pathLength - 1) {
            self.stop();
            return;
        }

        if (self.currentIndex + 10 <= pathLength - 1) {
            self.preloadAction([self.path[self.currentIndex + 10]]);
        }

        var currentInterval =
            getInterval(
                self.graphs[self.graphIndex].edges[self.path[self.currentIndex - 1]],
                self.path[self.currentIndex],
                self.intervalTime);

        self.timeoutToken = window.setTimeout(function () { onNavigation(self); }, currentInterval);
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
        this.graphIndex = result.index;
        this.currentIndex = 0;
        this.startAction();
        this.navigationAction(this.path[this.currentIndex])
        this.preloadAction(this.path.slice(1, Math.min(10, this.path.length)))

        var currentInterval = isFinite(this.intervalTime) ?
            getInterval(
                this.graphs[this.graphIndex].edges[this.path[this.currentIndex]],
                this.path[this.currentIndex + 1],
                this.intervalTime) :
            1000;

        var _this = this;
        this.timeoutToken = window.setTimeout(function () { onNavigation(_this); }, currentInterval);
    }

    /**
     * Stops an ongoing journey between two nodes in a graph.
     */
    Journey.prototype.stop = function (continuation) {
        if (this.timeoutToken === undefined || this.started === false) {
            return;
        }

        window.clearTimeout(this.timeoutToken);
        this.timeoutToken = undefined;
        this.currentIndex = 0;
        this.path = undefined;
        this.graphIndex = undefined;

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
     * @param {Boolean} usePenalty Value indicating if a penalty should be used.
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
        usePenalty) {

        JourneyBase.apply(this, [graphs, shots, intervalTime, usePenalty]);

        this.navigationAction = navigationAction;
        this.nodeAction = nodeAction;
        this.startAction = startAction;
        this.stopAction = stopAction;
        this.continuationAction = continuationAction;
        this.preloadAction = preloadAction;

        this.previousTime = undefined;
        this.currentIndex = 0;
        this.u = 0;
        this.path = undefined;
        this.linearCurve = undefined;
        this.intervalToken = undefined;
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

        if (this.currentIndex + 10 <= this.path.length - 1) {
            this.preloadAction([this.path[this.currentIndex + 10]]);
        }

        var elapsed = currentTime - this.previousTime;
        this.previousTime = currentTime;
        var totalTime = this.intervalTime * this.linearCurve.getLength() / 15;

        this.u = Math.min(this.u + (elapsed / totalTime), 1);

        // Retrieve t from the linear curve to calculate weight and index.
        var t = this.linearCurve.getUtoTmapping(this.u);
        var point = (this.path.length - 1) * t;
        var intPoint = Math.floor(point);
        var weight = point - intPoint;

        if (intPoint > this.currentIndex && intPoint < this.path.length) {
            this.currentIndex = intPoint;
            this.nodeAction(this.path[this.currentIndex + 1]);
        }

        var position = this.linearCurve.getPoint(t);

        // Calculate the target by linear interpolation between shot targets.
        var shot_id1 = this.path[intPoint];
        var vd1 = this.shots[shot_id1]['target'];

        var shot_id2 = this.path[Math.min(intPoint + 1, this.path.length - 1)];
        var vd2 = this.shots[shot_id2]['target'];

        var target = new THREE.Vector3().copy(vd1).lerp(vd2, weight);

        this.navigationAction(position, target);

        if (this.u >= 1) {
            this.stop();
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
        this.linearCurve = new LinearCurve(this.getGeometry(this.path, 'position').vertices);

        var position = this.linearCurve.getPointAt(0);
        var shot_id = this.path[0];
        var target = this.shots[shot_id]['target'];

        this.previousTime = Date.now();
        this.u = 0;
        this.currentIndex = 0;

        this.startAction();
        this.preloadAction(this.path.slice(1, Math.min(10, this.path.length)));
        this.nodeAction(this.path[this.currentIndex + 1]);
        this.navigationAction(position, target);

        _this = this;
        this.intervalToken = window.setInterval(function () { move.call(_this); }, 1000/60);
    }

    /**
     * Stops a smooth journey.
     * @param {Boolean} continuation Specifying if the continuation action should be invoked.
     */
    SmoothJourney.prototype.stop = function (continuation) {
        if (this.intervalToken === undefined || this.started === false) {
            return;
        }

        window.clearInterval(this.intervalToken);
        this.intervalToken = undefined;

        var nextIndex = Math.min(this.currentIndex + 1, this.path.length - 1);
        var nextNode = this.path[nextIndex];

        this.path = undefined;
        this.linearCurve = undefined;
        this.previousTime = undefined;
        this.currentIndex = 0;
        this.u = 0;

        if (continuation === true) {
            this.continuationAction(nextNode);
        }

        this.stopAction();

        this.started = false;
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
        this.journey = undefined;
        this.destination = undefined;
        this.line = undefined;
    }

    // Private function for calculating the desired maximum interval.
    var getInterval = function () {
        var interval = undefined;
        if (controls.animationSpeed === 0) {
            interval = Infinity;
        }
        else {
            interval = (4 - 15 * controls.animationSpeed) * 1000;
        }

        return interval;
    }

    // Private function for retrieving a camera based on the id.
    var getCamera = function (shot_id) {
        var camera = undefined;
        for (var i = 0; i < camera_lines.length; ++i) {
            if (camera_lines[i].shot_id === shot_id) {
                camera = camera_lines[i];
            }
        }

        return camera === undefined ? null : camera;
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

    // Private function which retrieves a camera and
    // creates its image plane.
    var setImagePlane = function (shot_id) {
        var camera = getCamera(shot_id);
        if (camera === null) {
            return;
        }

        setImagePlaneCamera(camera);
    }

    // Private function for preloading images.
    var preload = function (shot_ids) {
        for (var i = 0; i < shot_ids.length; i++) {
            var tempImg = new Image();
            tempImg.src = imageURL(shot_ids[i]);
        }
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

    // Private function converting shot dictionary with rotations and translations
    // values to shot dictionary with optical centers and viewing directions.
    var convertShots = function (shots) {
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

    // Private function for setting the position and direction of the orbit controls camera
    // used for the smooth navigation movement.
    var smoothNavigation = function (position, target) {
        controls.goto(position, target);
    }

    // Private function for continuing the movement to the next node when a journey is stopped.
    var continuation = function (shot_id) {
        var camera = getCamera(shot_id);
        navigateToShot(camera);
    }

    /**
     * Initializes a journey wrapper.
     * @param {shots} Dictionary of shots with rotation and translation arrays.
     */
    JourneyWrapper.prototype.initialize = function (shots) {
        if ('nav' in urlParams && 'dest' in urlParams) {

            this.destination = urlParams.dest;
            var _this = this;

            $.getJSON(urlParams.nav, function(data) {

                _this.journey =
                    'jou' in urlParams && urlParams.jou === 'basic' ?
                        new Journey(
                            data,
                            convertShots(shots),
                            getInterval(),
                            navigation,
                            start,
                            stop,
                            preload,
                            true) :
                        new SmoothJourney(
                            data,
                            convertShots(shots),
                            getInterval(),
                            smoothNavigation,
                            setImagePlane,
                            start,
                            stop,
                            continuation,
                            preload,
                            true);

                _this.initialized = true;

                options.showPath = false;
                $('#journeyButton').show();

                if ('img' in urlParams && selectedCamera !== undefined) {
                    _this.toggle();
                }
                else {
                    _this.addShowPathController();
                }
            });
        }
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
        if (this.initialized !== true || selectedCamera === undefined){
            return;
        }

        this.hidePath();

        var geometry = this.journey.getPathGeometry(selectedCamera.shot_id, this.destination);
        if (geometry === null) {
            return;
        }

        var material = new THREE.LineBasicMaterial({
            color: 0xffff88,
            linewidth: 5
        });

        this.line = new THREE.Line(geometry, material);
        this.line.name = 'shortestPath'
        scene.add(this.line);
        render();
    }

    /**
     * Hides the shortest path from the scene.
     */
    JourneyWrapper.prototype.hidePath = function () {
        if (this.initialized !== true || this.line === undefined){
            return;
        }

        if (this.line !== undefined) {
            var sceneLine = scene.getObjectByName(this.line.name);
            scene.remove(sceneLine);
            this.line = undefined;
            render();
        }
    }

    /**
     * Adds a show path checkbox to the options.
     */
    JourneyWrapper.prototype.addShowPathController = function () {
        if (this.initialized !== true || this.showPathController !== undefined){
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
        if (this.initialized !== true || this.showPathController === undefined){
            return;
        }

        this.hidePath();
        f1.remove(this.showPathController);
        this.showPathController = undefined;
    }

    return JourneyWrapper;
})(jQuery);

var journeyWrapper = new JourneyWrapper();

