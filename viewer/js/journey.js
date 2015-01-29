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
     * @param {Object} graph
     * @param {String} source
     * @param {String} target
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

var Journey = (function () {

    /**
     * A journey.
     * @constructor
     * @param {String} graphs A list of graphs.
     * @param {String} intervalTime The maximum time between navigation.
     * @param {String} navigationAction The action to execute on navigation.
     * @param {String} startAction The action to run when starting a journey.
     * @param {String} stopAction The action to run when stopping a journey.
     * @param {String} usePenalty Boolean indicating if a penalty should be used.
     */
    function Journey(graphs, intervalTime, navigationAction, startAction, stopAction, usePenalty) {
        this.graphs = graphs;
        this.intervalTime = intervalTime;
        this.navigationAction = navigationAction;
        this.startAction = startAction;
        this.stopAction = stopAction;
        this.usePenalty = usePenalty;
        this.timeoutToken = undefined;
        this.path = undefined;
        this.graphIndex = undefined;
        this.currentIndex = 0;
        this.started = false;
        this.dijkstra = new Dijkstra();
    }

    // Private function for calculating the interval value.The max distance of an edge is
    // 20. The interval is the fraction of the max distance multiplied by the current
    // interval time. A smallest value is defined to avoid too fast navigation..
    var getInterval = function (edges, node, intervalTime) {
        var distance = edges[node].weight;
        return Math.max((distance / 20) * intervalTime, 0.5 * 1000);
    }

    // Private callback function for setInterval.
    var onNavigation = function (self) {
        var pathLength = self.path.length;
        self.currentIndex++;

        if (self.started !== true || self.currentIndex >= pathLength) {
            self.stop();
            return;
        }

        self.navigationAction(self.path[self.currentIndex]);

        if (self.currentIndex === pathLength - 1) {
            self.stop();
            return;
        }

        var currentInterval =
            getInterval(
                self.graphs[self.graphIndex].edges[self.path[self.currentIndex]],
                self.path[self.currentIndex + 1],
                self.intervalTime);

        self.timeoutToken = window.setTimeout(function () { onNavigation(self); }, currentInterval);
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
    Journey.prototype.updateInterval = function (intervalTime) {
        this.intervalTime = intervalTime;
    }

     /**
     * Gets a value indicating whether a journey is ongoing.
     * @return {Boolean} A value indicating whether a journey is ongoing.
     */
    Journey.prototype.isStarted = function () {
        return this.started;
    }

    /**
     * Calculate the shortest path between two nodes in a graph.
     * @param {String} from
     * @param {String} to
     * @return {Array} An array of node names corresponding to the path
     */
    Journey.prototype.shortestPath = function (from, to) {
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

        var currentInterval =
            getInterval(
                this.graphs[this.graphIndex].edges[this.path[this.currentIndex]],
                this.path[this.currentIndex + 1],
                this.intervalTime);

        var _this = this;
        this.timeoutToken = window.setTimeout(function () { onNavigation(_this); }, currentInterval);
    }

    /**
     * Stops an ongoing journey between two nodes in a graph.
     */
    Journey.prototype.stop = function () {
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

var JourneyWrapper = (function ($) {

    /**
     * A journey wrapper.
     * @constructor
     */
    function JourneyWrapper() {
        this.initialized = false;
        this.journey = undefined;
        this.destination = undefined;
    }

    var getInterval = function () {
        var interval = undefined;
        if (controls.animationSpeed === 0) {
            interval = 3 * 1000;
        }
        else {
            interval = (3 - 10 * (controls.animationSpeed)) * 1000;
        }

        return interval;
    }

    var navigation = function (shot_id) {
        var camera = undefined;
        for (var i = 0; i < camera_lines.length; ++i) {
            if (camera_lines[i].shot_id === shot_id) {
                camera = camera_lines[i];
            }
        }

        if (camera === undefined) {
            return;
        }

        setImagePlaneCamera(camera);
        navigateToShot(camera);
    }

    var start = function () {
        setMovingMode('walk');
        $('#journeyButton').html('X');
    }

    var stop = function () {
        $('#journeyButton').html('Go');
    }

    /**
     * Initializes a journey wrapper.
     */
    JourneyWrapper.prototype.initialize = function () {
        if ('graph' in urlParams && 'dest' in urlParams) {

            this.destination = urlParams.dest;
            var _this = this;

            $.getJSON(urlParams.graph, function(data) {

                _this.journey =
                    new Journey(
                        data,
                        getInterval(),
                        navigation,
                        start,
                        stop,
                        true);

                _this.initialized = true;
                $('#journeyButton').show();

                if ('img' in urlParams && selectedCamera !== undefined) {
                    _this.toggleJourney();
                }
            });
        }
    }

    /**
     * Updates the interval.
     */
    JourneyWrapper.prototype.updateInterval = function () {
        if (this.initialized !== true){
            return;
        }

        this.journey.updateInterval(getInterval());
    }

    /**
     * Stops a journey.
     */
    JourneyWrapper.prototype.stop = function () {
        if (this.initialized !== true){
            return;
        }

        if (this.journey.isStarted() === true) {
            this.journey.stop();
        }
    }

    /**
     * Toggles the journey state between started and stopped.
     */
    JourneyWrapper.prototype.toggleJourney = function () {
        if (this.initialized !== true){
            return;
        }

        if (this.journey.isStarted() === true) {
            this.journey.stop();
            return;
        }

        if (selectedCamera === undefined) {
            return;
        }

        this.journey.updateInterval(getInterval());
        this.journey.start(selectedCamera.shot_id, this.destination);
    }

    return JourneyWrapper;
})(jQuery);

var journeyWrapper = new JourneyWrapper();

