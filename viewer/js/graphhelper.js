var Dijkstra = (function () {

     /**
     * A class for calculations on graphs using Dijkstra's algorithm.
     * @constructor
     */
    function Dijkstra(graph) {
        this.graph = graph;
    }

    // Private sort delegate for ordering key value pairs arranged
    // as an array of two items like [key, value].
    var keyValueSorter = function(kv1, kv2) {
        return parseFloat(kv1[1]) - parseFloat(kv2[1]);
    }

     /**
     * Calculate the shortest path between two nodes in a graph using
     * Dijkstra's Algorithm.
     * @param {String} source
     * @param {String} target
     * @return {Array} An array of node names corresponding to the path
     */
    Dijkstra.prototype.shortestPath = function (source, target, weight) {
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

        while (touchedNodes) {

            var touched = [];
            for (var key in touchedNodes) {
                if (Object.prototype.hasOwnProperty.call(touchedNodes, key)) {
                    touched.push([key, touchedNodes[key]])
                }
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

            var currentEdges = this.graph.edges[currentNode] || {};

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

var GraphHelper = (function () {

    /**
     * A graph helper.
     * @constructor
     */
    function GraphHelper(data, navigationAction, initialAction, intervalTime) {
        this.graphs = data;
        this.navigationAction = navigationAction;
        this.initialAction = initialAction;
        this.intervalTime = intervalTime;
        this.intervalToken = undefined;
        this.path = undefined;
        this.currentIndex = 0;
        this.started = false;
    }

    // Private callback function for setInterval.
    var onMove = function(self) {
        self.currentIndex++;
        if (self.started !== true || self.currentIndex >= self.path.length) {
            self.stopJourney();
            return;
        }

        self.navigationAction(self.path[self.currentIndex]);
    }

    /**
     * Sets the interval time.
     */
    GraphHelper.prototype.setIntervalTime = function (intervalTime) {
        this.intervalTime = intervalTime;
    }

     /**
     * Gets a value indicating whether a journey is ongoing.
     * @return {Boolean} A value indicating whether a journey is ongoing.
     */
    GraphHelper.prototype.getIsStarted = function() {
        return this.started;
    }

    /**
     * Calculate the shortest path between two nodes in a graph.
     * @param {String} from
     * @param {String} to
     * @return {Array} An array of node names corresponding to the path
     */
    GraphHelper.prototype.shortestPath = function (from, to) {
        var journeyGraph = undefined;
        for (var k in this.graphs) {
            if (this.graphs.hasOwnProperty(k)) {
                // Ensure that both nodes exist in the graph.
                if (this.graphs[k].nodes.indexOf(from) > -1 &&
                    this.graphs[k].nodes.indexOf(to) > -1) {
                    journeyGraph = this.graphs[k];
                    break;
                }
            }
        }

        if (journeyGraph === undefined) {
            return undefined;
        }

        var dijkstra = new Dijkstra(journeyGraph);
        var path = dijkstra.shortestPath(from, to, 'weight');

        return path;
    }

    /**
     * Starts a journey between two nodes in a graph.
     * @param {Number} from
     * @param {Number} to
     */
    GraphHelper.prototype.startJourney = function (from, to) {
        if (this.started === true) {
            return;
        }

        this.path = this.shortestPath(from, to);
        if (this.path === undefined) {
            return;
        }

        this.started = true;
        this.currentIndex = 0;
        this.initialAction('walk');
        this.navigationAction(this.path[this.currentIndex])

        var _this = this;
        this.intervalToken = window.setInterval(function () { onMove(_this); }, this.intervalTime);
    }

    /**
     * Stops an ongoing journey between two nodes in a graph.
     */
    GraphHelper.prototype.stopJourney = function () {
        if (this.intervalToken === undefined) {
            return;
        }

        window.clearInterval(this.intervalToken);
        this.intervalToken = undefined;
        this.currentIndex = 0;
        this.path = undefined;

        this.started = false;
    }

    return GraphHelper;
})();