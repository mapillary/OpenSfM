var Dijkstra = (function () {

    function Dijkstra(graph) {
        this.graph = graph;
    }

    var keyValueSorter = function(t1, t2) {
        return parseFloat(t1[1]) - parseFloat(t2[1]);
    }

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
            var touchedDists = [];
            for (var key in touchedNodes) {
                if (Object.prototype.hasOwnProperty.call(touchedNodes, key)) {
                    touchedDists.push([ key, touchedNodes[key]])
                }
            }

            touchedDists.sort(keyValueSorter);

            var shortestDist = touchedDists[0];
            var previousNode = shortestDist[0];
            var dist = shortestDist[1];

            visited[previousNode] = true;
            delete touchedNodes[previousNode];

            if (previousNode === target) {
                break;
            }

            var nodeEdges = this.graph.edges[previousNode] || {};

            for (var node in nodeEdges) {
                if (Object.prototype.hasOwnProperty.call(nodeEdges, node)) {
                    var edge = nodeEdges[node];

                    if (Object.prototype.hasOwnProperty.call(visited, node)) {
                        continue;
                    }

                    var touchedNode = node;
                    var distance = edge[weight];

                    var totalDistance = distances[previousNode] + distance;

                    if (!distances[touchedNode] || totalDistance < distances[touchedNode])
                    {
                        distances[touchedNode] = totalDistance;
                        previous[touchedNode] = previousNode;
                        touchedNodes[touchedNode] = totalDistance;
                    }
                }
            }
        }

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
     * Gets a value indicating whether a journey is started.
     */
    GraphHelper.prototype.getIsStarted = function() {
        return this.started;
    }

    /**
     * Calculate the shortest path between two nodes in a graph.
     * @param {Number} from
     * @param {Number} to
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