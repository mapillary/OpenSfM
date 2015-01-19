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

        var path = journeyGraph.nodes.sort();

        return path;

        return journeyGraph.nodes.sort();
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

        var _this = this;

        this.path = this.shortestPath(from, to);
        if (this.path === undefined) {
            return;
        }
        this.started = true;

        this.currentIndex = 0;
        var a = this.path[this.currentIndex]

        this.initialAction('walk');
        this.navigationAction(this.path[this.currentIndex])

        this.intervalToken = window.setInterval(function () {
            return _this.onMove();
        }, this.intervalTime);
    }
    
    /**
     * Callback function for interval.
     */
    GraphHelper.prototype.onMove = function () {
        this.currentIndex++;
        if (this.started !== true || this.currentIndex >= this.path.length) {
            this.stopJourney();
            return;
        }

        this.navigationAction(this.path[this.currentIndex]);
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