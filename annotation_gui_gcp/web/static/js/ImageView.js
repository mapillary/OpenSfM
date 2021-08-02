const canvas = document.getElementById("imgCanvas");
const imageListBox = document.getElementById("imageSelectBox");
const context = canvas.getContext("2d");
const Measurements = {};
const image = new Image();
let currentPointID = null;
let currentImageKey;
let currentImageScale;

function changeImage(image_key) {
    image.onload = function () {
        resizeCanvas();
        displayImage(image_key);
        drawMeasurements();
    };
    image.src = 'image/' + image_key;
}

function displayImage(image_key) {
    currentImageKey = image_key;
    // Clear Canvas
    context.fillStyle = "#FFF";
    context.fillRect(0, 0, canvas.width, canvas.height);

    // Scale image to fit and draw
    const w = image.width;
    const h = image.height;
    if (w > h) {
        currentImageScale = canvas.width / w;
    }
    else {
        currentImageScale = canvas.height / h;
    }
    context.drawImage(image, 0, 0, w * currentImageScale, h * currentImageScale);

}

function onImageSelect() {
    const opt = imageListBox.options[imageListBox.options.selectedIndex];
    console.log("selected", opt.value);

    changeImage(opt.value);
}

function populateImageList(points) {
    const L = imageListBox.options.length - 1;
    for (let i = L; i >= 0; i--) {
        imageListBox.remove(i);
    }

    for (let image_id in points) {
        const opt = document.createElement("option");
        opt.text = image_id;
        opt.value = image_id;
        imageListBox.options.add(opt);
    }

    redrawWindow();
}

function populateMeasurements(points) {
    for (let image_id in points) {
        Measurements[image_id] = {};
        for (let point_id in points[image_id]) {
            const norm_point = points[image_id][point_id];
            const measurement = new Measurement(norm_point[0], norm_point[1], point_id);
            Measurements[image_id][point_id] = measurement;
        }
    }
    redrawWindow();
}

function initialize_event_source() {
    let sse = new EventSource("/stream");

    sse.addEventListener("sync", function (e) {
        const data = JSON.parse(e.data);
        const delay = Date.now() - Math.round(data.time * 1000);
        console.log("SSE message delay is", delay, "ms");
        populateImageList(data["points"]);
        populateMeasurements(data["points"]);
        currentPointID = data["selected_point"];
        redrawWindow();
    })

}

function initialize() {
    initialize_event_source();
    canvas.addEventListener("mousedown", mouseClicked, false);
    window.addEventListener("resize", onWindowResize);
    imageListBox.addEventListener('change', onImageSelect);
    redrawWindow();
}

function resizeCanvas() {
    context.canvas.width = window.innerWidth - imageListBox.offsetWidth - 30;
    context.canvas.height = window.innerHeight - 30;
}

function redrawWindow() {
    // box.size = box.options.length;
    resizeCanvas();
    displayImage(currentImageKey);
    drawMeasurements();
}

function onWindowResize() {
    redrawWindow();
}

class Measurement {
    constructor(x, y, id, image_id) {
        this.norm_x = x;
        this.norm_y = y;
        this.id = id;
        this.image_id = image_id;
        this.radius_px = 10;
    }
}

function drawOneMeasurement(measurement) {
    // Draw measurement
    const normalizer = Math.max(image.width, image.height);
    const x = (image.width / 2 + measurement.norm_x * normalizer) * currentImageScale;
    const y = (image.height / 2 + measurement.norm_y * normalizer) * currentImageScale;
    const radius_px = measurement.radius_px * currentImageScale;
    context.beginPath();
    context.arc(x, y, radius_px, 0, 2 * Math.PI, false);
    // context.fillStyle = 'red';
    // context.fill();
    context.lineWidth = Math.max(1, Math.min(10, radius_px / 10));
    context.strokeStyle = '#000';
    context.stroke();

    context.font = "20px Arial";
    const markerText = measurement.id;
    const textMeasurements = context.measureText(markerText);
    context.fillStyle = "#000";
    context.fillText(markerText, x + textMeasurements.width / 2, y);
}

function drawMeasurements() {
    if (!(currentImageKey in Measurements)) { return; }

    // Draw measurements
    for (const [id, measurement] of Object.entries(Measurements[currentImageKey])) {
        drawOneMeasurement(measurement);
    }
};

function post_json(data) {
    const method = 'POST';
    const headers = {
        'Accept': 'application/json',
        'Content-Type': 'application/json',
    };
    const body = JSON.stringify(data, null, 4);
    const url = 'postdata';
    fetch(url, { method, headers, body })
}

function remove_point_observation(point_id) {
    const data = {
        command: "remove_point_observation",
        point_id: "point_id",
    };
    post_json(data);
}

function add_or_update_point_observation(measurement) {
    const data = {
        command: "add_or_update_point_observation",
        point_id: measurement.id,
        radius_px: measurement.radius_px,
        xy: [measurement.norm_x, measurement.norm_y],
        image_id: measurement.image_id,
    };
    post_json(data);
}

const mouseClicked = function (mouse) {
    if (currentPointID === null) {
        console.log("No point selected, ignoring click")
        return;
    }

    // native pixel coordinates
    const rect = canvas.getBoundingClientRect();
    const normalizer = Math.max(image.width, image.height);
    const norm_x = ((mouse.x - rect.left) / currentImageScale - image.width / 2) / normalizer;
    const norm_y = ((mouse.y - rect.top) / currentImageScale - image.height / 2) / normalizer;

    const measurement = new Measurement(norm_x, norm_y, currentPointID, currentImageKey);

    // Send the clicked point to the backend. Will be draw on next sync
    add_or_update_point_observation(measurement);

}

window.addEventListener('load', initialize);