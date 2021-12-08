let canvas;
let imageListBox;
let context;
const Measurements = {};
const image = new Image();
let currentPointID = null;
let currentImageID;
let currentImageScale;
const RedirectCache = {};
let maxImageSize = 1024;
const defaultSigma = 0.004 // 70/95/99% CI = 2.6/3.8/7.6 px @ VGA resolution.
const preloadLock = new Set()
let pointColors = {};
window.addEventListener('DOMContentLoaded', onDOMLoaded);

function onDOMLoaded() {
    canvas = document.getElementById("imgCanvas");
    imageListBox = document.getElementById("imageSelectBox");
    context = canvas.getContext("2d");
    window.addEventListener('load', initialize);
}

function changeImage(image_key) {
    image.onload = function () {
        preloadNeigbors(image_key);
        resizeCanvas();
        displayImage(image_key);
        drawMeasurements();
    };
    let url = `${window.location.href}/image/${maxImageSize}/${image_key}`;
    if (url in RedirectCache){
        url = RedirectCache[url]
    }
    image.src = url;
}

function displayImage(image_key) {
    currentImageID = image_key;
    // Clear Canvas
    context.fillStyle = "#FFF";
    context.fillRect(0, 0, canvas.width, canvas.height);

    if (!currentImageID){return;}

    // Scale image to fit and draw
    const w = image.width;
    const h = image.height;
    const scale_x = canvas.width / image.width;
    const scale_y = canvas.height / image.height;
    currentImageScale = Math.min(scale_x, scale_y);
    context.drawImage(image, 0, 0, w * currentImageScale, h * currentImageScale);


    for (let i = 0; i < imageListBox.length; i++) {
        const opt = imageListBox.options[i];
        opt.style.fontWeight = (opt.value == currentImageID) ? "bold" : "normal"
    }
    imageListBox.value = currentImageID;

}

function onImageSelect() {
    const opt = imageListBox.options[imageListBox.options.selectedIndex];
    changeImage(opt.value);
}

function populateImageList(images) {
    const L = imageListBox.options.length - 1;
    for (let i = L; i >= 0; i--) {
        imageListBox.remove(i);
    }

    let index = 0;
    for (let image_id in images) {
        const opt = document.createElement("option");
        opt.text = index;
        opt.value = image_id;
        imageListBox.options.add(opt);

        if (opt.value == currentImageID) {
            opt.style.fontWeight = "bold";
            imageListBox.selectedIndex = index;;
        }
        index +=1;
    }

    if (imageListBox.selectedIndex == -1){
        currentImageID = null;
    }
    else{
        preloadNeigbors(currentImageID);
    }
    redrawWindow();
}

function preloadImage(key){
    const url = `${window.location.href}/image/${maxImageSize}/${key}`;
    if (!(url in RedirectCache) && !(preloadLock.has(url))){
        preloadLock.add(url);
        const req = new XMLHttpRequest();
        req.onload = function () {
            preloadLock.delete(url);
            RedirectCache[url] = req.responseURL;
        };
        req.open("GET", url, true);
        req.send();
    }
}

function preloadNeigbors(image_key){
    // Find the index of the current image
    let image_index;
    for (let i = 0; i < imageListBox.options.length; i++) {
        if (image_key == imageListBox.options[i].value){
            image_index = i;
            break;
        }
    }
    // Preload the neighbors
    for (let i = 0; i < imageListBox.options.length; i++) {
        const image_key = imageListBox.options[i].value;
        if (i != image_index && Math.abs(i - image_index) < 10){
            preloadImage(image_key);
        }
    }
}

function populateMeasurements(points) {
    for (let image_id in points) {
        Measurements[image_id] = {};
        for (let point_id in points[image_id]) {
            const norm_point = points[image_id][point_id];
            const measurement = new Measurement(norm_point[0], norm_point[1], point_id, image_id, norm_point[2]);
            Measurements[image_id][point_id] = measurement;
        }
    }
    redrawWindow();
}

function onSyncHandler(data) {
    pointColors = data["colors"];
    populateImageList(data["points"]);
    populateMeasurements(data["points"]);
    currentPointID = data["selected_point"];
    redrawWindow();
}

function initialize() {
    const sse = initialize_event_source([
        { event: "sync", handler: onSyncHandler }
    ]);
    window.onunload = () => {
        sse.close();
    }

    canvas.addEventListener("mousedown", mouseClicked, false);
    canvas.addEventListener("mousewheel", mouseWheelTurned, false);
    window.addEventListener("resize", onWindowResize);
    imageListBox.addEventListener('change', onImageSelect);
    post_json({ event: "init" });
}

function resizeCanvas() {
    context.canvas.width = window.innerWidth - imageListBox.offsetWidth - 30;
    context.canvas.height = window.innerHeight - 30;
}

function redrawWindow() {
    // box.size = box.options.length;
    resizeCanvas();
    displayImage(currentImageID);
    drawMeasurements();
}

function onWindowResize() {
    redrawWindow();
}

class Measurement {
    constructor(norm_x, norm_y, id, image_id, norm_precision) {
        this.norm_x = norm_x; // x, y in normalized pixels
        this.norm_y = norm_y;
        this.id = id;
        this.image_id = image_id;
        this.norm_precision = norm_precision; // std. deviation in pixels / max(w,h)
    }
}

function drawCircle(x, y, radius_px, color){
    context.beginPath();
    context.arc(x, y, radius_px, 0, 2 * Math.PI, false);
    // context.fillStyle = 'red';
    // context.fill();
    context.lineWidth = Math.max(1, Math.min(10, radius_px / 10));
    context.strokeStyle = color;
    context.stroke();
}

function drawOneMeasurement(measurement) {
    // Draw measurement
    const color = pointColors[measurement.id];
    const normalizer = Math.max(image.width, image.height);
    const x = (image.width / 2 + measurement.norm_x * normalizer) * currentImageScale;
    const y = (image.height / 2 + measurement.norm_y * normalizer) * currentImageScale;
    const radius_px = measurement.norm_precision * normalizer * currentImageScale;

    // drawCircle(x, y, radius_px, color); // 70% confidence interval
    drawCircle(x, y, radius_px * 2, color); // 95% confidence interval
    drawCircle(x, y, radius_px * 3, color); // 99% confidence interval

    context.font = "20px Arial";
    const markerText = measurement.id;
    const textMeasurements = context.measureText(markerText);
    context.fillStyle = "#000";
    context.fillText(markerText, x + textMeasurements.width / 2, y);
}

function drawMeasurements() {
    if (!(currentImageID in Measurements)) { return; }

    // Draw measurements
    for (const [id, measurement] of Object.entries(Measurements[currentImageID])) {
        drawOneMeasurement(measurement);
    }
};

function remove_point_observation(image_id, point_id) {
    const data = {
        event: "remove_point_observation",
        image_id: image_id,
        point_id: point_id,
    };
    post_json(data);
}

function add_or_update_point_observation(measurement) {
    const data = {
        event: "add_or_update_point_observation",
        point_id: measurement.id,
        norm_precision: measurement.norm_precision,
        xy: [measurement.norm_x, measurement.norm_y],
        image_id: measurement.image_id,
    };
    post_json(data);
}

const mouseWheelTurned = function (wheel) {
    // Find index of selected image
    let selected_i = null;
    for (let i = 0; i < imageListBox.options.length; i++) {
        const opt = imageListBox.options[i];
        if (currentImageID == opt.value) {
            selected_i = i;
            break;
        }
    }

    selected_i = (wheel.deltaY > 0) ? selected_i - 1 : selected_i + 1

    if (selected_i >= 0 && selected_i < imageListBox.options.length) {
        changeImage(imageListBox.options[selected_i].value);
    }
}

const mouseClicked = function (mouse) {
    if (currentPointID === null) {
        console.log("No point selected, ignoring click")
        return;
    }

    if (mouse.button == 0) {
        // native pixel coordinates
        const rect = canvas.getBoundingClientRect();
        const normalizer = Math.max(image.width, image.height);
        const norm_x = ((mouse.x - rect.left) / currentImageScale - image.width / 2) / normalizer;
        const norm_y = ((mouse.y - rect.top) / currentImageScale - image.height / 2) / normalizer;

        const measurement = new Measurement(norm_x, norm_y, currentPointID, currentImageID, defaultSigma);

        // Send the clicked point to the backend. Will be draw on next sync
        add_or_update_point_observation(measurement);
    }
    else {
        remove_point_observation(currentImageID, currentPointID);
    }
}
