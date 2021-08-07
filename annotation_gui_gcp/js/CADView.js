import * as THREE from 'https://unpkg.com/three@0.127.0/build/three.module.js';
import { FBXLoader } from 'https://unpkg.com/three@0.127.0/examples/jsm/loaders/FBXLoader.js';
import { OrbitControls } from 'https://unpkg.com/three@0.127.0/examples/jsm/controls/OrbitControls.js';

//----Variables----//
//DOM element to attach the renderer to
let viewport;

//built-in three.js _cameraControls will be attached to this
let _cameraControls;

//camera attributes
const view_angle = 45;

//----Constructors----//
const renderer = new THREE.WebGLRenderer({ antialias: true });
const _scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(view_angle, 1);

//constructs an instance of a white light
renderer.setClearColor(0x05CB63); // Background color
const pointLight = new THREE.PointLight(0xFFFFFF);
const ambientLight = new THREE.AmbientLight(0x404040); // soft white light

let _raycaster;

// List of three.js 3D objects depicting annotated GCPs
let _gcps = {};

// cad model
let _cad_model = null;
let _cad_model_bbox = null;
let _cad_model_filename = null;

// Marker that points at the tracked camera
let _trackingMarker = null;

window.addEventListener('load', initialize);

function getCompoundBoundingBox(object3D) {
    let box = null;
    object3D.traverse(function (obj3D) {
        let geometry = obj3D.geometry;
        if (geometry === undefined) return;
        geometry.computeBoundingBox();
        if (box === null) {
            box = geometry.boundingBox;
        } else {
            box.union(geometry.boundingBox);
        }
    });
    return box;
}

function onWindowResize() {
    resizeCanvas()
}

function resizeCanvas() {
    const w = window.innerWidth;
    const h = window.innerHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
}

function fitCameraToSelection(camera, controls, selection, fitOffset = 1.2) {

    const box = new THREE.Box3();

    for (const object of selection) box.expandByObject(object);

    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const maxSize = Math.max(size.x, size.y, size.z);
    const fitHeightDistance = maxSize / (2 * Math.atan(Math.PI * camera.fov / 360));
    const fitWidthDistance = fitHeightDistance / camera.aspect;
    const distance = fitOffset * Math.max(fitHeightDistance, fitWidthDistance);

    const direction = controls.target.clone()
        .sub(camera.position)
        .normalize()
        .multiplyScalar(distance);

    controls.maxDistance = distance * 10;
    controls.target.copy(center);
    camera.near = distance / 100;
    camera.far = distance * 100;
    camera.updateProjectionMatrix();
    camera.position.copy(controls.target).sub(direction);

    controls.update();

}


function load_cad_model(path_model) {
    console.log("Loading CAD model " + path_model)
    const loader = new FBXLoader();
    loader.load(path_model, function (object) {
        console.log("Loaded CAD model " + path_model)
        _cad_model = object;
        _scene.add(object);

        // Set the camera position to center of bbox
        _cad_model_bbox = getCompoundBoundingBox(object)
        let cameraTarget = new THREE.Vector3();
        _cad_model_bbox.getCenter(cameraTarget);
        object.localToWorld(cameraTarget);

        _cameraControls.target = cameraTarget;
        fitCameraToSelection(camera, _cameraControls, object.children)
    });
}



function setup_scene() {
    //Sets up the renderer to the same size as a DOM element
    //and attaches it to that element
    viewport.appendChild(renderer.domElement);


    _cameraControls = new OrbitControls(camera, renderer.domElement);
    _cameraControls.movementSpeed = 1;
    _cameraControls.domElement = viewport;


    _raycaster = new THREE.Raycaster();
    _raycaster.params.Points.threshold = 0.1;

    camera.position.set(10, 10, 10);
    camera.lookAt(new THREE.Vector3(0, 0, 0));
    pointLight.position.set(10, 50, 150);
    _scene.add(camera);
    _scene.add(pointLight);
    _scene.add(ambientLight);

    initializeTrackingMarker();
    resizeCanvas()
}


function makeTextSprite(message, parameters) {
    if (parameters === undefined) parameters = {};

    let fontface = parameters.hasOwnProperty("fontface") ?
        parameters["fontface"] : "Arial";

    let fontsize = parameters.hasOwnProperty("fontsize") ?
        parameters["fontsize"] : 18;

    let borderThickness = parameters.hasOwnProperty("borderThickness") ?
        parameters["borderThickness"] : 4;

    let borderColor = parameters.hasOwnProperty("borderColor") ?
        parameters["borderColor"] : { r: 0, g: 0, b: 0, a: 1.0 };

    let backgroundColor = parameters.hasOwnProperty("backgroundColor") ?
        parameters["backgroundColor"] : { r: 255, g: 255, b: 255, a: 1.0 };

    let canvas = document.createElement('canvas');
    let context = canvas.getContext('2d');
    context.font = "Bold " + fontsize + "px " + fontface;

    // get size data (height depends only on font size)
    let metrics = context.measureText(message);
    let textWidth = metrics.width;

    // background color
    context.fillStyle = "rgba(" + backgroundColor.r + "," + backgroundColor.g + ","
        + backgroundColor.b + "," + backgroundColor.a + ")";
    // border color
    context.strokeStyle = "rgba(" + borderColor.r + "," + borderColor.g + ","
        + borderColor.b + "," + borderColor.a + ")";

    context.lineWidth = borderThickness;
    roundRect(context, borderThickness / 2, borderThickness / 2, textWidth + borderThickness, fontsize * 1.4 + borderThickness, 6);
    // 1.4 is extra height factor for text below baseline: g,j,p,q.

    // text color
    context.fillStyle = "rgba(0, 0, 0, 1.0)";

    context.fillText(message, borderThickness, fontsize + borderThickness);

    // canvas contents will be used for a texture
    let texture = new THREE.Texture(canvas)
    texture.needsUpdate = true;

    let spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    let sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(500, 250, 1.0);
    return sprite;
}

// function for drawing rounded rectangles
function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r);
    ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    ctx.lineTo(x + r, y + h);
    ctx.quadraticCurveTo(x, y + h, x, y + h - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
}

function initializeTrackingMarker() {
    const sphereGeometry = new THREE.SphereGeometry(150);
    _trackingMarker = new THREE.Mesh(sphereGeometry);
    _trackingMarker.material.color = { 'r': 1.0, 'g': 0, 'b': 0 };
    _scene.add(_trackingMarker);
}

function updateTrackingMarker(new_position) {
    _trackingMarker.position.copy(new_position);
}

function updateGCPLabels() {
    for (var gcp_id in _gcps) {
        const sphere = _gcps[gcp_id]["marker"];
        const sprite = _gcps[gcp_id]["label"]
        const director_vector = new THREE.Vector3();
        director_vector.subVectors(camera.position, sphere.position);
        const cam_to_gcp_distance = director_vector.length();
        sprite.scale.set(cam_to_gcp_distance / 5, cam_to_gcp_distance / 10, 1.0);
        sprite.position.addVectors(sphere.position, director_vector.setLength(300.0));
        sprite.center.set(0, 1)
    }
}

function create_or_update_gcp(gcp_id, xyz, reprojection_xyz, color) {
    const gcp_position = new THREE.Vector3(xyz[0], xyz[1], xyz[2]);
    const gcp_reprojection = reprojection_xyz ? new THREE.Vector3(reprojection_xyz[0], reprojection_xyz[1], reprojection_xyz[2]) : gcp_position;
    let sphere;
    let sprite;
    let line;
    // check if the property/key is defined in the object itself, not in parent
    if (!_gcps.hasOwnProperty(gcp_id)) {

        // A sphere marks the location of the annotation
        const sphereGeometry = new THREE.SphereGeometry(50);
        sphere = new THREE.Mesh(sphereGeometry);
        _scene.add(sphere);

        // A label
        sprite = makeTextSprite(gcp_id, { fontsize: 32, fontface: "Georgia" });
        _scene.add(sprite);

        // A line from the annotation position to its reprojection
        const line_material = new THREE.LineBasicMaterial();
        const geometry = new THREE.BufferGeometry();
        line = new THREE.Line(geometry, line_material);
        _scene.add(line);

        _gcps[gcp_id] = { "marker": sphere, "label": sprite, "reprojectionLine": line };
    }
    else {
        sphere = _gcps[gcp_id]["marker"];
        sprite = _gcps[gcp_id]["label"]
        line = _gcps[gcp_id]["reprojectionLine"]
    }
    sphere.position.copy(gcp_position);
    sphere.material.color = { 'r': color[0] / 255.0, 'g': color[1] / 255.0, 'b': color[2] / 255.0 };
    sphere.material.needsupdate = true;
    line.geometry.setFromPoints([gcp_position, gcp_reprojection]);
    line.material.color = { 'r': color[0] / 255.0, 'g': color[1] / 255.0, 'b': color[2] / 255.0 };
}

function update_gcps(annotations) {
    for (var gcp_id in _gcps) {
        if (!annotations.hasOwnProperty(gcp_id)) {
            _scene.remove(_gcps[gcp_id]["marker"])
            _scene.remove(_gcps[gcp_id]["label"])
            _scene.remove(_gcps[gcp_id]["reprojectionLine"])
            delete _gcps[gcp_id];
        }
    }
    for (var gcp_id in annotations) {
        create_or_update_gcp(
            gcp_id,
            annotations[gcp_id]["coordinates"],
            annotations[gcp_id].hasOwnProperty("reprojection") ? annotations[gcp_id]["reprojection"] : null,
            annotations[gcp_id]["color"],
        );
    }
    updateGCPLabels();
}

function point_camera_at_xy(point) {
    // Replace Z with the maximum Z on the whole model
    point.y = _cad_model_bbox.max.y
    point_camera_at_xyz(point);
}

function point_camera_at_xyz(point) {
    console.log(point);
    _cameraControls.target.copy(point);
    updateTrackingMarker(point);
    _cameraControls.update();
}

function onSyncHandler(data) {
    update_gcps(data.annotations);
}

function initialize() {
    viewport = document.getElementById('viewport');
    setup_scene();
    load_cad_model(window.location.href + '/model')

    viewport.addEventListener('pointerdown', onViewportMouseClick, false);
    window.addEventListener("resize", onWindowResize);

    const sse = initialize_event_source([
        { event: "sync", handler: onSyncHandler },
        { event: "move_camera", handler: point_camera_at_xy },
    ]);
    window.onunload = () => {
        console.log("Unloaded CAD window? closing sse")
        sse.close();
    }

    // call update
    update();
    post_json({ event: "init" });
}


function remove_point_observation() {
    post_json({ event: "remove_point_observation" });
}

function add_or_update_point_observation(xyz) {
    const data = {
        xyz: xyz,
        event: "add_or_update_point_observation",
    };
    post_json(data);
}

function onViewportMouseClick(event) {

    if (!event.ctrlKey && !event.altKey) {
        return
    }

    event.preventDefault();

    if (event.ctrlKey) {
        switch (event.button) {
            case 0: // left
                const pickposition = setPickPosition(event)
                _raycaster.setFromCamera(pickposition, camera);

                const intersections = _raycaster.intersectObject(_cad_model, true);
                const intersection = (intersections.length) > 0 ? intersections[0] : null;

                if (intersection !== null) {
                    const xyz = [intersection.point['x'], intersection.point['y'], intersection.point['z']];
                    add_or_update_point_observation(xyz);
                }
                break;
            case 1: // middle
                break;
            case 2: // right
                remove_point_observation();
                break;
        }
    }
    else { // Alt is pressed
        switch (event.button) {
            case 0: // left
                const pickposition = setPickPosition(event)
                _raycaster.setFromCamera(pickposition, camera);

                const intersections = _raycaster.intersectObject(_cad_model, true);
                const intersection = (intersections.length) > 0 ? intersections[0] : null;

                if (intersection !== null) {
                    point_camera_at_xyz(intersection.point);
                }
                break;
            case 1: // middle
                break;
            case 2: // right
                break;
        }

    }

}

function getCanvasRelativePosition(event) {
    const rect = viewport.getBoundingClientRect();
    return {
        x: (event.clientX - rect.left) * window.innerWidth / rect.width,
        y: (event.clientY - rect.top) * window.innerHeight / rect.height,
    };
}

function setPickPosition(event) {
    const pos = getCanvasRelativePosition(event);
    return {
        x: (pos.x / window.innerWidth) * 2 - 1,
        y: (pos.y / window.innerHeight) * -2 + 1,  // note we flip Y
    };
}

// function onWindowResize() {

//     camera.aspect = window.innerWidth / window.innerHeight;
//     camera.updateProjectionMatrix();

//     renderer.setSize(window.innerWidth, window.innerHeight);

// }

//a cross-browser method for efficient animation, more info at:
// http://paulirish.com/2011/requestanimationframe-for-smart-animating/
window.requestAnimFrame = (function () {
    return window.requestAnimationFrame ||
        window.webkitRequestAnimationFrame ||
        window.mozRequestAnimationFrame ||
        window.oRequestAnimationFrame ||
        window.msRequestAnimationFrame ||
        function (callback) {
            window.setTimeout(callback, 1000 / 60);
        };
})();

//----Update----//
function update() {
    // Update GCP labels so that they track the camera
    updateGCPLabels();

    _cameraControls.update(1);

    pointLight.position.copy(camera.position);

    draw();

    // requests the browser to call update again at it's own pace
    requestAnimFrame(update);
}

function draw() {
    renderer.render(_scene, camera);
}
