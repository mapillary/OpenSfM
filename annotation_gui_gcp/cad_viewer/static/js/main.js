import * as THREE from 'https://unpkg.com/three/build/three.module.js';
import { OBJLoader } from 'https://unpkg.com/three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'https://unpkg.com/three/examples/jsm/loaders/MTLLoader.js';
import { FlyControls } from 'https://unpkg.com/three/examples/jsm/controls/FlyControls.js';
import Stats from 'https://unpkg.com/three/examples/jsm/libs/stats.module.js';

//----Variables----//
//DOM element to attach the renderer to
var viewport;

//built-in three.js controls will be attached to this
var controls;

//viewport size
let viewportWidth = 800;
let viewportHeight = 600;

//camera attributes
const view_angle = 45;
const aspect = viewportWidth / viewportHeight;

//----Constructors----//
const renderer = new THREE.WebGLRenderer({ antialias: true });
const scene = new THREE.Scene();
let clock = new THREE.Clock();

const camera = new THREE.PerspectiveCamera(view_angle, aspect);

//constructs an instance of a white light
renderer.setClearColor(0x006600); // Green background
const pointLight = new THREE.PointLight(0xFFFFFF);


// clicking
let stats;
let pointclouds = [];
let raycaster;
let intersection = null;
let spheresIndex = 0;
let toggle = 0;

const mouse = new THREE.Vector2();
const spheres = [];
const n_spheres = 1;
const threshold = 0.1;
// const object_scale = 1.0

function load_cad_model(path_model, path_material) {
    console.log("Loading CAD model")
    const mtlLoader = new MTLLoader();
    mtlLoader.load(path_material, (mtl) => {

        mtl.preload();
        const objLoader = new OBJLoader();
        objLoader.setMaterials(mtl);
        objLoader.load(path_model, (root) => {

            // Set as true to maintain the consistency of vertex position after scaling
            // for (let i = 0, l = root.children.length; i < l; i++) {
            //     root.children[i].geometry.verticesNeedUpdate = true
            // }

            // root.scale.set(object_scale, object_scale, object_scale);
            scene.add(root);

            // add points
            for (let i = 0, l = root.children.length; i < l; i++) {
                let material = new THREE.PointsMaterial({ color: 0xFFFFFF, size: 0.25 })
                let mesh = new THREE.Points(root.children[i].geometry, material)
                //mesh.scale.set(object_scale, object_scale, object_scale);
                pointclouds.push(mesh);
                scene.add(mesh)
            }

            // // Set the camera position to object center
            // const i = root.children.length;
            // const geometry = root.children[i - 1].geometry;
            // geometry.computeBoundingBox();
            // let center = new THREE.Vector3();
            // geometry.boundingBox.getCenter(center);
            // root.children[i - 1].localToWorld(center);
            // center = center.multiplyScalar(object_scale)
            // camera.position.copy(center);
        });
    });
}


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

function setup_scene() {
    //Sets up the renderer to the same size as a DOM element
    //and attaches it to that element
    renderer.setSize(viewportWidth, viewportHeight);
    viewport = document.getElementById('viewport');
    viewport.appendChild(renderer.domElement);

    camera.position.set(10, 10, 10);
    camera.lookAt(new THREE.Vector3(0, 0, 0));

    //attaches fly controls to the camera
    controls = new FlyControls(camera, renderer.domElement);
    controls.movementSpeed = 1;
    controls.domElement = viewport;
    controls.rollSpeed = 0.01;
    controls.autoForward = false;
    controls.dragToLook = true;

    pointLight.position.set(10, 50, 150);

    raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = threshold;

    stats = new Stats();

    scene.add(camera);
    scene.add(pointLight);
}

function initialize() {
    setup_scene();


    // Add a sphere that we'll use as a marker
    const sphereGeometry = new THREE.SphereBufferGeometry(0.1, 32, 32);
    const sphereMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    for (let i = 0; i < n_spheres; i++) {
        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        scene.add(sphere);
        spheres.push(sphere);
    }

    // Add the model
    const path_model = "/static/resources/teapot.obj";
    const path_material = "/static/resources/teapot.mtl";
    load_cad_model(path_model, path_material)



    window.addEventListener('resize', onWindowResize, false);
    // document.addEventListener( 'pointerdown', onDocumentMouseClick, false );

    // call update
    update();
}


function onDocumentMouseClick(event) {

    event.preventDefault();

    switch (event.button) {
        case 0: // left
            const pickposition = setPickPosition(event)
            mouse.x = pickposition.x;
            mouse.y = pickposition.y;


            raycaster.setFromCamera(mouse, camera);

            const intersections = raycaster.intersectObjects(pointclouds, true);
            intersection = (intersections.length) > 0 ? intersections[0] : null;

            if (toggle > 0.02 && intersection !== null) {

                console.log("Select point:");
                console.log(intersection.point);


                const method = 'POST';
                const headers = {
                    'Accept': 'application/json',
                    'Content-Type': 'application/json',
                };

                const data = {
                    xyz: [intersection.point['x'], intersection.point['y'], intersection.point['z']],
                    command: "add_or_update_point_observation",
                };
                const body = JSON.stringify(data, null, 4);
                const url = 'postdata';

                const response = fetch(url, { method, headers, body })


                spheres[spheresIndex].position.copy(intersection.point);
                spheres[spheresIndex].scale.set(1, 1, 1);
                spheresIndex = (spheresIndex + 1) % spheres.length;

                toggle = 0;
            }

            for (let i = 0; i < spheres.length; i++) {

                const sphere = spheres[i];
                sphere.scale.multiplyScalar(0.98);
                sphere.scale.clampScalar(0.01, 1);

            }
            break;
        case 1: // middle
            break;
        case 2: // right
            const method = 'POST';
            const headers = {
                'Accept': 'application/json',
                'Content-Type': 'application/json',
            };

            const data = {
                command: "remove_point_observation",
            };
            const body = JSON.stringify(data, null, 4);
            const url = 'postdata';

            const response = fetch(url, { method, headers, body })
            break;
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

function onWindowResize() {

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    renderer.setSize(window.innerWidth, window.innerHeight);

}

//----Update----//
function update() {
    //requests the browser to call update at it's own pace
    requestAnimFrame(update);

    //update controls
    controls.update(1);

    //call draw
    draw();
    stats.update();
}

//----Draw----//
function draw() {
    toggle += clock.getDelta();
    renderer.render(scene, camera);
}


document.onload = initialize();