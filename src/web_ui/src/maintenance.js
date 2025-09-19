// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://" + "10.7.101.111" + ":9090",
});

ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
});

ros.on("error", function (error) {
    console.log("Error connecting to WebSocket server:", error);
});

ros.on("close", function () {
    console.log("Connection to WebSocket server closed.");
});

// SHARED VARIABLES
// ================================================================
let shared_steering_feedback = 0;
let shared_target_steering = 1.0472;

let shared_velocity_feedback = 0;
let shared_target_velocity = 2;

let shared_gyro_feedback = 0;
let shared_gyro_target = 0.5236;
let shared_gyro_offset = 0.1;

// ================================================================

// LIDAR BAWAH
// ===============================================================
const LIDAR_TOPIC = "/scan";                // sensor_msgs/LaserScan
const FIXED_FRAME = "lidar1_link";            // pick a frame that exists in your TF tree

const statusEl = document.getElementById("lidar-status");
const nextBtn = document.getElementById("next-1");

const viewer = new ROS3D.Viewer({
    divID: "lidarView",
    width: 560,
    height: 320,
    antialias: true,
    background: "#ffffff",
    cameraPose: { x: -2.0, y: 0, z: 0.3 },
    displayGrids: true,
    displayAxes: false,
    preserveDrawingBuffer: true
});

// ====== Add a grid to the viewer ======
var grid = new THREE.GridHelper(10, 10, 0x444444, 0x888888);
grid.rotation.x = Math.PI / 2;
viewer.scene.add(grid);

// ====== Don't remove these hidden objects ======
var hiddenGeo = new THREE.BoxGeometry(0.01, 0.01, 0.01);
var hiddenMat = new THREE.MeshLambertMaterial({ color: 0xffffff });
var hidden = new THREE.Mesh(hiddenGeo, hiddenMat);
hidden.position.set(0, 0, 0);
hidden.scale.set(0.0001, 0.0001, 0.0001);
hidden.position.set(1000, 1000, 1000);
viewer.addObject(hidden);

hiddenGeo = new THREE.BoxGeometry(0.01, 0.01, 0.01);
hiddenMat = new THREE.MeshLambertMaterial({ color: 0xffffff });
hidden = new THREE.Mesh(hiddenGeo, hiddenMat);
hidden.position.set(0, 0, 0);
hidden.scale.set(0.0001, 0.0001, 0.0001);
hidden.position.set(1000, 1000, 1000);
viewer.addObject(hidden);
// ============================================================================================
// PARAMETER
const boxWidth = 0.30;
const notAllowedBoxHeight = 0.15;
const allowedBoxHeigt = 0.2 - notAllowedBoxHeight;
const offsetAxisX = -1.5;

function addCalibratedBox(x, y) {
    // NOT ALLOWED (merah bawah)
    var geo1 = new THREE.BoxGeometry(boxWidth, boxWidth, notAllowedBoxHeight);
    var mat1 = new THREE.MeshBasicMaterial({
        color: 0xff0000,
        opacity: 0.5,
        transparent: true,
        polygonOffset: true,
        polygonOffsetFactor: 2,
        polygonOffsetUnits: 2
    });
    var mesh1 = new THREE.Mesh(geo1, mat1);
    mesh1.position.set(x + offsetAxisX, y, notAllowedBoxHeight / 2);
    viewer.scene.add(mesh1);

    var geo11 = new THREE.BoxGeometry(boxWidth, boxWidth, notAllowedBoxHeight);
    var mat11 = new THREE.MeshBasicMaterial({
        color: 0xff0000,
        opacity: 1,
        transparent: false,
        polygonOffset: true,
        polygonOffsetFactor: 0,
        polygonOffsetUnits: 0,
        wireframe: true
    });
    var mesh11 = new THREE.Mesh(geo11, mat11);
    mesh11.position.set(x + offsetAxisX, y, notAllowedBoxHeight / 2);
    viewer.scene.add(mesh11);

    // ALLOWED (hijau atas)
    var geo2 = new THREE.BoxGeometry(boxWidth, boxWidth, allowedBoxHeigt);
    var mat2 = new THREE.MeshBasicMaterial({
        color: 0x00ff00,
        opacity: 0.5,
        transparent: true,
        polygonOffset: true,
        polygonOffsetFactor: 2,
        polygonOffsetUnits: 2
    });
    var mesh2 = new THREE.Mesh(geo2, mat2);
    mesh2.position.set(x + offsetAxisX, y, notAllowedBoxHeight + allowedBoxHeigt / 2);
    viewer.scene.add(mesh2);

    var geo21 = new THREE.BoxGeometry(boxWidth, boxWidth, allowedBoxHeigt);
    var mat21 = new THREE.MeshBasicMaterial({
        color: 0x008800,
        opacity: 1,
        transparent: false,
        polygonOffset: true,
        polygonOffsetFactor: 0,
        polygonOffsetUnits: 0,
        wireframe: true
    });
    var mesh21 = new THREE.Mesh(geo21, mat21);
    mesh21.position.set(x + offsetAxisX, y, notAllowedBoxHeight + allowedBoxHeigt / 2);
    viewer.scene.add(mesh21);
}

// ============================================================================================
addCalibratedBox(1, 0);
addCalibratedBox(2, -1);
addCalibratedBox(3, 1);

var sphereGeo = new THREE.SphereGeometry(0.05, 32, 32);
var sphereMat = new THREE.MeshBasicMaterial({ color: 0x0000ff });
var sphere = new THREE.Mesh(sphereGeo, sphereMat);
sphere.position.set(offsetAxisX, 0, 0);
viewer.scene.add(sphere);

// ============================================================================================
window.lidarViewer = viewer;

// helper: ubah camera pose saat runtime dan paksa render
function setLidarCameraPose(x, y, z, lookAtX = 0, lookAtY = 0, lookAtZ = 0) {
    if (!window.lidarViewer || !window.lidarViewer.camera) return;
    const cam = window.lidarViewer.camera;
    cam.position.set(x, y, z);
    cam.lookAt(new THREE.Vector3(lookAtX, lookAtY, lookAtZ));
    if (window.lidarViewer.controls) {
        window.lidarViewer.controls.target.set(lookAtX, lookAtY, lookAtZ);
        window.lidarViewer.controls.update();
    }
    cam.updateProjectionMatrix();
    try { window.lidarViewer.renderer.render(window.lidarViewer.scene, cam); } catch (e) { console.warn('force render failed', e); }
}
window.setLidarCameraPose = setLidarCameraPose;
// ============================================================================================

// TF client (use a frame that exists; often map/odom/base_link/laser)
const tfClient = new ROSLIB.TFClient({
    ros,
    fixedFrame: FIXED_FRAME,
    angularThres: 0.01,
    transThres: 0.01,
    rate: 15.0
});

// ====== LASERSCAN VISUALIZER ======
const laser = new ROS3D.LaserScan({
    ros,
    tfClient,
    rootObject: viewer.scene,
    topic: LIDAR_TOPIC,
    // Optional styling
    material: { size: 0.02 }
});

// Unlock Next when first message arrives
const scanSub = new ROSLIB.Topic({
    ros,
    name: LIDAR_TOPIC,
    messageType: "sensor_msgs/LaserScan",
    throttle_rate: 500
});

let firstMsg = false;
scanSub.subscribe((msg) => {
    if (!firstMsg) {
        firstMsg = true;
        statusEl.textContent = `Streaming LiDAR: ${LIDAR_TOPIC} (${msg.ranges.length} beams)`;
        nextBtn.disabled = false;  // ✅ enable Next when data flows
    }
});

// Kamera
// ===============================================================

const img = document.getElementById('cam');
const cvs = document.getElementById('overlay');
const ctx = cvs.getContext('2d');
// const url_kamera = "http://" + window.location.hostname + ":8080/stream?topic=/camera/rs2_cam_main/color/image_raw";
const url_kamera = "http://" + "10.7.101.111" + ":8080/stream?topic=/image_bgr";

img.src = url_kamera;

const width = 640;
const height = 360;

// draw one static rectangle (x,y,w,h) in *displayed* pixels
function drawStaticBox() {
    ctx.clearRect(0, 0, cvs.width, cvs.height);
    console.log(cvs.width, cvs.height);
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#ff00ff';

    // make vertical line in the middle
    ctx.beginPath();
    ctx.moveTo((320) / width * cvs.width, 0 / height * cvs.height);
    ctx.lineTo((320) / width * cvs.width, height / height * cvs.height);
    ctx.stroke();

    // make horizontal line in the middle
    // ctx.beginPath();
    // ctx.moveTo(0 / width * cvs.width, (180) / height * cvs.height);
    // ctx.lineTo((640) / width * cvs.width, (180) / height * cvs.height);
    // ctx.stroke();

    const distance1 = 100; // pixels from bottom
    const distance2 = 120; // pixels from bottom
    const distance3 = 140; // pixels from bottom
    const distance4 = 160; // pixels from bottom
    const distance5 = 180; // pixels from bottom
    const distance6 = 200; // pixels from bottom

    ctx.beginPath();
    ctx.moveTo(0 / width * cvs.width, (height - distance1) / height * cvs.height);
    ctx.lineTo((640) / width * cvs.width, (height - distance1) / height * cvs.height);
    ctx.stroke();

    // ctx.beginPath();
    // ctx.moveTo(0 / width * cvs.width, (height - distance2) / height * cvs.height);
    // ctx.lineTo((640) / width * cvs.width, (height - distance2) / height * cvs.height);
    // ctx.stroke();

    // ctx.beginPath();
    // ctx.moveTo(0 / width * cvs.width, (height - distance3) / height * cvs.height);
    // ctx.lineTo((640) / width * cvs.width, (height - distance3) / height * cvs.height);
    // ctx.stroke();

    // ctx.beginPath();
    // ctx.moveTo(0 / width * cvs.width, (height - distance4) / height * cvs.height);
    // ctx.lineTo((640) / width * cvs.width, (height - distance4) / height * cvs.height);
    // ctx.stroke();

    // ctx.beginPath();
    // ctx.moveTo(0 / width * cvs.width, (height - distance5) / height * cvs.height);
    // ctx.lineTo((640) / width * cvs.width, (height - distance5) / height * cvs.height);
    // ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(0 / width * cvs.width, (height - distance6) / height * cvs.height);
    ctx.lineTo((640) / width * cvs.width, (height - distance6) / height * cvs.height);
    ctx.stroke();

    // make diagonal line
    const offset_center = 100;
    ctx.beginPath();
    ctx.moveTo((320 + offset_center) / width * cvs.width, (0) / height * cvs.height);
    ctx.lineTo((640) / width * cvs.width, (360) / height * cvs.height);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo((320 - offset_center) / width * cvs.width, (0) / height * cvs.height);
    ctx.lineTo((0) / width * cvs.width, (360) / height * cvs.height);
    ctx.stroke();

}

// make sure canvas matches the displayed image size
function syncCanvasToImage() {
    const r = img.getBoundingClientRect();
    cvs.width = Math.round(r.width);
    cvs.height = Math.round(r.height);
}

function drawOverlay() {
    cvs.width = img.clientWidth;
    cvs.height = img.clientHeight;

    ctx.clearRect(0, 0, cvs.width, cvs.height);
    drawStaticBox();
    requestAnimationFrame(drawOverlay);
}

// draw after the first frame is laid out
img.addEventListener('load', () => {
    requestAnimationFrame(drawOverlay);
});

// if you ever resize the <img> via CSS, keep overlay in sync
window.addEventListener('resize', () => {
    syncCanvasToImage();
    drawStaticBox();
});

// call once in case the image is already loaded
if (img.complete) { syncCanvasToImage(); drawStaticBox(); }

// LIDAR SAMPING
// =============================================================== 

// STEERING 
// ==============================================================
var sub_master2ui = new ROSLIB.Topic({
    ros: ros,
    name: "/master/to_ui",
    messageType: "std_msgs/Float32MultiArray",
});

sub_master2ui.subscribe(function (message) {
    let velocity_actuation = message.data[0];
    let steering_actuation = message.data[1];
    let velocity_feedback = message.data[2];
    let steering_feedback = message.data[3];

    let steering_target_value = shared_target_steering * 135 / 6.28;
    let steering_current_value = steering_feedback * 135 / 6.28;
    let steering_current_deg = steering_feedback * 180 / 3.14;

    let velocity_actuation_display = shared_target_velocity * 10 * 3.6;
    let velocity_feedback_display = velocity_feedback * 10 * 3.6;

    shared_steering_feedback = steering_feedback;
    shared_velocity_feedback = velocity_feedback;

    if (idx_global.innerText == "4") {
        set_steering(".steering-circle1", ".steering-circle2", "steering-text", steering_target_value, steering_current_value, steering_current_deg);
    } else if (idx_global.innerText == "5") {
        set_velocity(".velocity-circle1", ".velocity-circle2", "velocity-text", velocity_actuation_display, velocity_feedback_display);
    }
});

let var_global = document.getElementById("var-global");
let idx_global = document.getElementById("idx-global");

setInterval(() => {
    if (idx_global.innerText == "4") {
        if (shared_steering_feedback >= shared_target_steering) {
            var_global.setAttribute("class", "steering1");
            shared_target_steering = -Math.abs(shared_target_steering);
        }
        else if (var_global.getAttribute("class") === "steering1" && shared_steering_feedback <= shared_target_steering) {
            var_global.setAttribute("class", "steering2");
            shared_target_steering = 0;
        }
    } else if (idx_global.innerText == "5") {
        if (shared_velocity_feedback >= shared_target_velocity) {
            var_global.setAttribute("class", "velocity1");
            shared_target_velocity = 0;
        }
    } else if (idx_global.innerText == "6") {
        if (Math.abs(shared_gyro_target - shared_gyro_feedback) < 5 * 3.14 / 180 && shared_gyro_target > 0) {
            var_global.setAttribute("class", "gyro1");
            shared_gyro_target = -shared_gyro_target;
        } else if (Math.abs(shared_gyro_target - shared_gyro_feedback) < 5 * 3.14 / 180 && shared_gyro_target < 0) {
            var_global.setAttribute("class", "gyro2");
            shared_gyro_target = 0;
            shared_gyro_offset = 0;
        }
    }
}, 100);

function set_velocity(selector1, selector2, id_text, value1, value2) {
    if (value1 > 100) value1 = 100;
    if (value2 > 100) value2 = 100;

    const new_value1 = value1 / 100 * 75;
    const new_value2 = value2 / 100 * 75;

    const progressText = document.getElementById(id_text);
    const velocity_kmph = document.getElementById("velocity-kmph");
    var circle = document.querySelector(selector1);
    var circle2 = document.querySelector(selector2);
    var length = circle.getTotalLength(); // Get the total length of the circle's path
    var length2 = circle2.getTotalLength();

    circle.style.strokeDasharray = length; // Set the stroke dasharray to the total length of the circle
    circle.style.strokeDashoffset = length - (length * (new_value1 / 100)); // Initially hide the stroke

    circle2.style.strokeDasharray = length2; // Set the stroke dasharray to the total length of the circle
    circle2.style.strokeDashoffset = length2 - (length2 * (new_value2 / 100)); // Initially hide the stroke

    value1_display = value2 * 0.1;
    progressText.textContent = `${value1_display.toFixed(2)}`;
    velocity_kmph.textContent = `${value1_display.toFixed(2)}` + " km/h";
}

function set_steering(selector1, slector2, id_text, value1, value2, raw_actuation) {
    value1 = value1 * -1;
    value2 = value2 * -1;

    if (value1 > 100) value1 = 100;
    if (value2 > 100) value2 = 100;

    const new_value1 = value1 / 100 * 75;
    const new_value2 = value2 / 100 * 75;

    const progressText = document.getElementById(id_text);
    var circle1 = document.querySelector(selector1);
    var length1 = circle1.getTotalLength(); // Get the total length of the circle's path

    circle1.style.strokeDasharray = length1; // Set the stroke dasharray to the total length of the circle
    circle1.style.strokeDashoffset = length1 - (length1 * (new_value1 / 100)); // Initially hide the stroke

    circle1.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle1.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    var circle2 = document.querySelector(slector2);
    var length2 = circle2.getTotalLength(); // Get the total length of the circle's path

    circle2.style.strokeDasharray = length2; // Set the stroke dasharray to the total length of the circle
    circle2.style.strokeDashoffset = length2 - (length2 * (new_value2 / 100)); // Initially hide the stroke

    circle2.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle2.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    progressText.textContent = `${raw_actuation.toFixed(1)}` + "°";
}

// GYRO
// ==============================================================
const gyroSub = new ROSLIB.Topic({
    ros,
    name: "/slam/odometry/filtered",
    messageType: "nav_msgs/Odometry",
    throttle_rate: 100
});
gyroSub.subscribe((msg) => {
    // assuming pitch and roll are zero 
    const yaw = Math.atan2(
        2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
        1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
    );

    let gyro_value = (yaw) * 135 / 6.28;
    let yaw_deg = yaw * 180 / 3.14;

    shared_gyro_feedback = yaw;

    if (idx_global.innerText == "6") {
        set_gyro(".gyro-circle00", ".gyro-circle1", ".gyro-circle2", "gyro-text", shared_gyro_target * 135 / 6.28, gyro_value, yaw_deg, shared_gyro_offset * 135 / 6.28);
    }
}
);

function set_gyro(selector0, selector1, selector2, id_text, value1, value2, raw_actuation, offset) {
    value1 = value1 * -1;
    value2 = value2 * -1;

    if (value1 > 100) value1 = 100;
    if (value2 > 100) value2 = 100;

    let new_value0 = value1 / 100 * 75;
    let new_value1 = value1 / 100 * 75;
    const new_value2 = value2 / 100 * 75;

    if (value1 > 0) {
        new_value0 = new_value0 - offset;
        new_value1 = new_value1 + offset;
    } else {
        new_value0 = new_value0 + offset;
        new_value1 = new_value1 - offset;
    }

    const progressText = document.getElementById(id_text);

    var circle0 = document.querySelector(selector0);
    var circle1 = document.querySelector(selector1);
    var circle2 = document.querySelector(selector2);

    var length0 = circle0.getTotalLength(); // Get the total length of the circle0's path
    var length1 = circle1.getTotalLength(); // Get the total length of the circle1's path
    var length2 = circle2.getTotalLength(); // Get the total length of the circle2's path

    circle0.style.strokeDasharray = length0; // Set the stroke dasharray to the total length of the circle0
    circle0.style.strokeDashoffset = length0 - (length0 * (new_value0 / 100)); // Initially hide the stroke

    circle0.style.transform = "rotate(-90deg)";
    circle0.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    circle1.style.strokeDasharray = length1; // Set the stroke dasharray to the total length of the circle1
    circle1.style.strokeDashoffset = length1 - (length1 * (new_value1 / 100)); // Initially hide the stroke

    circle1.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle1.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    circle2.style.strokeDasharray = length2; // Set the stroke dasharray to the total length of the circle2
    circle2.style.strokeDashoffset = length2 - (length2 * (new_value2 / 100)); // Initially hide the stroke

    circle2.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle2.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    progressText.textContent = `${raw_actuation.toFixed(1)}` + "°";
}

class CallibratedBox {
    constructor(position, boxSize, allowedSize, notAllowedSize, viewer) {
        this.position = position;
        this.boxSize = boxSize;
        this.allowedSize = allowedSize;
        this.notAllowedSize = notAllowedSize;
        this.viewer = viewer;

        this.drawBox();
    }

    drawBox() {
        const geo1 = new THREE.BoxGeometry(this.boxSize, this.boxSize, this.notAllowedSize);
        const mat1 = new THREE.MeshBasicMaterial({ color: 0xff0000, opacity: 0.5, transparent: true });
        const mesh1 = new THREE.Mesh(geo1, mat1);
        mesh1.position.set(this.position.x, this.position.y, this.notAllowedSize / 2);
        this.viewer.scene.add(mesh1);

        const edges1 = new THREE.EdgesGeometry(geo1);
        const wire1 = new THREE.LineSegments(edges1, new THREE.LineBasicMaterial({ color: 0x000000 }));
        wire1.position.copy(mesh1.position);
        this.viewer.scene.add(wire1);

        const geo2 = new THREE.BoxGeometry(this.boxSize, this.boxSize, this.allowedSize);
        const mat2 = new THREE.MeshBasicMaterial({ color: 0x00ff00, opacity: 0.5, transparent: true });
        const mesh2 = new THREE.Mesh(geo2, mat2);
        mesh2.position.set(this.position.x, this.position.y, this.notAllowedSize + this.allowedSize / 2);
        this.viewer.scene.add(mesh2);

        const edges2 = new THREE.EdgesGeometry(geo2);
        const wire2 = new THREE.LineSegments(edges2, new THREE.LineBasicMaterial({ color: 0x000000 }));
        wire2.position.copy(mesh2.position);
        this.viewer.scene.add(wire2);
    }
}

let steering_test_fb = 0.5;
let velocity_test_fb = 0.0;
let gyro_test_fb = 0.0;

setInterval(() => {
    const steering_test_act = 1;
    shared_steering_feedback = steering_test_fb;


    let steering_target_value = shared_target_steering * 135 / 6.28;
    let steering_current_value = steering_test_fb * 135 / 6.28;
    let steering_current_deg = steering_test_fb * 180 / 3.14;

    let velocity_test_act = 0.0;
    shared_velocity_feedback = velocity_test_fb;

    let velocity_actuation_display = shared_target_velocity * 10 * 3.6;
    let velocity_feedback_display = velocity_test_fb * 10 * 3.6;

    let gyro_current_value = gyro_test_fb * 135 / 6.28;
    let gyro_yaw_deg = gyro_test_fb * 180 / 3.14;
    shared_gyro_feedback = gyro_test_fb;

    let target_steering_value0 = 0.5 * 135 / 6.28;
    let target_steering_value1 = 0.9 * 135 / 6.28;

    let offset_steering = 0.5 * 135 / 6.28;

    // set_target_steering(".steering-circle-target", target_steering_value0, offset_steering);

    if (idx_global.innerText == "4") {
        set_steering(".steering-circle1", ".steering-circle2", "steering-text", steering_target_value, steering_current_value, steering_current_deg);
        if (shared_target_steering > 0) {
            steering_test_fb += 0.1;
        }
        else {
            steering_test_fb -= 0.1;
        }
    } else if (idx_global.innerText == "5") {
        set_velocity(".velocity-circle1", ".velocity-circle2", "velocity-text", velocity_actuation_display, velocity_feedback_display);
        velocity_test_fb += 0.1;
    } else if (idx_global.innerText == "6") {
        gyro_test_fb += 0.05;
        set_gyro(".gyro-circle00", ".gyro-circle1", ".gyro-circle2", "gyro-text", shared_gyro_target * 135 / 6.28, gyro_current_value, gyro_yaw_deg, shared_gyro_offset * 135 / 6.28);
        if (gyro_test_fb > 3.14) {
            gyro_test_fb -= 6.28;
        }
    }
}, 100);