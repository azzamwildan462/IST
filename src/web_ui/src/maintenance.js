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
    displayAxes: false
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
// ================================================

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
        opacity: 0.7,
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
function set_steering(selector1, id_text, value1, raw_actuation) {
    value1 = value1 * -1;

    if (value1 > 100) value1 = 100;

    const new_value1 = value1 / 100 * 75;

    const progressText = document.getElementById(id_text);
    var circle = document.querySelector(selector1);
    var length = circle.getTotalLength(); // Get the total length of the circle's path

    circle.style.strokeDasharray = length; // Set the stroke dasharray to the total length of the circle
    circle.style.strokeDashoffset = length - (length * (new_value1 / 100)); // Initially hide the stroke

    circle.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    progressText.textContent = `${raw_actuation.toFixed(2)}`;
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

    set_gyro(".gyro-circle1", "gyro-text", gyro_value, yaw_deg);
}
);

function set_gyro(selector1, id_text, value1, raw_actuation) {
    value1 = value1 * -1;

    if (value1 > 100) value1 = 100;

    const new_value1 = value1 / 100 * 75;

    const progressText = document.getElementById(id_text);
    var circle = document.querySelector(selector1);
    var length = circle.getTotalLength(); // Get the total length of the circle's path

    circle.style.strokeDasharray = length; // Set the stroke dasharray to the total length of the circle
    circle.style.strokeDashoffset = length - (length * (new_value1 / 100)); // Initially hide the stroke

    circle.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    progressText.textContent = `${raw_actuation.toFixed(2)}`;
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