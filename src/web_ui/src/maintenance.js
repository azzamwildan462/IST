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
    url: "ws://" + window.location.hostname + ":9090",
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
    background: "#ffffff"
});
viewer.addObject(new ROS3D.Grid({ color: "#cccccc" }));

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
const url_kamera = "http://" + window.location.hostname + ":8080/stream?topic=/camera/rs2_cam_main/color/image_raw";

img.src = url_kamera;

// draw one static rectangle (x,y,w,h) in *displayed* pixels
function drawStaticBox() {
    ctx.clearRect(0, 0, cvs.width, cvs.height);
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#00d1b2';
    ctx.strokeRect(100, 80, 160, 90); // <-- change these numbers
}

// make sure canvas matches the displayed image size
function syncCanvasToImage() {
    const r = img.getBoundingClientRect();
    cvs.width = Math.round(r.width);
    cvs.height = Math.round(r.height);
}

// draw after the first frame is laid out
img.addEventListener('load', () => {
    syncCanvasToImage();
    drawStaticBox();
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