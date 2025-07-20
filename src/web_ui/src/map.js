class Robot {
    constructor(ip, conv_circle, conv_line, x, y, theta, radius) {
        this.ip = ip;
        this.conv_circle = conv_circle;
        this.conv_line = conv_line;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.radius = radius;
        this.routes = [];
        this.routes_line = [];

        this.color_r = 0;
        this.color_g = 0;
        this.color_b = 0;

        this.has_finished_routes_init = false;
        this.prev_has_finished_routes_init = false;
        this.has_route_drawed = false;

    }
}

// ================================================================================================================================

let robots = [];

// ================================================================================================================================


let wtf_skala = 10;
// let wtf_skala = 60;


// Create a Konva Stage
const stage = new Konva.Stage({
    container: 'map',
    width: window.innerWidth,
    height: window.innerHeight,
});

// Create a layer for grid and shapes
const robotLayer = new Konva.Layer();
const mapLayer = new Konva.Layer();
stage.add(mapLayer);
stage.add(robotLayer);

// ================================================================================================================================

// Enable zooming
stage.on('wheel', (e) => {
    e.evt.preventDefault();
    const scaleBy = 1.1;
    const oldScale = stage.scaleX();
    const pointer = stage.getPointerPosition();
    const mousePointTo = {
        x: (pointer.x - stage.x()) / oldScale,
        y: (pointer.y - stage.y()) / oldScale,
    };

    const newScale = e.evt.deltaY > 0 ? oldScale / scaleBy : oldScale * scaleBy;
    stage.scale({ x: newScale, y: newScale });

    const newPos = {
        x: pointer.x - mousePointTo.x * newScale,
        y: pointer.y - mousePointTo.y * newScale,
    };
    stage.position(newPos);
    stage.batchDraw();
});

// Enable map shifting (panning) with the right mouse button
let isDragging = false;
let dragStartPos = { x: 0, y: 0 };

stage.on('mousedown', (e) => {
    if (e.evt.button === 2) { // Check if the right mouse button is pressed
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }

    if (e.evt.button === 1 || e.evt.button === 2) {
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }
});

stage.on('mousemove', (e) => {
    if (!isDragging) return;

    const pointer = stage.getPointerPosition();
    const dx = pointer.x - dragStartPos.x;
    const dy = pointer.y - dragStartPos.y;

    stage.position({
        x: stage.x() + dx,
        y: stage.y() + dy,
    });
    stage.batchDraw();
    dragStartPos = pointer;
});

stage.on('mouseup', () => {
    isDragging = false;
});

stage.on('contextmenu', (e) => {
    // Prevent the browser's context menu from appearing on right-click
    e.evt.preventDefault();
});

// Handle window resizing
window.addEventListener('resize', () => {
    const width = window.innerWidth;
    const height = window.innerHeight;

    stage.width(width);
    stage.height(height);

    robotLayer.batchDraw();
    mapLayer.batchDraw();
});

// ================================================================================================================================

function addRobot(ip, x, y, theta, radius, colourr) {
    const original_x = x;
    const original_y = y;
    const original_theta = theta;

    ip = ip;
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;
    theta = theta;
    radius = radius;


    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            robots[i].conv_circle.position({ x: x * wtf_skala, y: y * wtf_skala });
            robots[i].conv_line.points([x * wtf_skala, y * wtf_skala, x * wtf_skala + radius * wtf_skala * Math.cos(theta), y * wtf_skala - radius * wtf_skala * Math.sin(theta)]);
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;

            robotLayer.batchDraw();

            return;
        }
    }

    const conv_circle = new Konva.Circle({
        x: x * wtf_skala,
        y: y * wtf_skala,
        radius: radius * wtf_skala,
        fill: colourr,
        draggable: true,
    });

    // Calculate the end point of the line based on theta
    const thetaRadians = theta; // Convert to radians
    const lineEndX = x * wtf_skala + radius * wtf_skala * Math.cos(thetaRadians);
    const lineEndY = y * wtf_skala - radius * wtf_skala * Math.sin(thetaRadians);

    // Draw the line inside the circle
    const conv_line = new Konva.Line({
        points: [x * wtf_skala, y * wtf_skala, lineEndX, lineEndY],
        stroke: 'Cyan',
        strokeWidth: 5,
    });

    let robot_buffer = new Robot(ip, conv_circle, conv_line, x, y, theta, radius);

    robots.push(robot_buffer);

    robotLayer.add(robot_buffer.conv_circle);
    robotLayer.add(robot_buffer.conv_line);
    robotLayer.draw();
}

// ================================================================================================================================

// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

let last_time_update_map = 0;
let mapCanvas = document.createElement("canvas");
let mapCtx = mapCanvas.getContext("2d");

const EMERGENCY_LIDAR_DEPAN_DETECTED = 0b010
const EMERGENCY_CAMERA_OBS_DETECTED = 0b100
const EMERGENCY_GYRO_ANOMALY_DETECTED = 0b1000
const EMERGENCY_ICP_SCORE_TERLALU_BESAR = 0b10000
const EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR = 0b100000
const EMERGENCY_STOP_KARENA_OBSTACLE = 0b1000000
const STATUS_TOWING_ACTIVE_AUTO = 0b01

const status_emergency = document.getElementById('status-emergency');
const terminal_terakhir = document.getElementById('terminal-terakhir');

let isTryingToConnect = false;

const alarm = new Audio('alarm1.wav');
alarm.loop = true;  // 🔁 Set looping once
let test_audio_play = 0


// ================================================================

let robot_odometry_subscriber = null;
let robot_map_subscriber = null;
let robot_status_emergency_subscriber = null;
let robot_terminal_terakhir_subscriber = null;

// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

function connectRos() {
    if (isTryingToConnect) return;

    isTryingToConnect = true;
    console.log('Attempting to connect to rosbridge...');

    ros.connect('ws://' + window.location.hostname + ':9090');

    // Give up this attempt if not connected after a few seconds
    setTimeout(() => {
        if (!ros.isConnected) {
            isTryingToConnect = false;
        }
    }, 3000);
}

// First connection attempt
connectRos();

// Try to reconnect every 3 seconds if not connected
setInterval(() => {
    if (!ros.isConnected) {
        status_emergency.innerHTML = "Towing Disconnected";
        connectRos();
    }
}, 3000);

// CAPEKKKKK, NEXT menambahkan terminal terakhir
ros.on("connection", function () {
    console.log("Connected to WebSocket server.");

    // ===================================================

    robot_odometry_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/slam/odometry/filtered',
        messageType: 'nav_msgs/Odometry'
    });

    robot_odometry_subscriber.subscribe(function (message) {
        const x = message.pose.pose.position.x;
        const y = message.pose.pose.position.y;
        const q = message.pose.pose.orientation;
        const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const radius = 2.05;

        addRobot("0.0.0.0", x, y, theta, radius, 'magenta');
    });

    // ===================================================

    robot_map_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/slam/map', // Change topic name as needed
        messageType: 'nav_msgs/OccupancyGrid'
    });

    // Subscribe to map messages and draw the map
    robot_map_subscriber.subscribe(function (message) {
        let map = message.data;
        let width = message.info.width;
        let height = message.info.height;

        // Resize the canvas buffer
        mapCanvas.width = width;
        mapCanvas.height = height;

        // Draw the map to the canvas buffer
        let imageData = mapCtx.createImageData(width, height);
        for (let i = 0; i < map.length; i++) {
            let occupancy = map[i];

            let r, g, b;

            if (occupancy === -1) {
                // Unknown space -> Dark Green
                r = 71;
                g = 128;
                b = 118;
            } else if (occupancy === 0) {
                // Free space -> White
                r = 255;
                g = 255;
                b = 255;
            } else {
                // Occupied space -> Color gradient (Red to Orange)
                let t = occupancy / 100; // Normalize to 0-1

                // Interpolate red and green between red (255, 0, 0) and orange (255, 165, 0)
                r = 0; // Red is fixed
                g = Math.round(165 - t * 165); // From 165 to 0 (green scale)
                b = 0; // Blue stays 0 for red-to-orange color
            }

            // Assign colors to image data
            imageData.data[i * 4 + 0] = r;  // Red
            imageData.data[i * 4 + 1] = g;  // Green
            imageData.data[i * 4 + 2] = b;  // Blue
            imageData.data[i * 4 + 3] = 255; // Fully opaque
        }
        mapCtx.putImageData(imageData, 0, 0);

        let imageObj = new Image();
        imageObj.onload = function () {
            let mapImage = new Konva.Image({
                image: imageObj,
                width: width * wtf_skala * message.info.resolution,
                height: height * wtf_skala * message.info.resolution,
                x: (message.info.origin.position.x + stage.width() * 0.5 / wtf_skala) * wtf_skala,
                y: (stage.height() * 0.5 / wtf_skala - message.info.origin.position.y) * wtf_skala,
            });

            // Flip Y-axis if the map's origin is bottom-left
            mapImage.scaleY(-1);
            // mapImage.offsetY(height);

            // Clear and add the new map image
            mapLayer.destroyChildren();
            mapLayer.add(mapImage);
            mapLayer.draw();
        };
        imageObj.src = mapCanvas.toDataURL();

        console.log("Map received and drawn.");

        robot_map_subscriber.unsubscribe(); // Hanya kirim sekali saja
    });

    // ===================================================

    robot_status_emergency_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: "/master/status_emergency",
        messageType: "std_msgs/Int16",
    });

    robot_status_emergency_subscriber.subscribe(function (message) {
        if ((message.data & STATUS_TOWING_ACTIVE_AUTO) == 0) {
            status_emergency.innerHTML = "Towing Mode Manual";
        }
        else if ((message.data & EMERGENCY_STOP_KARENA_OBSTACLE) == EMERGENCY_STOP_KARENA_OBSTACLE) {
            status_emergency.innerHTML = "WARNING: Towing Berhenti Karena Obstacle";
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_GYRO_ANOMALY_DETECTED) == EMERGENCY_GYRO_ANOMALY_DETECTED) {
            status_emergency.innerHTML = "WARNING: Gyro Anomali Terdeteksi";
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_ICP_SCORE_TERLALU_BESAR) == EMERGENCY_ICP_SCORE_TERLALU_BESAR) {
            status_emergency.innerHTML = "WARNING: Anomali Lingkungan Terdeteksi";
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR) == EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR) {
            status_emergency.innerHTML = "WARNING: Hipotesis Kesalahan Posisi";
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_LIDAR_DEPAN_DETECTED) == EMERGENCY_LIDAR_DEPAN_DETECTED) {
            status_emergency.innerHTML = "WARNING: Lidar Mendeteksi Objek";
        }
        else if ((message.data & EMERGENCY_CAMERA_OBS_DETECTED) == EMERGENCY_CAMERA_OBS_DETECTED) {
            status_emergency.innerHTML = "WARNING: Kamera Mendeteksi Objek";
        }
        else {
            status_emergency.innerHTML = "Towing Normal";
            if (test_audio_play == 0) {
                if (!alarm.paused) {
                    alarm.pause();
                    alarm.currentTime = 0;
                }
            }
        }
    });

    // ===================================================

    robot_terminal_terakhir_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: "/master/terminal_terakhir",
        messageType: "std_msgs/Int16",
    });

    robot_terminal_terakhir_subscriber.subscribe(function (message) {
        let terminal_terakhir_value = message.data;

        if (terminal_terakhir_value == -1) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: Tempat Parkir";
        }
        else if (terminal_terakhir_value == 0) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Belokan Pertama";
        }
        else if (terminal_terakhir_value == 1) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalan Lurus Pertama";
        }
        else if (terminal_terakhir_value == 2) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Belokan Cleaning Toribe";
        }
        else if (terminal_terakhir_value == 3) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Lorong Pertama";
        }
        else if (terminal_terakhir_value == 4) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Belokan Bea Cukai";
        }
        else if (terminal_terakhir_value == 5) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Lorong Bea Cukai";
        }
        else if (terminal_terakhir_value == 6) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Unloading Toribe";
        }
        else if (terminal_terakhir_value == 7) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Lorong Bea Cukai";
        }
        else if (terminal_terakhir_value == 8) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Belokan Bea Cukai";
        }
        else if (terminal_terakhir_value == 9) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Lorong Pertama";
        }
        else if (terminal_terakhir_value == 10) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Belokan Cleaning Toribe";
        }
        else if (terminal_terakhir_value == 11) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalan Lurus Pertama";
        }
        else if (terminal_terakhir_value == 12) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Area Hashiboriki";
        }
    });
});

ros.on("error", function (error) {
    isTryingToConnect = false;
    console.log("Error connecting to WebSocket server:", error);
});

ros.on("close", function () {
    isTryingToConnect = false;
    console.log("Connection to WebSocket server closed.");

    if (robot_odometry_subscriber) {
        robot_odometry_subscriber.unsubscribe();
        robot_odometry_subscriber = null;
    }

    if (robot_map_subscriber) {
        robot_map_subscriber.unsubscribe();
        robot_map_subscriber = null;
    }

    if (robot_status_emergency_subscriber) {
        robot_status_emergency_subscriber.unsubscribe();
        robot_status_emergency_subscriber = null;
    }
});



// document.addEventListener('keydown', function (event) {
//     if (event.key == "j") {
//         test_audio_play = 1;
//     }
//     else if (event.key == 'u') {
//         test_audio_play = 0;
//     }

// });

// setInterval(() => {
//     if (test_audio_play == 1) {
//         if (alarm.paused) {
//             alarm.play();
//         }
//     }
//     else {
//         if (!alarm.paused) {
//             alarm.pause();
//             alarm.currentTime = 0;
//         }
//     }
// }, 20);