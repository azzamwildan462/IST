class Robot {
    constructor(ip, conv_circle, conv_line, conv_image, conv_toribe, x, y, theta, radius) {
        this.ip = ip;
        this.conv_circle = conv_circle;
        this.conv_line = conv_line;
        this.conv_image = conv_image;
        this.conv_toribe = conv_toribe;
        this.joint_circle = null;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.radius = radius;
        this.routes = [];
        this.routes_line = [];

        // Add trailer physics properties
        this.toribe_x = x;
        this.toribe_y = y;
        this.toribe_theta = theta;
        this.prev_x = x;
        this.prev_y = y;
        this.prev_theta = theta;

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
    // position: {x: 776.1564000000002, y: 132.96359999999993},
    // scale: {x: 0.5499159600000001, y: 0.5499159600000001}
    // position: {x: 913.9893384540003, y: 66.63718094599972},
    // scale: {x: 0.8856451527396005, y: 0.8856451527396005}

});

// Create a layer for grid and shapes
const robotLayer = new Konva.Layer();
const mapLayer = new Konva.Layer();
const waypointsLayer = new Konva.Layer();
stage.add(waypointsLayer);
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
function normalizeAngle(angle) {
    while (angle > Math.PI) {
        angle -= 2 * Math.PI;
    }
    while (angle < -Math.PI) {
        angle += 2 * Math.PI;
    }
    return angle;
}

function addRobotImageWithToribe(ip, x, y, theta, radius, imageScale) {
    // console.log(`addRobotImageWithToribe called: ip=${ip}, x=${x}, y=${y}, theta=${(theta * 180/Math.PI).toFixed(1)}°`);
    
    const original_x = x;
    const original_y = y;
    const original_theta = theta;
    
    // Transform coordinates
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;

    // Check if robot already exists and UPDATE it
    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            // Update existing robot positions with realistic trailer physics
            const towingX = x * wtf_skala;
            const towingY = y * wtf_skala;
            
            // Calculate velocity and steering angle
            const dx = original_x - robots[i].prev_x;
            const dy = original_y - robots[i].prev_y;
            const velocity = Math.sqrt(dx * dx + dy * dy);
            const dt = 1.5; // 50ms time step
            
            // Trailer physics parameters
            const L = radius * 2.5; // Distance between axles (hitch length)
            const trailer_L = radius * 2.5; // Trailer wheelbase
            
            // Calculate steering angle from velocity with angle normalization
            const dtheta = normalizeAngle(original_theta - robots[i].prev_theta);
            let steering_angle = 0;
            if (velocity > 0.01) {
                steering_angle = Math.atan2(dtheta * L, velocity * dt);
                // Limit steering angle
                steering_angle = Math.max(-Math.PI/4, Math.min(Math.PI/4, steering_angle));
            }
            
            // Update trailer position using bicycle model
            if (velocity > 0.01) {
                // Calculate target trailer angle with proper normalization
                const trailer_theta_target = normalizeAngle(original_theta - steering_angle);
                const current_trailer_theta = normalizeAngle(robots[i].toribe_theta);
                
                // Smooth trailer rotation with proper angle difference handling
                const angle_diff = normalizeAngle(trailer_theta_target - current_trailer_theta);
                robots[i].toribe_theta = normalizeAngle(current_trailer_theta + angle_diff * 0.3);
                
                // Calculate trailer position based on physics
                const joint_offset_x = -L * Math.cos(original_theta);
                const joint_offset_y = -L * Math.sin(original_theta);
                const trailer_offset_x = -trailer_L * Math.cos(robots[i].toribe_theta);
                const trailer_offset_y = -trailer_L * Math.sin(original_theta);
                
                robots[i].toribe_x = original_x + (joint_offset_x + trailer_offset_x) / wtf_skala;
                robots[i].toribe_y = original_y + (joint_offset_y + trailer_offset_y) / wtf_skala;
            }
            
            // Update towing vehicle position and rotation
            robots[i].conv_image.position({
                x: towingX,
                y: towingY
            });
            robots[i].conv_image.rotation(90 + original_theta * -180 / Math.PI);
            
            // Calculate joint position (hitch point)
            const jointDistance = radius * 1.2 * wtf_skala;
            const jointX = towingX - jointDistance * Math.cos(original_theta);
            const jointY = towingY + jointDistance * Math.sin(original_theta);
            
            // Update joint position
            if (robots[i].joint_circle) {
                robots[i].joint_circle.position({
                    x: jointX,
                    y: jointY
                });
            }
            
            const toribeDistance = radius * 0.7 * wtf_skala; // Jarak toribe dari joint
            const toribeX = jointX - toribeDistance * Math.cos(robots[i].toribe_theta); // Toribe di belakang joint
            const toribeY = jointY + toribeDistance * Math.sin(robots[i].toribe_theta); // Toribe di belakang joint
            
            // Update toribe position BERDASARKAN JOINT, bukan screen coordinates
            if (robots[i].conv_toribe) {
                robots[i].conv_toribe.position({
                    x: toribeX,
                    y: toribeY
                });
                robots[i].conv_toribe.rotation(90 + robots[i].toribe_theta * -180 / Math.PI);
            }
            // Update direction line
            if (robots[i].conv_line) {
                robots[i].conv_line.points([
                    towingX, towingY, 
                    towingX + radius * wtf_skala * Math.cos(original_theta), 
                    towingY - radius * wtf_skala * Math.sin(original_theta)
                ]);
            }
            
            // Store previous values for next iteration
            robots[i].prev_x = original_x;
            robots[i].prev_y = original_y;
            robots[i].prev_theta = original_theta;
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;

            robotLayer.batchDraw();
            return; // IMPORTANT: Return here to prevent creating new robot
        }
    }

    // Only create new robot if it doesn't exist
    // console.log(`Creating NEW robot with IP: ${ip}`);
    
    // Check if images are loaded
    if (!towingimage.complete || !toribeimage.complete) {
        console.error("Images not loaded yet");
        return;
    }

    // IMPORTANT: Remove any existing robot with same IP before creating new one
    robots = robots.filter(robot => {
        if (robot.ip === ip) {
            // console.log(`Removing existing robot with IP: ${ip}`);
            if (robot.conv_image) robot.conv_image.destroy();
            if (robot.conv_toribe) robot.conv_toribe.destroy();
            if (robot.conv_line) robot.conv_line.destroy();
            if (robot.joint_circle) robot.joint_circle.destroy();
            if (robot.conv_circle) robot.conv_circle.destroy();
            return false; // Remove this robot
        }
        return true; // Keep other robots
    });

    const towingX = x * wtf_skala;
    const towingY = y * wtf_skala;

    // Create towing vehicle image
    const conv_image = new Konva.Image({
        x: towingX,
        y: towingY,
        image: towingimage,
        width: radius * 2 * wtf_skala,
        height: radius * 2 * wtf_skala,
        offsetX: radius * wtf_skala,
        offsetY: radius * wtf_skala,
        rotation: 90 + original_theta * -180 / Math.PI,
        scale: { x: imageScale, y: imageScale },
        draggable: false,
    });

    // Calculate joint position (hitch point)
    const jointDistance = radius * 1.2 * wtf_skala;
    const jointX = towingX - jointDistance * Math.cos(original_theta);
    const jointY = towingY + jointDistance * Math.sin(original_theta);

    // Create joint circle (red connection point)
    const joint_circle = new Konva.Circle({
        x: jointX,
        y: jointY,
        radius: radius * 0.3 * wtf_skala,
        fill: 'red',
        stroke: 'darkred',
        strokeWidth: 2,
    });

    // Initial trailer position (behind towing vehicle)
    const toribeDistance = radius * 0.7 * wtf_skala;
    const toribeX = jointX - toribeDistance * Math.cos(original_theta); // Berdasarkan joint
    const toribeY = jointY + toribeDistance * Math.sin(original_theta); // Berdasarkan joint

    // Create toribe image BERDASARKAN JOINT POSITION
    const conv_toribe = new Konva.Image({
        x: toribeX,      // PERBAIKAN: Gunakan toribeX yang dihitung dari joint
        y: toribeY,      // PERBAIKAN: Gunakan toribeY yang dihitung dari joint
        image: toribeimage,
        width: radius * 1.8 * wtf_skala,
        height: radius * 1.8 * wtf_skala,
        offsetX: radius * 0.9 * wtf_skala,
        offsetY: radius * 0.9 * wtf_skala,
        rotation: 90 + original_theta * -180 / Math.PI,
        scale: { x: imageScale * 0.9, y: imageScale * 0.9 },
        draggable: false,
    });

    // Create direction line for towing vehicle
    const conv_line = new Konva.Line({
        points: [
            towingX, towingY, 
            towingX + radius * wtf_skala * Math.cos(original_theta), 
            towingY - radius * wtf_skala * Math.sin(original_theta)
        ],
        stroke: 'cyan',
        strokeWidth: 5,
    });

    // Create robot object with physics tracking
    let robot_buffer = new Robot(ip, null, conv_line, conv_image, conv_toribe, original_x, original_y, original_theta, radius);
    robot_buffer.joint_circle = joint_circle;
    
    // Initialize trailer physics with normalized angles
    robot_buffer.toribe_x = (toribeX / wtf_skala) - stage.width() * 0.5 / wtf_skala; // Convert back to world coords
    robot_buffer.toribe_y = stage.height() * 0.5 / wtf_skala - (toribeY / wtf_skala); // Convert back to world coords
    robot_buffer.toribe_theta = normalizeAngle(original_theta);
    robot_buffer.prev_x = original_x;
    robot_buffer.prev_y = original_y;
    robot_buffer.prev_theta = normalizeAngle(original_theta);
    
    robots.push(robot_buffer);
    
    // Add all elements to layer (order matters for layering)
    robotLayer.add(conv_toribe);     // Toribe at bottom
    // robotLayer.add(joint_circle);    // Joint in middle
    robotLayer.add(conv_image);      // Towing vehicle on top
    robotLayer.add(conv_line);       // Direction line on top
    
    robotLayer.draw();
    
}

function addRobotImage(ip, x, y, theta, radius, imageScale) {
    const original_x = x;
    const original_y = y;
    const original_theta = theta;

    // Transform coordinates
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;

    // Check if robot already exists
    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            // Update existing robot image position and rotation
            robots[i].conv_image.position({
                x: x * wtf_skala,
                y: y * wtf_skala
            });
            robots[i].conv_image.rotation(90 + theta * -180 / Math.PI); // Convert radians to degrees
            robots[i].conv_line.points([
                x * wtf_skala, y * wtf_skala,
                x * wtf_skala + radius * wtf_skala * Math.cos(theta),
                y * wtf_skala - radius * wtf_skala * Math.sin(theta)
            ]);
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;

            robotLayer.batchDraw();
            return;
        }
    }

    const conv_image = new Konva.Image({
        x: x * wtf_skala,
        y: y * wtf_skala,
        image: towingimage,
        width: radius * 2 * wtf_skala,
        height: radius * 2 * wtf_skala,
        offsetX: radius * wtf_skala,
        offsetY: radius * wtf_skala,
        rotation: theta * 180 / Math.PI, // Convert radians to degrees for Konva
        scale: { x: imageScale, y: imageScale },
        draggable: false,
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


    // Create robot object and add to array
    let robot_buffer = new Robot(ip, null, conv_line, conv_image, x, y, theta, radius);
    robots.push(robot_buffer);

    robotLayer.add(robot_buffer.conv_image);
    robotLayer.add(robot_buffer.conv_line);
    robotLayer.draw();
}

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
const EMERGENCY_GANDENGAN_LEPAS = 0b100000000
const STATUS_TOWING_ACTIVE_AUTO = 0b01

const status_emergency = document.getElementById('status-emergency');
const terminal_terakhir = document.getElementById('terminal-terakhir');
const battery_soc = document.getElementById('battery-soc');
const counter_lap = document.getElementById('counter-lap');

let isTryingToConnect = false;

const alarm = new Audio('alarm1.wav');
alarm.loop = true;  // 🔁 Set looping once
let test_audio_play = 0

const alarm_40 = new Audio('alarm40.mp3');
alarm_40.loop = true;  // 🔁 Set looping onc

const alarm_30 = new Audio('alarm30.mp3');
alarm_30.loop = true;  // 🔁 Set looping once

const towingimage = new Image();
// towingimage.src = 'towingtopview.png';
towingimage.src = 'towing.png';

const toribeimage = new Image();
toribeimage.src = 'toribe.png';

// ================================================================

let robot_odometry_subscriber = null;
let robot_map_subscriber = null;
let robot_status_emergency_subscriber = null;
let robot_terminal_terakhir_subscriber = null;
let robot_wp_subscriber = null;
let robot_soc = null;
let robot_counter_lap = null;

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

    robot_counter_lap = new ROSLIB.Topic({
        ros: ros,
        name: '/master/counter_lap',
        messageType: 'std_msgs/Int32'
    });
    robot_counter_lap.subscribe(function (message) {
        const lap = message.data;
        counter_lap.innerHTML = `Counter Lap: ${lap}`;

        if (lap <= 15) {
            counter_lap.style.color = 'green';
        } else if (lap <= 30 && lap > 15) {
            counter_lap.style.color = 'blue';
        } else if (lap <= 45 && lap > 30) {
            counter_lap.style.color = 'orange';
        } else {
            counter_lap.style.color = 'red';
        }
    });

    robot_soc = new ROSLIB.Topic({
        ros: ros,
        name: '/master/battery_soc',
        messageType: 'std_msgs/Int16'
    });

    robot_soc.subscribe(function (message) {
        const soc = message.data;
        battery_soc.innerHTML = `Battery SOC: ${soc}%`;

        // Change color based on SOC value
        if (soc < 30) {
            battery_soc.style.color = 'red';
            if (alarm_30.paused) {
                alarm_30.play();
            }
        } else if (soc < 40) {
            battery_soc.style.color = 'orange';
            if (alarm_30.paused) {
                alarm_30.play();
            }
        } else {
            battery_soc.style.color = 'green';
            if (test_audio_play == 0) {
                if (!alarm_30.paused) {
                    alarm_30.pause();
                    alarm_30.currentTime = 0;
                }
                if (!alarm_40.paused) {
                    alarm_40.pause();
                    alarm_40.currentTime = 0;
                }
            }
        }
    });

    robot_wp_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/master/waypoints',
        messageType: 'sensor_msgs/PointCloud'
    });

    robot_wp_subscriber.subscribe(function (message) {
        const points = message.points;
        const waypoints = [];

        for (let i = 0; i < points.length; i++) {
            const x = points[i].x;
            const y = points[i].y;

            const x_tf = x + stage.width() * 0.5 / wtf_skala;
            const y_tf = stage.height() * 0.5 / wtf_skala - y;

            waypoints.push(x_tf * wtf_skala, y_tf * wtf_skala);
        }

        // Clear the shape layer
        waypointsLayer.destroyChildren();

        // Draw the waypoints
        const waypointsLine = new Konva.Line({
            points: waypoints,
            stroke: 'blue',
            strokeWidth: 5,
        });
        waypointsLayer.add(waypointsLine);

        waypointsLayer.draw();

        robot_wp_subscriber.unsubscribe(); // Unsubscribe after receiving the waypoints once
    }
    );

    // ===================================================

    robot_odometry_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/master/pose_filtered',
        messageType: 'nav_msgs/Odometry'
    });

    robot_odometry_subscriber.subscribe(function (message) {
        const x = message.pose.pose.position.x;
        const y = message.pose.pose.position.y;
        const q = message.pose.pose.orientation;
        const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const radius = 4.05;
        const offsetimage = 90;

        // addRobot("0.0.0.0", x, y, theta, radius, 'red');
        // addRobotImage("0.0.0.0", x, y, theta, radius, 1.5);
        addRobotImageWithToribe("0.0.0.0", x, y, theta, radius, 1.5);
    });

    // ===================================================

    // robot_map_subscriber = new ROSLIB.Topic({
    //     ros: ros,
    //     name: '/slam/map', // Change topic name as needed
    //     messageType: 'nav_msgs/OccupancyGrid'
    // });

    // // Subscribe to map messages and draw the map
    // robot_map_subscriber.subscribe(function (message) {
    //     let map = message.data;
    //     let width = message.info.width;
    //     let height = message.info.height;

    //     // Resize the canvas buffer
    //     mapCanvas.width = width;
    //     mapCanvas.height = height;

    //     // Draw the map to the canvas buffer
    //     let imageData = mapCtx.createImageData(width, height);
    //     for (let i = 0; i < map.length; i++) {
    //         let occupancy = map[i];

    //         let r, g, b;

    //         if (occupancy === -1) {
    //             // Unknown space -> Dark Green
    //             r = 71;
    //             g = 128;
    //             b = 118;
    //         } else if (occupancy === 0) {
    //             // Free space -> White
    //             r = 255;
    //             g = 255;
    //             b = 255;
    //         } else {
    //             // Occupied space -> Color gradient (Red to Orange)
    //             let t = occupancy / 100; // Normalize to 0-1

    //             // Interpolate red and green between red (255, 0, 0) and orange (255, 165, 0)
    //             r = 0; // Red is fixed
    //             g = Math.round(165 - t * 165); // From 165 to 0 (green scale)
    //             b = 0; // Blue stays 0 for red-to-orange color
    //         }

    //         // Assign colors to image data
    //         imageData.data[i * 4 + 0] = r;  // Red
    //         imageData.data[i * 4 + 1] = g;  // Green
    //         imageData.data[i * 4 + 2] = b;  // Blue
    //         imageData.data[i * 4 + 3] = 255; // Fully opaque
    //     }
    //     mapCtx.putImageData(imageData, 0, 0);

    //     let imageObj = new Image();
    //     imageObj.onload = function () {
    //         let mapImage = new Konva.Image({
    //             image: imageObj,
    //             width: width * wtf_skala * message.info.resolution,
    //             height: height * wtf_skala * message.info.resolution,
    //             x: (message.info.origin.position.x + stage.width() * 0.5 / wtf_skala) * wtf_skala,
    //             y: (stage.height() * 0.5 / wtf_skala - message.info.origin.position.y) * wtf_skala,
    //         });

    //         // Flip Y-axis if the map's origin is bottom-left
    //         mapImage.scaleY(-1);
    //         // mapImage.offsetY(height);

    //         // Clear and add the new map image
    //         mapLayer.destroyChildren();
    //         mapLayer.add(mapImage);
    //         mapLayer.draw();
    //     };
    //     imageObj.src = mapCanvas.toDataURL();

    //     console.log("Map received and drawn.");

    //     robot_map_subscriber.unsubscribe(); // Hanya kirim sekali saja
    // });

    // ===================================================

    robot_status_emergency_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: "/master/status_emergency",
        messageType: "std_msgs/Int16",
    });

    robot_status_emergency_subscriber.subscribe(function (message) {
        if ((message.data & STATUS_TOWING_ACTIVE_AUTO) == 0) {
            status_emergency.innerHTML = "Towing Mode Manual";
            if (!alarm.paused) {
                alarm.pause();
                alarm.currentTime = 0;
            }
            if (!alarm_30.paused) {
                alarm_30.pause();
                alarm_30.currentTime = 0;
            }
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
        else if ((message.data & EMERGENCY_GANDENGAN_LEPAS) == EMERGENCY_GANDENGAN_LEPAS) {
            status_emergency.innerHTML = "WARNING: GANDENGAN LEPAS";
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
                if (!alarm_30.paused) {
                    alarm_30.pause();
                    alarm_30.currentTime = 0;
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
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Area Jibcrane";
        }
        else if (terminal_terakhir_value == 26) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Tikungan samping lab";
        }
        else if (terminal_terakhir_value == 24) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalur 1 lurus depan yokai 1";
        }
        else if (terminal_terakhir_value == 1) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Tikungan samping yokai 1";
        }
        else if (terminal_terakhir_value == 3) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalur 1 lurus samping yokai 1i";
        }
        else if (terminal_terakhir_value == 5) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalur 1 tikungan bawah tangga";
        }
        else if (terminal_terakhir_value == 6) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalur 1 tikungan bawah tangga";
        }
        else if (terminal_terakhir_value == 11) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalur 1 lurus setelah bawah tangga";
        }
        else if (terminal_terakhir_value == 15) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalur 1 tikungan bea cukai";
        }
        else if (terminal_terakhir_value == 19) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Jalur 1 lurus setelah beacukai";
        }
        else if (terminal_terakhir_value == 23) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Degasing";
        }
        else if (terminal_terakhir_value == 46) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Degasing";
        }
        else if (terminal_terakhir_value == 7) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Degasing";
        }
        else if (terminal_terakhir_value == 25) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Degasing";
        }
        else if (terminal_terakhir_value == 47) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Berangkat) Degasing";
        }
        else if (terminal_terakhir_value == 35) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalur 2 lurus setelah bea cukai";
        }
        else if (terminal_terakhir_value == 37) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalur 2 tikungan bea cukai";
        }
        else if (terminal_terakhir_value == 38) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalur 2 lurus sebelum bea cukai";
        }
        else if (terminal_terakhir_value == 40) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalur 2 tikungan bawah tangga";
        }
        else if (terminal_terakhir_value == 41) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalur 2 lurus samping yokai 1";
        }
        else if (terminal_terakhir_value == 43) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalur 2 tikungan jembatan penyebrangan";
        }
        else if (terminal_terakhir_value == 48) {
            terminal_terakhir.innerHTML = "Terminal Terakhir: (Pulang) Jalur 2 tikungan jembatan penyebrangan";
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

    if (robot_wp_subscriber) {
        robot_wp_subscriber.unsubscribe();
        robot_wp_subscriber = null;
    }

    if (robot_soc) {
        robot_soc.unsubscribe();
        robot_soc = null;
    }

    if (robot_counter_lap) {
        robot_counter_lap.unsubscribe();
        robot_counter_lap = null;
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

