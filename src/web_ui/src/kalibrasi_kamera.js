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

// DOM 
const frame_kamera = document.getElementById("frame_kamera");
frame_kamera.src = "http://" + window.location.hostname + ":8080/stream?topic=/forklift_detector_vision/frame_display&type=ros_compressed&quality=30"

const frame_binary = document.getElementById("frame_binary");
frame_binary.src = "http://" + window.location.hostname + ":8080/stream?topic=/forklift_detector_vision/frame_binary&type=ros_compressed&quality=30"

const frame_aruco = document.getElementById("frame_aruco");
frame_aruco.src = "http://" + window.location.hostname + ":8080/stream?topic=/forklift_detector_vision/frame_display&type=ros_compressed&quality=30"

const low_h_slider = document.getElementById("low_h_slider");
low_h_slider.value = 0;
const low_h_value = document.getElementById("low_h_value");
low_h_value.innerHTML = low_h_slider.value;
function update_low_h(new_value) {
    send_params();
    low_h_value.innerHTML = new_value;
}

const high_h_slider = document.getElementById("high_h_slider");
high_h_slider.value = 0;
const high_h_value = document.getElementById("high_h_value");
high_h_value.innerHTML = high_h_slider.value;
function update_high_h(new_value) {
    send_params();
    high_h_value.innerHTML = new_value;
}

const low_l_slider = document.getElementById("low_l_slider");
low_l_slider.value = 0;
const low_l_value = document.getElementById("low_l_value");
low_l_value.innerHTML = low_l_slider.value;
function update_low_l(new_value) {
    send_params();
    low_l_value.innerHTML = new_value;
}

const high_l_slider = document.getElementById("high_l_slider");
high_l_slider.value = 0;
const high_l_value = document.getElementById("high_l_value");
high_l_value.innerHTML = high_l_slider.value;
function update_high_l(new_value) {
    send_params();
    high_l_value.innerHTML = new_value;
}

const low_s_slider = document.getElementById("low_s_slider");
low_s_slider.value = 0;
const low_s_value = document.getElementById("low_s_value");
low_s_value.innerHTML = low_s_slider.value;
function update_low_s(new_value) {
    send_params();
    low_s_value.innerHTML = new_value;
}

const high_s_slider = document.getElementById("high_s_slider");
high_s_slider.value = 0;
const high_s_value = document.getElementById("high_s_value");
high_s_value.innerHTML = high_s_slider.value;
function update_high_s(new_value) {
    send_params();
    high_s_value.innerHTML = new_value;
}

// ================================================================

function send_params() {
    req_params = new ROSLIB.ServiceRequest({
        req_type: 1,
        req_data: [
            parseInt(low_h_slider.value),
            parseInt(high_h_slider.value),
            parseInt(low_l_slider.value),
            parseInt(high_l_slider.value),
            parseInt(low_s_slider.value),
            parseInt(high_s_slider.value),
        ],
    });

    ser_params.callService(req_params, (result) => {
    });
}

// ================================================================

req_params = new ROSLIB.ServiceRequest({
    req_type: 0
});

ser_params = new ROSLIB.Service({
    ros: ros,
    name: "/forklift_detector_vision/params",
    serviceType: "ros2_interface/srv/Params",
});

ser_params.callService(req_params, (result) => {
    low_h_slider.value = result.data[0];
    high_h_slider.value = result.data[1];
    low_l_slider.value = result.data[2];
    high_l_slider.value = result.data[3];
    low_s_slider.value = result.data[4];
    high_s_slider.value = result.data[5];

    low_h_value.innerHTML = low_h_slider.value;
    high_h_value.innerHTML = high_h_slider.value;
    low_l_value.innerHTML = low_l_slider.value;
    high_l_value.innerHTML = high_l_slider.value;
    low_s_value.innerHTML = low_s_slider.value;
    high_s_value.innerHTML = high_s_slider.value;
});