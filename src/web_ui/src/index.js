// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

let velocity_actuation = 0;
let velocity_feedback = 0;
let steering_actuation = 0;
let steering_feedback = 0;
let baterai_value = 45;
let global_fsm_value = 0;
let master_fsm_value = 0;

// ================================================================

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

var sub_master2ui = new ROSLIB.Topic({
    ros: ros,
    name: "/master/to_ui",
    messageType: "std_msgs/Float32MultiArray",
});

sub_master2ui.subscribe(function (message) {
    velocity_actuation = message.data[0];
    velocity_feedback = message.data[1];
    steering_actuation = message.data[2];
    steering_feedback = message.data[3];
});

var sub_master_global_fsm = new ROSLIB.Topic({
    ros: ros,
    name: "/master/global_fsm",
    messageType: "std_msgs/Int16",
});

sub_master_global_fsm.subscribe(function (message) {
    global_fsm_value = message.data;
});

var sub_master_local_fsm = new ROSLIB.Topic({
    ros: ros,
    name: "/master/local_fsm",
    messageType: "std_msgs/Int16",
});

sub_master_local_fsm.subscribe(function (message) {
    master_fsm_value = message.data;
});


// ================================================================

function set_velocity(selector1, selector2, id_text, value1, value2) {

    if (value1 > 100) value1 = 100;
    if (value2 > 100) value2 = 100;

    const new_value1 = value1 / 100 * 75;
    const new_value2 = value2 / 100 * 75;

    const progressText = document.getElementById(id_text);
    var circle = document.querySelector(selector1);
    var circle2 = document.querySelector(selector2);
    var length = circle.getTotalLength(); // Get the total length of the circle's path
    var length2 = circle2.getTotalLength();

    circle.style.strokeDasharray = length; // Set the stroke dasharray to the total length of the circle
    circle.style.strokeDashoffset = length - (length * (new_value1 / 100)); // Initially hide the stroke

    circle2.style.strokeDasharray = length2; // Set the stroke dasharray to the total length of the circle
    circle2.style.strokeDashoffset = length2 - (length2 * (new_value2 / 100)); // Initially hide the stroke

    value1_display = value1 * 0.1;
    progressText.textContent = `${value1_display.toFixed(2)}`;
}

function set_steering(selector1, selector2, id_text, value1, value2) {

    value1 = value1 * -1;
    value2 = value2 * -1;


    if (value1 > 100) value1 = 100;
    if (value2 > 100) value2 = 100;

    const new_value1 = value1 / 100 * 75;
    const new_value2 = value2 / 100 * 75;

    const progressText = document.getElementById(id_text);
    var circle = document.querySelector(selector1);
    var circle2 = document.querySelector(selector2);
    var length = circle.getTotalLength(); // Get the total length of the circle's path
    var length2 = circle2.getTotalLength();

    circle.style.strokeDasharray = length; // Set the stroke dasharray to the total length of the circle
    circle.style.strokeDashoffset = length - (length * (new_value1 / 100)); // Initially hide the stroke

    circle2.style.strokeDasharray = length2; // Set the stroke dasharray to the total length of the circle
    circle2.style.strokeDashoffset = length2 - (length2 * (new_value2 / 100)); // Initially hide the stroke

    circle.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle.style.transformOrigin = "50% 50%"; // Ensure rotation is centered
    circle2.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle2.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    progressText.textContent = `${value1.toFixed(0)}%`;
}

function control_display_fsm() {
    if (global_fsm_value == 0) {
        global_fsm.innerHTML = ": INIT"
    }
    else if (global_fsm_value == 1) {
        global_fsm.innerHTML = ": Pre-Operation"
    }
    else if (global_fsm_value == 2) {
        global_fsm.innerHTML = ": Safe-Operation"
    }
    else if (global_fsm_value == 3) {
        global_fsm.innerHTML = ": Operation"
    }


    if (master_fsm_value == 0) {
        master_fsm.innerHTML = ": Pre-Follow lane"
    }
    else if (master_fsm_value == 1) {
        master_fsm.innerHTML = ": Follow Lane"
    }
    else if (master_fsm_value == 2) {
        master_fsm.innerHTML = ": Menunggu di Station 1"
    }
    else if (master_fsm_value == 3) {
        master_fsm.innerHTML = ": Menunggu di Station 2"
    }
}

// ================================================================

const velocity_kmph = document.getElementById('velocity-kmph');
const steering_rad = document.getElementById('steering-rad');
const batera_dom = document.getElementById('baterai-dom');
const global_fsm = document.getElementById('global-fsm');
const master_fsm = document.getElementById('master-fsm');

setInterval(() => {
    let velocity_actuation_display = velocity_actuation * 10;
    let velocity_feedback_display = velocity_feedback * 10;
    let steering_actuation_display = (steering_actuation) * 100 / 3.14;
    let steering_feedback_display = (steering_feedback) * 100 / 3.14;

    set_velocity('.velocity-circle1', '.velocity-circle2', 'velocity-text', velocity_actuation_display, velocity_feedback_display);
    velocity_kmph.textContent = `${velocity_feedback.toFixed(2)} km/h`;
    set_steering('.steering-circle1', '.steering-circle2', 'steering-text', steering_actuation_display, steering_feedback_display);
    steering_rad.textContent = `${steering_feedback.toFixed(2)} rad`;

    if (baterai_value > 50) {
        batera_dom.className = "progress is-success"
    }
    else if (baterai_value > 40) {
        batera_dom.className = "progress is-warning"
    }
    else {
        batera_dom.className = "progress is-danger"
    }
    batera_dom.value = baterai_value;

    control_display_fsm();

}, 50);
