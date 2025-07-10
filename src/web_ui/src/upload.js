// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

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

document.addEventListener('DOMContentLoaded', () => {
    const HOST = window.location.hostname;
    const PORT = 6273;

    // Point the forms at Flask endpoints
    document.getElementById('formA').action = `http://${HOST}:${PORT}/uploadA`;
    document.getElementById('formB').action = `http://${HOST}:${PORT}/uploadB`;

    document.querySelectorAll('.file-input').forEach((input) => {
        const nameSpan = input.closest('.file').querySelector('.file-name');
        input.addEventListener('change', () => {
            if (input.files.length > 0) {
                nameSpan.textContent = input.files[0].name;
            } else {
                nameSpan.textContent = 'No file selected';
            }
        });
    });
});
