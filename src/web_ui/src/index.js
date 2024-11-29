// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://localhost:9090",
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

// Create a ROSLIB Topic to subscribe to the 'chatter' topic
var listener = new ROSLIB.Topic({
    ros: ros,
    name: "/chatter",
    messageType: "std_msgs/String",
});

listener.subscribe(function (message) {
    console.log("Received message:", message.data);
});