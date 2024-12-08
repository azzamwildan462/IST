// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

// Node configurations
const nodes = [
    {
        id: 'beckhoff',
        topic: '/beckhoff/error_code',
        messageType: 'std_msgs/Int16',
        sectionId: 'beckhoff-section',
        tableId: 'beckhoff-error-body',
        errorCodes: {
            0: 'No error',
            1: 'EtherCAT communication error',
            // Add more Beckhoff error codes and descriptions
        },
        errorSolutions: {
            0: 'No action needed',
            1: 'Reset PLC',
            // Add more Beckhoff error codes and solutions
        }
    },
    {
        id: 'camera',
        topic: '/cam_kanan/error_code',
        messageType: 'std_msgs/Int16',
        sectionId: 'camera-section',
        tableId: 'camera-error-body',
        errorCodes: {
            0: 'No error',
            1: 'Failed to set camera fourcc',
            2: 'Failed to set camera width',
            3: 'Failed to set camera height',
            4: 'Failed to set camera fps',
            11: 'Failed to capture image',
            // Add more Camera error codes and descriptions
        },
        errorSolutions: {
            0: 'No action needed',
            1: 'Check camera settings',
            2: 'Check camera settings',
            3: 'Check camera settings',
            4: 'Check camera settings',
            11: 'Check camera connection',
            // Add more Camera error codes and solutions
        }
    },
    {
        id: 'detection',
        topic: '/aruco_kanan/error_code',
        messageType: 'std_msgs/Int16',
        sectionId: 'detection-section',
        tableId: 'detection-error-body',
        errorCodes: {
            0: 'No error',
            1: 'Failed to load config',
            2: 'Failed to save config',
            3: 'Failed to convert image',
            4: 'Failed to clone image',
            5: 'No data',
            // Add more Detection error codes and descriptions
        },
        errorSolutions: {
            0: 'No action needed',
            1: 'Check config file',
            2: 'Check config file',
            3: 'Check image conversion',
            4: 'Check image cloning',
            5: 'Check data source',
            // Add more Detection error codes and solutions
        }
    },
    {
        id: 'lidar',
        topic: '/lidar/error_code',
        messageType: 'std_msgs/Int16',
        sectionId: 'lidar-section',
        tableId: 'lidar-error-body',
        errorCodes: {
            0: 'No error',
            1: 'Failed to connect to Lidar',
            2: 'Failed to start Lidar',
            3: 'Failed to stop Lidar',
            4: 'Failed to read Lidar',
            5: 'Failed to write Lidar',
            // Add more Lidar error codes and descriptions
        },
        errorSolutions: {
            0: 'No action needed',
            1: 'Check Lidar connection',
            2: 'Check Lidar connection',
            3: 'Check Lidar connection',
            4: 'Check Lidar connection',
            5: 'Check Lidar connection',
            // Add more Lidar error codes and solutions
        }
    },
    {
        id: 'pose-estimator',
        topic: '/slam/error_code',
        messageType: 'std_msgs/Int16',
        sectionId: 'pose-estimator-section',
        tableId: 'pose-estimator-error-body',
        errorCodes: {
            0: 'No error',
            1: 'Failed to load map',
            2: 'Failed to save map',
            3: 'Failed to localize',
            4: 'Failed to set pose',
            5: 'Failed to get pose',
            // Add more Pose Estimator error codes and descriptions
        },
        errorSolutions: {
            0: 'No action needed',
            1: 'Check map file',
            2: 'Check map file',
            3: 'Check localization',
            4: 'Check pose setting',
            5: 'Check pose retrieval',
            // Add more Pose Estimator error codes and solutions
        }
    },
    {
        id: 'obstacle-filter',
        topic: '/obstacle_filter/error_code',
        messageType: 'std_msgs/Int16',
        sectionId: 'obstacle-filter-section',
        tableId: 'obstacle-filter-error-body',
        errorCodes: {
            0: 'No error',
            1: 'Failed to set filter',
            2: 'Failed to apply filter',
            3: 'Failed to get filtered data',
            // Add more Obstacle Filter error codes and descriptions
        },
        errorSolutions: {
            0: 'No action needed',
            1: 'Check filter settings',
            2: 'Check filter application',
            3: 'Check filtered data retrieval',
            // Add more Obstacle Filter error codes and solutions
        }
    }
    // Add more nodes as needed
];

// Connect to ROS
const ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090"
});

ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
    initializeNodes();
});

ros.on("error", function (error) {
    console.log("Error connecting to WebSocket server:", error);
});

ros.on("close", function () {
    console.log("Connection to WebSocket server closed.");
});

// Initialize nodes and their subscribers
function initializeNodes() {
    nodes.forEach(node => {
        // Check if topic exists
        const topic = new ROSLIB.Topic({
            ros: ros,
            name: node.topic,
            messageType: node.messageType
        });

        topic.subscribe(() => {
            subscribeToTopic(node);
            document.getElementById(node.sectionId).style.display = 'block';
        });
    });
}

// Subscribe to a node's topic
function subscribeToTopic(node) {
    const listener = new ROSLIB.Topic({
        ros: ros,
        name: node.topic,
        messageType: node.messageType
    });

    listener.subscribe((message) => {
        try {
            const errorCode = message.data;
            updateErrorTable(node.tableId, errorCode, node.errorCodes, node.errorSolutions);
        } catch (e) {
            console.error(`Error parsing message for ${node.id}:`, e);
        }
    });
}

// Create table row
function createErrorRow(errorCode, errorCodes, errorSolutions) {
    const tr = document.createElement('tr');
    
    const errorTd = document.createElement('td');
    const errorInput = document.createElement('input');
    errorInput.className = 'input';
    errorInput.type = 'text';
    errorInput.value = errorCodes[errorCode] || errorCode;
    errorInput.readOnly = true;
    errorTd.appendChild(errorInput);

    const solutionTd = document.createElement('td');
    const solutionInput = document.createElement('input');
    solutionInput.className = 'input';
    solutionInput.type = 'text';
    solutionInput.value = errorSolutions[errorCode] || 'Solusi tidak ditemukan';
    solutionInput.readOnly = true;
    solutionTd.appendChild(solutionInput);

    tr.appendChild(errorTd);
    tr.appendChild(solutionTd);
    
    // Add animation to new rows
    anime({
        targets: tr,
        opacity: [0, 1],
        translateY: [20, 0],
        duration: 500,
        easing: 'easeOutQuad'
    });

    return tr;
}

// Update error table
let currentErrorCodes = {};

function updateErrorTable(tableId, errorCode, nodeErrorCodes, nodeErrorSolutions) {
    const tableBody = document.getElementById(tableId);
    if (!tableBody) return;

    if (currentErrorCodes[tableId] !== errorCode) {
        currentErrorCodes[tableId] = errorCode;
        tableBody.innerHTML = '';
        const row = createErrorRow(errorCode, nodeErrorCodes, nodeErrorSolutions);
        tableBody.appendChild(row);
    }
}

// For testing purposes

setTimeout(() => {
    // updateErrorTable('beckhoff-error-body', 0, nodes[0].errorCodes, nodes[0].errorSolutions);
    // document.getElementById('beckhoff-section').style.display = 'block';

    // updateErrorTable('camera-error-body', 1, nodes[1].errorCodes, nodes[1].errorSolutions);
    // document.getElementById('camera-section').style.display = 'block';
}, 1000);
