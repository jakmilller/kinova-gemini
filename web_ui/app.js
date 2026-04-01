// Connect to ROS bridge
const ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

// DOM Elements
const statusEl = document.getElementById('connection-status');
const logConsole = document.getElementById('log-console');
const commandForm = document.getElementById('command-form');
const commandInput = document.getElementById('command-input');
const estopBtn = document.getElementById('estop-btn');

const statePosition = document.getElementById('state-position');
const stateOrientation = document.getElementById('state-orientation');
const stateJoints = document.getElementById('state-joints');
const stateGripper = document.getElementById('state-gripper');

const performanceBox = document.getElementById('performance-box');
const completionTimeEl = document.getElementById('completion-time');

let lastCommandStartTime = 0;

// Connection Callbacks
ros.on('connection', function() {
    statusEl.textContent = 'Connected';
    statusEl.classList.remove('bg-red-100', 'text-red-600');
    statusEl.classList.add('bg-green-100', 'text-green-600');
    addLog('System', 'Connected to ROS 2 bridge.');
});

ros.on('error', function(error) {
    statusEl.textContent = 'Error';
    statusEl.classList.remove('bg-green-100', 'text-green-600');
    statusEl.classList.add('bg-red-100', 'text-red-600');
    addLog('Error', 'WebSocket connection error.');
});

ros.on('close', function() {
    statusEl.textContent = 'Disconnected';
    statusEl.classList.remove('bg-green-100', 'text-green-600');
    statusEl.classList.add('bg-red-100', 'text-red-600');
    addLog('System', 'Disconnected from ROS 2.');
});

// Publishers
const instructionsTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/user_instructions',
    messageType : 'std_msgs/msg/String'
});

const voiceTriggerTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/voice_trigger',
    messageType : 'std_msgs/msg/Bool'
});

// Subscribers
const brainStatusListener = new ROSLIB.Topic({
    ros : ros,
    name : '/brain_status',
    messageType : 'std_msgs/msg/String'
});

const voiceStatusListener = new ROSLIB.Topic({
    ros : ros,
    name : '/voice_status',
    messageType : 'std_msgs/msg/String'
});

const voiceStatusIndicator = document.getElementById('voice-status-indicator');
const voiceStatusText = document.getElementById('voice-status-text');
const voiceBtn = document.getElementById('voice-btn');
let isRecording = false;

voiceStatusListener.subscribe(function(message) {
    const status = message.data;
    if (status === 'IDLE') {
        voiceStatusIndicator.classList.add('hidden');
        voiceBtn.classList.remove('text-red-600', 'animate-pulse');
        voiceBtn.classList.add('text-gray-400');
        isRecording = false;
    } else {
        voiceStatusIndicator.classList.remove('hidden');
        voiceStatusText.textContent = `Voice: ${status}`;
        
        if (status === 'LISTENING') {
            voiceBtn.classList.add('text-red-600', 'animate-pulse');
            voiceBtn.classList.remove('text-gray-400');
            isRecording = true;
        } else if (status === 'PROCESSING') {
            voiceBtn.classList.remove('text-red-600', 'animate-pulse');
            voiceBtn.classList.add('text-blue-600');
        }
    }
});

voiceBtn.addEventListener('click', function() {
    isRecording = !isRecording;
    const msg = new ROSLIB.Message({
        data: isRecording
    });
    voiceTriggerTopic.publish(msg);
    
    if (isRecording) {
        addLog('System', 'Voice command triggered...');
    }
});

// Spacebar Press-and-Hold Logic
let spacebarPressed = false;

window.addEventListener('keydown', function(e) {
    // Only trigger if spacebar is pressed AND we are not already recording
    // AND the user is not currently typing in the command input field
    if (e.code === 'Space' && !spacebarPressed && document.activeElement !== commandInput) {
        e.preventDefault(); // Prevent scrolling down the page
        spacebarPressed = true;
        
        const msg = new ROSLIB.Message({
            data: true
        });
        voiceTriggerTopic.publish(msg);
        addLog('System', 'Voice recording started (Spacebar held)...');
    }
});

window.addEventListener('keyup', function(e) {
    if (e.code === 'Space' && spacebarPressed) {
        spacebarPressed = false;
        
        const msg = new ROSLIB.Message({
            data: false
        });
        voiceTriggerTopic.publish(msg);
        addLog('System', 'Voice recording stopped (Spacebar released).');
    }
});

brainStatusListener.subscribe(function(message) {
    const data = message.data;
    addLog('Brain', data);

    // Check for latency report (time to start of move)
    if (data.includes('LATENCY:')) {
        const timeMatch = data.match(/LATENCY: ([\d.]+)s/);
        if (timeMatch) {
            performanceBox.classList.remove('hidden');
            completionTimeEl.textContent = timeMatch[1] + 's';
        }
    }
});

const robotStateListener = new ROSLIB.Topic({
    ros : ros,
    name : '/robot_state',
    messageType : 'ros2_interfaces/msg/RobotState'
});

robotStateListener.subscribe(function(message) {
    statePosition.textContent = `${message.x.toFixed(3)}, ${message.y.toFixed(3)}, ${message.z.toFixed(3)}`;
    stateOrientation.textContent = `${message.theta_x.toFixed(3)}, ${message.theta_y.toFixed(3)}, ${message.theta_z.toFixed(3)}`;
    
    const gripperVal = message.gripper_position.toFixed(1);
    stateGripper.textContent = `${gripperVal}%`;
    const gripperBar = document.getElementById('gripper-bar');
    if (gripperBar) {
        gripperBar.style.width = `${gripperVal}%`;
    }
    
    if (message.joint_angles && message.joint_angles.length >= 7) {
        stateJoints.innerHTML = message.joint_angles.map((j, i) => `
            <div class="flex justify-between border-b border-gray-50 pb-1">
                <span class="text-gray-400 font-bold uppercase">J${i+1}</span>
                <span class="font-bold text-gray-900">${j.toFixed(1)}°</span>
            </div>
        `).join('');
    }
});

// Handle Command Submission
commandForm.addEventListener('submit', function(e) {
    e.preventDefault();
    const cmd = commandInput.value.trim();
    if (cmd) {
        const msg = new ROSLIB.Message({
            data: cmd
        });
        instructionsTopic.publish(msg);
        addLog('User', cmd);
        commandInput.value = '';
        
        // Reset timer display for new command
        performanceBox.classList.remove('hidden');
        completionTimeEl.textContent = '...';
    }
});

// Handle E-Stop
estopBtn.addEventListener('click', function() {
    const msg = new ROSLIB.Message({
        data: 'Stop immediately!'
    });
    instructionsTopic.publish(msg); // Both brain and controller listen to this for stop
    addLog('E-STOP', 'Emergency Stop Issued!', true);
    
    // Visual feedback for e-stop
    estopBtn.classList.add('bg-red-800');
    setTimeout(() => estopBtn.classList.remove('bg-red-800'), 500);
});

// Helper: Add log to console
function addLog(sender, message, isError = false) {
    const logEl = document.createElement('div');
    const time = new Date().toLocaleTimeString([], { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
    
    let colorClass = 'text-green-400';
    if (sender === 'User') colorClass = 'text-blue-400';
    if (sender === 'Brain') colorClass = 'text-purple-400';
    if (isError || sender === 'E-STOP' || sender === 'Error' || message.includes('FAILED')) colorClass = 'text-red-500 font-bold';
    if (message.includes('SUCCESS')) colorClass = 'text-green-300 font-bold';

    logEl.innerHTML = `<span class="text-gray-600 font-normal mr-1">${time}</span><span class="${colorClass} mr-2">[${sender}]</span><span class="text-gray-100">${message}</span>`;
    logConsole.appendChild(logEl);
    
    // Auto-scroll to bottom
    logConsole.scrollTop = logConsole.scrollHeight;
}
