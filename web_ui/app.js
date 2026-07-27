// Connect to ROS bridge
const ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

// DOM Elements
const statusEl = document.getElementById('connection-status');
const chatContainer = document.getElementById('chat-container');
const commandForm = document.getElementById('command-form');
const commandInput = document.getElementById('command-input');
const estopBtn = document.getElementById('estop-btn');

const statePosition = document.getElementById('state-position');
const stateOrientation = document.getElementById('state-orientation');
const stateJoints = document.getElementById('state-joints');
const stateGripper = document.getElementById('state-gripper');

let lastJointAngles = null;

// Connection Callbacks
ros.on('connection', function() {
    statusEl.textContent = 'Connected';
    statusEl.classList.remove('bg-red-100', 'text-red-600');
    statusEl.classList.add('bg-green-100', 'text-green-600');
    addChatMessage('system', 'Connected to ROS 2 bridge.');
});

ros.on('error', function(error) {
    statusEl.textContent = 'Error';
    statusEl.classList.remove('bg-green-100', 'text-green-600');
    statusEl.classList.add('bg-red-100', 'text-red-600');
    addChatMessage('system', 'WebSocket connection error.');
});

ros.on('close', function() {
    statusEl.textContent = 'Disconnected';
    statusEl.classList.remove('bg-green-100', 'text-green-600');
    statusEl.classList.add('bg-red-100', 'text-red-600');
    addChatMessage('system', 'Disconnected from ROS 2.');
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
const geminiChatListener = new ROSLIB.Topic({
    ros : ros,
    name : '/gemini_chat',
    messageType : 'std_msgs/msg/String'
});

const voiceStatusListener = new ROSLIB.Topic({
    ros : ros,
    name : '/voice_status',
    messageType : 'std_msgs/msg/String'
});

const voiceStatusIndicator = document.getElementById('voice-status-indicator');
const voiceStatusText = document.getElementById('voice-status-text');
const voiceStatusDot = document.getElementById('voice-status-dot');
const voiceBtn = document.getElementById('voice-btn');

const VOICE_UI = {
    IDLE:       { label: 'Hold to talk',   dot: 'bg-gray-400',               btn: 'text-gray-400' },
    LISTENING:  { label: 'Listening…',     dot: 'bg-red-500 animate-pulse',  btn: 'text-red-600' },
    PROCESSING: { label: 'Transcribing…',  dot: 'bg-blue-500 animate-pulse', btn: 'text-blue-600' },
};
const VOICE_BTN_COLORS = ['text-red-600', 'text-blue-600', 'text-gray-400'];

if (voiceStatusListener && voiceStatusIndicator && voiceStatusText && voiceBtn) {
    voiceStatusListener.subscribe(function(message) {
        const ui = VOICE_UI[message.data];
        if (!ui) return;

        voiceStatusIndicator.classList.remove('hidden');
        voiceStatusText.textContent = ui.label;
        if (voiceStatusDot) {
            voiceStatusDot.className = `inline-block w-2 h-2 rounded-full mr-2 ${ui.dot}`;
        }
        voiceBtn.classList.remove(...VOICE_BTN_COLORS, 'animate-pulse');
        voiceBtn.classList.add(ui.btn);
    });

    // Press-and-hold: True on press starts recording, False on release sends it for
    // transcription — mirroring the physical Arduino button. Track state so a stray
    // event (e.g. mouseleave after mouseup) can't publish a duplicate.
    let holding = false;
    const startTalk = function(e) {
        if (e) e.preventDefault();
        if (holding) return;
        holding = true;
        voiceTriggerTopic.publish(new ROSLIB.Message({ data: true }));
    };
    const stopTalk = function(e) {
        if (e) e.preventDefault();
        if (!holding) return;
        holding = false;
        voiceTriggerTopic.publish(new ROSLIB.Message({ data: false }));
    };

    voiceBtn.title = 'Hold to talk';
    voiceBtn.addEventListener('mousedown', startTalk);
    voiceBtn.addEventListener('mouseup', stopTalk);
    voiceBtn.addEventListener('mouseleave', stopTalk);  // pointer slid off while held
    voiceBtn.addEventListener('touchstart', startTalk);
    voiceBtn.addEventListener('touchend', stopTalk);
    voiceBtn.addEventListener('touchcancel', stopTalk);
}

geminiChatListener.subscribe(function(message) {
    try {
        const chatData = JSON.parse(message.data);
        addChatMessage(chatData.role, chatData.text, chatData.image, chatData.source);
    } catch (e) {
        console.error("Failed to parse chat message:", e);
    }
});

const robotStateListener = new ROSLIB.Topic({
    ros : ros,
    name : '/robot_state',
    messageType : 'ros2_interfaces/msg/RobotState'
});

robotStateListener.subscribe(function(message) {
    if (statePosition) {
        statePosition.textContent = `${message.x.toFixed(3)}, ${message.y.toFixed(3)}, ${message.z.toFixed(3)}`;
    }
    if (stateOrientation) {
        stateOrientation.textContent = `${message.theta_x.toFixed(3)}, ${message.theta_y.toFixed(3)}, ${message.theta_z.toFixed(3)}`;
    }
    
    if (stateGripper) {
        const gripperVal = message.gripper_position.toFixed(1);
        stateGripper.textContent = `${gripperVal}%`;
        const gripperBar = document.getElementById('gripper-bar');
        if (gripperBar) {
            gripperBar.style.width = `${gripperVal}%`;
        }
    }
    
    if (stateJoints && message.joint_angles && message.joint_angles.length >= 7) {
        stateJoints.innerHTML = message.joint_angles.map((j, i) => `
            <div class="flex justify-between border-b border-gray-50 pb-1">
                <span class="text-gray-400 font-bold uppercase">J${i+1}</span>
                <span class="font-bold text-gray-900">${j.toFixed(1)}°</span>
            </div>
        `).join('');
    }

    // Dynamic Client-side Movement State Estimation
    let isMoving = false;
    if (lastJointAngles && message.joint_angles) {
        const diffs = message.joint_angles.map((angle, i) => Math.abs(angle - lastJointAngles[i]));
        isMoving = diffs.some(diff => diff > 0.05);
    }
    lastJointAngles = message.joint_angles;

    const movingEl = document.getElementById('state-moving');
    if (movingEl) {
        if (isMoving) {
            movingEl.textContent = 'Moving';
            movingEl.className = 'px-3 py-1 rounded font-black uppercase text-[10px] tracking-wider bg-yellow-100 text-yellow-800 animate-pulse border border-yellow-200';
        } else {
            movingEl.textContent = 'Stationary';
            movingEl.className = 'px-3 py-1 rounded font-black uppercase text-[10px] tracking-wider bg-green-100 text-green-800 border border-green-200';
        }
    }
});

// Handle Command Submission
if (commandForm && commandInput) {
    commandForm.addEventListener('submit', function(e) {
        e.preventDefault();
        const cmd = commandInput.value.trim();
        if (cmd) {
            const msg = new ROSLIB.Message({
                data: cmd
            });
            instructionsTopic.publish(msg);
            commandInput.value = '';
        }
    });
}

// Handle E-Stop
if (estopBtn) {
    estopBtn.addEventListener('click', function() {
        const msg = new ROSLIB.Message({
            data: 'Stop immediately!'
        });
        instructionsTopic.publish(msg); // Both brain and controller listen to this for stop
        addChatMessage('system', 'EMERGENCY STOP ISSUED!');
        
        // Visual feedback for e-stop
        estopBtn.classList.add('bg-red-800');
        setTimeout(() => estopBtn.classList.remove('bg-red-800'), 500);
    });
}

// Helper: Add message to chat container
function escapeHtml(s) {
    const div = document.createElement('div');
    div.textContent = s == null ? '' : String(s);
    return div.innerHTML;
}

function addChatMessage(role, text, image = null, source = null) {
    if (!chatContainer) return;

    const messageEl = document.createElement('div');
    messageEl.classList.add('message', `message-${role}`);

    // Mark messages that came from speech, to distinguish them from typed ones.
    const micIcon = source === 'voice'
        ? ' <i class="fas fa-microphone ml-1" title="Spoken"></i>'
        : '';
    let contentHtml = `<div class="text-xs font-bold opacity-70 mb-1 uppercase tracking-tighter">${escapeHtml(role)}${micIcon}</div>`;

    // Use whitespace-pre-wrap to preserve newlines and indentation in reports
    contentHtml += `<div class="whitespace-pre-wrap font-mono mt-1 leading-relaxed">${escapeHtml(text)}</div>`;

    if (image) {
        contentHtml += `<img src="${image}" class="message-image mt-2 rounded border border-black/10" alt="Scene Analysis">`;
    }

    messageEl.innerHTML = contentHtml;
    chatContainer.appendChild(messageEl);

    // Auto-scroll to bottom
    chatContainer.scrollTop = chatContainer.scrollHeight;
}
