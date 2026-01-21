/**
 * Isaac Sim HMI Dashboard - JavaScript Controller
 * Handles WebSocket communication, UI updates, and robot control
 */

// ============== Configuration ==============
const CONFIG = {
    POLLING_INTERVAL: 100,  // ms for fallback polling
    RECONNECT_DELAY: 3000,   // ms before reconnection attempt
    NOTIFICATION_DURATION: 3000,  // ms to show notifications
    DECIMAL_PLACES: 4,
    JOINT_OFFSET_STEP: 0.1,  // radians
    POSITION_OFFSET_STEP: 0.01,  // meters
};

// ============== State Management ==============
const state = {
    socket: null,
    connected: false,
    selectedRobot: null,
    robots: [],
    robotStates: {},
    autoRefresh: true,
    lastUpdateTime: null,
    updateCount: 0,
    updateRateInterval: null,
};

// ============== Initialization ==============
document.addEventListener('DOMContentLoaded', () => {
    initializeWebSocket();
    setupEventListeners();
    startUpdateRateMonitor();
});

// ============== WebSocket Management ==============
function initializeWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}`;
    
    state.socket = io(wsUrl, {
        transports: ['websocket', 'polling'],
        reconnection: true,
        reconnectionDelay: CONFIG.RECONNECT_DELAY,
    });

    // Connection events
    state.socket.on('connect', () => {
        console.log('WebSocket connected');
        state.connected = true;
        updateConnectionStatus(true);
        showNotification('Connected to server', 'success');
    });

    state.socket.on('disconnect', () => {
        console.log('WebSocket disconnected');
        state.connected = false;
        updateConnectionStatus(false);
        showNotification('Disconnected from server', 'error');
    });

    state.socket.on('connect_error', (error) => {
        console.error('Connection error:', error);
        updateConnectionStatus(false);
    });

    // Robot events
    state.socket.on('robot_list', (data) => {
        console.log('Received robot list:', data);
        state.robots = data.robots || [];
        updateRobotList();
    });

    state.socket.on('robot_state', (data) => {
        if (state.autoRefresh) {
            handleRobotStateUpdate(data);
        }
    });

    state.socket.on('command_result', (data) => {
        const status = data.success ? 'success' : 'error';
        const message = data.success 
            ? `${data.type} sent successfully` 
            : `Failed to send ${data.type}`;
        showNotification(message, status);
    });
}

function updateConnectionStatus(connected) {
    const statusEl = document.getElementById('connectionStatus');
    const dotEl = statusEl.querySelector('.status-dot');
    const textEl = statusEl.querySelector('.status-text');
    
    if (connected) {
        dotEl.classList.remove('disconnected');
        dotEl.classList.add('connected');
        textEl.textContent = 'Connected';
    } else {
        dotEl.classList.remove('connected');
        dotEl.classList.add('disconnected');
        textEl.textContent = 'Disconnected';
    }
}

// ============== Event Listeners Setup ==============
function setupEventListeners() {
    // Robot management
    document.getElementById('scanRobotsBtn').addEventListener('click', scanForRobots);
    document.getElementById('addRobotBtn').addEventListener('click', addRobot);
    document.getElementById('newRobotNamespace').addEventListener('keypress', (e) => {
        if (e.key === 'Enter') addRobot();
    });

    // Joint control
    document.getElementById('sendJointCmd').addEventListener('click', sendJointCommand);
    document.getElementById('syncFromFeedback').addEventListener('click', syncJointsFromFeedback);
    
    // Setup joint sliders
    document.querySelectorAll('.joint-slider-group').forEach(group => {
        const slider = group.querySelector('.joint-slider');
        const valueInput = group.querySelector('.joint-value');
        
        // Sync slider and input
        slider.addEventListener('input', () => {
            valueInput.value = parseFloat(slider.value).toFixed(2);
        });
        
        valueInput.addEventListener('change', () => {
            slider.value = valueInput.value;
        });
        
        // Offset buttons
        group.querySelectorAll('.btn-offset').forEach(btn => {
            btn.addEventListener('click', () => {
                const offset = parseFloat(btn.dataset.offset);
                const newValue = parseFloat(slider.value) + offset;
                slider.value = newValue;
                valueInput.value = newValue.toFixed(2);
            });
        });
    });

    // Cartesian control
    document.getElementById('sendCartesianCmd').addEventListener('click', sendCartesianCommand);
    document.getElementById('syncCartesianFromFeedback').addEventListener('click', syncCartesianFromFeedback);
    
    // Cartesian offset buttons (inline)
    document.querySelectorAll('.input-with-offset .btn-offset').forEach(btn => {
        btn.addEventListener('click', () => {
            const targetId = btn.dataset.target;
            const offset = parseFloat(btn.dataset.offset);
            const input = document.getElementById(targetId);
            input.value = (parseFloat(input.value) + offset).toFixed(3);
        });
    });
    
    // Quick offset buttons
    document.querySelectorAll('.quick-offsets .btn-small').forEach(btn => {
        btn.addEventListener('click', () => {
            const axis = btn.dataset.axis;
            const delta = parseFloat(btn.dataset.delta);
            applyQuickCartesianOffset(axis, delta);
        });
    });

    // Auto-refresh toggle
    document.getElementById('autoRefreshToggle').addEventListener('change', (e) => {
        state.autoRefresh = e.target.checked;
        if (state.autoRefresh && state.selectedRobot) {
            subscribeToRobot(state.selectedRobot);
        }
    });
}

// ============== Robot Management ==============
async function scanForRobots() {
    try {
        const response = await fetch('/api/robots/scan', { method: 'POST' });
        const data = await response.json();
        
        if (data.success && data.discovered.length > 0) {
            showNotification(`Found ${data.discovered.length} robot(s)`, 'success');
            
            // Add discovered robots
            for (const namespace of data.discovered) {
                if (!state.robots.includes(namespace)) {
                    await addRobotByNamespace(namespace);
                }
            }
        } else {
            showNotification('No robots found', 'warning');
        }
    } catch (error) {
        console.error('Scan error:', error);
        showNotification('Failed to scan for robots', 'error');
    }
}

async function addRobot() {
    const input = document.getElementById('newRobotNamespace');
    const namespace = input.value.trim();
    
    if (!namespace) {
        showNotification('Please enter a robot namespace', 'warning');
        return;
    }
    
    await addRobotByNamespace(namespace);
    input.value = '';
}

async function addRobotByNamespace(namespace) {
    try {
        const response = await fetch('/api/robots/add', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ namespace })
        });
        
        const data = await response.json();
        
        if (data.success) {
            if (!state.robots.includes(namespace)) {
                state.robots.push(namespace);
            }
            updateRobotList();
            showNotification(`Added robot: ${namespace}`, 'success');
            
            // Auto-select if first robot
            if (state.robots.length === 1) {
                selectRobot(namespace);
            }
        } else {
            showNotification(`Failed to add robot: ${data.error || 'Unknown error'}`, 'error');
        }
    } catch (error) {
        console.error('Add robot error:', error);
        showNotification('Failed to add robot', 'error');
    }
}

async function removeRobot(namespace) {
    try {
        const response = await fetch('/api/robots/remove', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ namespace })
        });
        
        const data = await response.json();
        
        if (data.success) {
            state.robots = state.robots.filter(r => r !== namespace);
            delete state.robotStates[namespace];
            updateRobotList();
            
            if (state.selectedRobot === namespace) {
                state.selectedRobot = state.robots[0] || null;
                updateSelectedRobotDisplay();
            }
            
            showNotification(`Removed robot: ${namespace}`, 'success');
        }
    } catch (error) {
        console.error('Remove robot error:', error);
        showNotification('Failed to remove robot', 'error');
    }
}

function updateRobotList() {
    const container = document.getElementById('robotList');
    container.innerHTML = '';
    
    if (state.robots.length === 0) {
        container.innerHTML = '<p class="no-robots">No robots connected. Add a robot or scan for available robots.</p>';
        return;
    }
    
    state.robots.forEach(namespace => {
        const robotEl = document.createElement('div');
        robotEl.className = `robot-item ${state.selectedRobot === namespace ? 'selected' : ''}`;
        robotEl.innerHTML = `
            <span class="robot-name" title="${namespace}">${namespace}</span>
            <div class="robot-actions">
                <button class="btn-icon select-btn" title="Select robot">✓</button>
                <button class="btn-icon remove-btn" title="Remove robot">✕</button>
            </div>
        `;
        
        robotEl.querySelector('.select-btn').addEventListener('click', () => selectRobot(namespace));
        robotEl.querySelector('.remove-btn').addEventListener('click', () => removeRobot(namespace));
        robotEl.querySelector('.robot-name').addEventListener('click', () => selectRobot(namespace));
        
        container.appendChild(robotEl);
    });
}

function selectRobot(namespace) {
    state.selectedRobot = namespace;
    updateRobotList();
    updateSelectedRobotDisplay();
    subscribeToRobot(namespace);
    
    // Fetch initial state
    fetchRobotState(namespace);
    
    showNotification(`Selected robot: ${namespace}`, 'info');
}

function subscribeToRobot(namespace) {
    if (state.socket && state.connected) {
        state.socket.emit('subscribe_robot', { namespace });
    }
}

function updateSelectedRobotDisplay() {
    const robotName = state.selectedRobot || 'No robot selected';
    document.getElementById('selectedRobotJoint').textContent = robotName;
    document.getElementById('selectedRobotCartesian').textContent = robotName;
}

// ============== State Updates ==============
async function fetchRobotState(namespace) {
    try {
        const response = await fetch(`/api/robot/${namespace}/state`);
        const data = await response.json();
        
        if (data.success) {
            handleRobotStateUpdate({
                namespace,
                state: data.state
            });
        }
    } catch (error) {
        console.error('Fetch state error:', error);
    }
}

function handleRobotStateUpdate(data) {
    const { namespace, state: robotState } = data;
    
    // Store state
    state.robotStates[namespace] = robotState;
    
    // Track update rate
    state.updateCount++;
    
    // Only update UI if this is the selected robot
    if (namespace === state.selectedRobot) {
        updateFeedbackDisplay(robotState);
    }
}

function updateFeedbackDisplay(robotState) {
    // Update joint states table
    const jointStates = robotState.joint_states;
    const tbody = document.getElementById('jointStatesBody');
    
    if (jointStates.positions && jointStates.positions.length > 0) {
        tbody.innerHTML = jointStates.positions.map((pos, i) => {
            const name = jointStates.names[i] || `Joint ${i + 1}`;
            const velocity = jointStates.velocities[i] ?? '--';
            const effort = jointStates.efforts[i] ?? '--';
            
            return `
                <tr>
                    <td>${name}</td>
                    <td>${formatNumber(pos)}</td>
                    <td>${typeof velocity === 'number' ? formatNumber(velocity) : velocity}</td>
                    <td>${typeof effort === 'number' ? formatNumber(effort) : effort}</td>
                </tr>
            `;
        }).join('');
        
        // Update joint slider labels to match the actual joint names
        updateJointSliderLabels(jointStates.names);
    } else {
        tbody.innerHTML = '<tr><td colspan="4" class="no-data">No joint data available</td></tr>';
    }
    
    // Update Cartesian pose feedback
    const pose = robotState.cartesian_pose;
    
    document.getElementById('fbPosX').textContent = formatNumber(pose.position.x);
    document.getElementById('fbPosY').textContent = formatNumber(pose.position.y);
    document.getElementById('fbPosZ').textContent = formatNumber(pose.position.z);
    
    document.getElementById('fbOrientW').textContent = formatNumber(pose.orientation.w);
    document.getElementById('fbOrientX').textContent = formatNumber(pose.orientation.x);
    document.getElementById('fbOrientY').textContent = formatNumber(pose.orientation.y);
    document.getElementById('fbOrientZ').textContent = formatNumber(pose.orientation.z);
    
    // Update timestamp
    if (pose.timestamp) {
        const date = new Date(pose.timestamp);
        document.getElementById('poseTimestamp').textContent = `Last update: ${date.toLocaleTimeString()}`;
    }
}

// ============== Joint Control ==============
function updateJointSliderLabels(jointNames) {
    /**
     * Update the joint slider labels to match actual joint names from the robot
     */
    if (!jointNames || jointNames.length === 0) return;
    
    const sliderGroups = document.querySelectorAll('.joint-slider-group');
    sliderGroups.forEach((group, i) => {
        if (i < jointNames.length) {
            const label = group.querySelector('label');
            if (label) {
                label.textContent = jointNames[i];
            }
        }
    });
}

function getJointPositions() {
    const positions = [];
    document.querySelectorAll('.joint-slider-group').forEach(group => {
        const value = parseFloat(group.querySelector('.joint-slider').value);
        positions.push(value);
    });
    return positions;
}

function setJointPositions(positions) {
    document.querySelectorAll('.joint-slider-group').forEach((group, i) => {
        if (i < positions.length) {
            const slider = group.querySelector('.joint-slider');
            const valueInput = group.querySelector('.joint-value');
            slider.value = positions[i];
            valueInput.value = positions[i].toFixed(2);
        }
    });
}

function sendJointCommand() {
    if (!state.selectedRobot) {
        showNotification('Please select a robot first', 'warning');
        return;
    }
    
    const positions = getJointPositions();
    
    if (state.socket && state.connected) {
        state.socket.emit('send_joint_command', {
            namespace: state.selectedRobot,
            positions
        });
    } else {
        // Fallback to REST API
        fetch(`/api/robot/${state.selectedRobot}/joint_command`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ positions })
        }).then(r => r.json()).then(data => {
            if (data.success) {
                showNotification('Joint command sent', 'success');
            } else {
                showNotification('Failed to send joint command', 'error');
            }
        });
    }
}

function syncJointsFromFeedback() {
    if (!state.selectedRobot || !state.robotStates[state.selectedRobot]) {
        showNotification('No feedback data available', 'warning');
        return;
    }
    
    const robotState = state.robotStates[state.selectedRobot];
    const positions = robotState.joint_states.positions;
    
    if (positions && positions.length > 0) {
        setJointPositions(positions);
        showNotification('Synced joints from feedback', 'success');
    }
}

// ============== Cartesian Control ==============
function getCartesianPose() {
    return {
        position: {
            x: parseFloat(document.getElementById('posX').value) || 0,
            y: parseFloat(document.getElementById('posY').value) || 0,
            z: parseFloat(document.getElementById('posZ').value) || 0,
        },
        orientation: {
            w: parseFloat(document.getElementById('orientW').value) || 1,
            x: parseFloat(document.getElementById('orientX').value) || 0,
            y: parseFloat(document.getElementById('orientY').value) || 0,
            z: parseFloat(document.getElementById('orientZ').value) || 0,
        }
    };
}

function setCartesianPose(position, orientation) {
    document.getElementById('posX').value = position.x.toFixed(3);
    document.getElementById('posY').value = position.y.toFixed(3);
    document.getElementById('posZ').value = position.z.toFixed(3);
    
    document.getElementById('orientW').value = orientation.w.toFixed(3);
    document.getElementById('orientX').value = orientation.x.toFixed(3);
    document.getElementById('orientY').value = orientation.y.toFixed(3);
    document.getElementById('orientZ').value = orientation.z.toFixed(3);
}

function sendCartesianCommand() {
    if (!state.selectedRobot) {
        showNotification('Please select a robot first', 'warning');
        return;
    }
    
    const pose = getCartesianPose();
    
    if (state.socket && state.connected) {
        state.socket.emit('send_cartesian_command', {
            namespace: state.selectedRobot,
            position: pose.position,
            orientation: pose.orientation
        });
    } else {
        fetch(`/api/robot/${state.selectedRobot}/cartesian_command`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                position: pose.position,
                orientation: pose.orientation
            })
        }).then(r => r.json()).then(data => {
            if (data.success) {
                showNotification('Cartesian command sent', 'success');
            } else {
                showNotification('Failed to send Cartesian command', 'error');
            }
        });
    }
}

function syncCartesianFromFeedback() {
    if (!state.selectedRobot || !state.robotStates[state.selectedRobot]) {
        showNotification('No feedback data available', 'warning');
        return;
    }
    
    const robotState = state.robotStates[state.selectedRobot];
    const pose = robotState.cartesian_pose;
    
    setCartesianPose(pose.position, pose.orientation);
    showNotification('Synced Cartesian pose from feedback', 'success');
}

function applyQuickCartesianOffset(axis, delta) {
    if (!state.selectedRobot) {
        showNotification('Please select a robot first', 'warning');
        return;
    }
    
    const positionOffset = { x: 0, y: 0, z: 0 };
    positionOffset[axis] = delta;
    
    if (state.socket && state.connected) {
        state.socket.emit('send_cartesian_offset', {
            namespace: state.selectedRobot,
            position_offset: positionOffset
        });
    } else {
        fetch(`/api/robot/${state.selectedRobot}/cartesian_offset`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ position_offset: positionOffset })
        }).then(r => r.json()).then(data => {
            if (data.success) {
                showNotification(`Applied ${axis.toUpperCase()} offset: ${delta}`, 'success');
            }
        });
    }
}

// ============== UI Utilities ==============
function formatNumber(value) {
    if (typeof value !== 'number' || isNaN(value)) return '--';
    return value.toFixed(CONFIG.DECIMAL_PLACES);
}

function showNotification(message, type = 'info') {
    const container = document.getElementById('notifications');
    
    const notification = document.createElement('div');
    notification.className = `notification notification-${type}`;
    notification.innerHTML = `
        <span class="notification-message">${message}</span>
        <button class="notification-close">×</button>
    `;
    
    notification.querySelector('.notification-close').addEventListener('click', () => {
        notification.remove();
    });
    
    container.appendChild(notification);
    
    // Auto-remove after duration
    setTimeout(() => {
        if (notification.parentNode) {
            notification.classList.add('fade-out');
            setTimeout(() => notification.remove(), 300);
        }
    }, CONFIG.NOTIFICATION_DURATION);
}

function startUpdateRateMonitor() {
    state.updateRateInterval = setInterval(() => {
        const rate = state.updateCount;
        state.updateCount = 0;
        document.getElementById('updateRate').textContent = `${rate} Hz`;
    }, 1000);
}

// ============== Cleanup ==============
window.addEventListener('beforeunload', () => {
    if (state.socket) {
        state.socket.disconnect();
    }
    if (state.updateRateInterval) {
        clearInterval(state.updateRateInterval);
    }
});
