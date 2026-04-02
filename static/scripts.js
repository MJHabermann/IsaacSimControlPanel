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

const MOTION_CONFIG = {
    SEND_RATE_HZ: 60,      // how often commands are sent (60Hz for smooth motion)
    DEFAULT_DURATION: 4.0, // seconds to reach target (for interpolated mode)
    NORMAL_VELOCITY: 0.01, // rad/s for normal speed
    SLOW_VELOCITY: 0.002   // rad/s for slow motion (5x slower)
};

// ============== Motion Control State ==============
const motionState = {
    isMoving: false,
    currentAnimationId: null,
    currentMotionName: null
};

/**
 * Quintic ease-in-out - very smooth acceleration and deceleration
 * Better than smoothstep for robot motion
 */
function easeInOutQuintic(t) {
    return t < 0.5
        ? 16 * t * t * t * t * t
        : 1 - Math.pow(-2 * t + 2, 5) / 2;
}

/**
 * Stop any currently running motion
 */
function stopMotion() {
    if (motionState.currentAnimationId) {
        cancelAnimationFrame(motionState.currentAnimationId);
        motionState.currentAnimationId = null;
    }
    
    if (motionState.isMoving) {
        motionState.isMoving = false;
        motionState.currentMotionName = null;
        updateStopButtonState();
        showNotification('Motion stopped', 'warning');
    }
}

/**
 * Start a motion (for tracking purposes)
 */
function startMotion(motionName) {
    motionState.isMoving = true;
    motionState.currentMotionName = motionName;
    updateStopButtonState();
}

/**
 * Complete a motion successfully
 */
function completeMotion() {
    motionState.isMoving = false;
    motionState.currentAnimationId = null;
    motionState.currentMotionName = null;
    updateStopButtonState();
}

/**
 * Update the Stop button enabled/disabled state
 */
function updateStopButtonState() {
    // Update all stop motion buttons
    document.querySelectorAll('.stopMotionBtn').forEach(btn => {
        btn.disabled = !motionState.isMoving;
        if (motionState.isMoving) {
            btn.classList.add('active');
        } else {
            btn.classList.remove('active');
        }
    });
}

// ============== State Management ==============
const state = {
    socket: null,
    connected: false,
    selectedRobot: null,
    robots: [],
    robotStates: {},
    robotConfigs: {},  // Track number of joints per robot: { 'Plug_Bot': 12, 'robot1': 6, ... }
    robotSlots: { 1: null, 2: null },  // Track which robot is assigned to each slot (1 and 2)
    selectedSlotOrder: [],  // Track the order in which slots were selected [1st selected robot, 2nd selected robot]
    autoRefresh: true,
    lastUpdateTime: null,
    updateCount: 0,
    updateRateInterval: null,
};

// ============== Authentication Helpers ==============
/**
 * Wrapper for fetch that handles authentication errors
 * Redirects to login page if server returns 401 Unauthorized
 */
async function authFetch(url, options = {}) {
    const response = await fetch(url, {
        ...options,
        credentials: 'same-origin'  // Include session cookies
    });
    
    if (response.status === 401) {
        // Session expired or not authenticated - redirect to login
        showNotification('Session expired. Redirecting to login...', 'warning');
        setTimeout(() => {
            window.location.href = '/login';
        }, 1500);
        throw new Error('Unauthorized');
    }
    
    return response;
}

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
        
        // Set up robot configurations (default 6 joints, special cases override)
        state.robots.forEach(robot => {
            if (robot === 'Plug_Bot') {
                state.robotConfigs[robot] = 12;  // Plug_Bot has 12 joints (6 arm + 6 gripper)
            } else if (!state.robotConfigs[robot]) {
                state.robotConfigs[robot] = 6;   // Default to 6 joints
            }
        });
        
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

    // Toggle joint sliders visibility
    document.querySelectorAll('.toggle-joint-view').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            toggleJointSliders(slot, btn);
        });
    });

    // Joint control - Slot-aware button listeners
    document.querySelectorAll('.sendJointCmd').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            sendJointCommandForSlot(slot);
        });
    });
    
    document.querySelectorAll('.syncFromFeedback').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            syncJointsFromFeedbackForSlot(slot);
        });
    });
    
    document.querySelectorAll('.homeJointsBtn').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            homeJointsForSlot(slot);
        });
    });
    
    document.querySelectorAll('.PaperRollBtn').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            readyToRollForSlot(slot);
        });
    });
    
    document.querySelectorAll('.MoveRollToConveyorBtn').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            moveRollToConveyorForSlot(slot);
        });
    });
    
    document.querySelectorAll('.stopMotionBtn').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            stopMotionForSlot(slot);
        });
    });
    
    
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
    
    // Setup gripper slider
    document.querySelectorAll('.gripper-slider-group').forEach(group => {
        const slider = group.querySelector('.gripper-slider');
        const valueInput = group.querySelector('.gripper-value');
        
        // Sync slider and input
        slider.addEventListener('input', () => {
            valueInput.value = parseFloat(slider.value).toFixed(3);
        });
        
        valueInput.addEventListener('change', () => {
            slider.value = valueInput.value;
        });
        
        // Offset buttons for gripper
        group.querySelectorAll('.gripper-offset').forEach(btn => {
            btn.addEventListener('click', () => {
                const offset = parseFloat(btn.dataset.offset);
                const newValue = parseFloat(slider.value) + offset;
                slider.value = newValue;
                valueInput.value = newValue.toFixed(3);
            });
        });
    });
    
    // Auto-refresh toggle
    document.getElementById('autoRefreshToggle').addEventListener('change', (e) => {
        state.autoRefresh = e.target.checked;
        if (state.autoRefresh && state.selectedRobot) {
            subscribeToRobot(state.selectedRobot);
        }
    });

    // Trigger buttons
    document.querySelectorAll('.btn-trigger').forEach(btn => {
        btn.addEventListener('click', () => {
            const group = parseInt(btn.dataset.group);
            sendTrigger(group, btn);
        });
    });

    // Roll Holder button
    const rollHolderBtn = document.getElementById('rollHolderOpenBtn');
    if (rollHolderBtn) {
        rollHolderBtn.addEventListener('click', () => {
            sendRollHolderTrigger(rollHolderBtn);
        });
    }

    // Cartesian control - Slot-aware button listeners
    document.querySelectorAll('.sendCartesianCmd').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            sendCartesianCommandForSlot(slot);
        });
    });
    
    document.querySelectorAll('.syncCartesianFromFeedback').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            syncCartesianFromFeedbackForSlot(slot);
        });
    });

    // Cartesian offset buttons for position inputs (slot-specific)
    document.querySelectorAll('.cartesian-inputs .input-with-offset .btn-offset').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            const targetId = btn.dataset.target;
            const offset = parseFloat(btn.dataset.offset);
            const inputId = `${targetId}-slot-${slot}`;
            const input = document.getElementById(inputId);
            if (input) {
                input.value = (parseFloat(input.value) + offset).toFixed(3);
            }
        });
    });
    
    // Quick cartesian offset buttons (slot-aware)
    document.querySelectorAll('.quick-offsets .btn-small').forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            const axis = btn.dataset.axis;
            const delta = parseFloat(btn.dataset.delta);
            applyQuickCartesianOffsetForSlot(slot, axis, delta);
        });
    });
}

// ============== Joint Sliders Toggle ==============
function toggleJointSliders(slot, button) {
    const panel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
    const jointsContainer = panel.querySelector('.joint-sliders');
    
    const isHidden = jointsContainer.classList.contains('hidden');
    
    if (isHidden) {
        // Show sliders
        jointsContainer.classList.remove('hidden');
        button.textContent = '👁️ Hide Sliders';
        button.classList.remove('hiding-sliders');
        button.classList.add('showing-sliders');
    } else {
        // Hide sliders
        jointsContainer.classList.add('hidden');
        button.textContent = '👁️‍🗨️ Show Sliders';
        button.classList.remove('showing-sliders');
        button.classList.add('hiding-sliders');
    }
}

// ============== Robot Management ==============
async function scanForRobots() {
    try {
        console.log('[Scan] Starting robot scan...');
        const response = await authFetch('/api/robots/scan', { method: 'POST' });
        
        console.log(`[Scan] Response status: ${response.status}`);
        
        if (!response.ok) {
            const errorText = await response.text();
            console.error(`[Scan] HTTP ${response.status}: ${errorText}`);
            showNotification(`Failed to scan: HTTP ${response.status}`, 'error');
            return;
        }
        
        const data = await response.json();
        console.log('[Scan] Response data:', data);
        
        if (data.success && data.discovered && data.discovered.length > 0) {
            showNotification(`Found ${data.discovered.length} robot(s)`, 'success');
            
            // Add discovered robots
            for (const namespace of data.discovered) {
                console.log(`[Scan] Adding discovered robot: ${namespace}`);
                if (!state.robots.includes(namespace)) {
                    await addRobotByNamespace(namespace);
                }
            }
        } else {
            showNotification('No robots found', 'warning');
        }
    } catch (error) {
        console.error('[Scan] Error:', error);
        showNotification(`Failed to scan for robots: ${error.message}`, 'error');
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
        const response = await authFetch('/api/robots/add', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ namespace })
        });
        
        console.log(`[Robot Add] Response status: ${response.status} for namespace: ${namespace}`);
        
        if (!response.ok) {
            const errorText = await response.text();
            console.error(`[Robot Add] HTTP ${response.status}: ${errorText}`);
            showNotification(`Failed to add robot: HTTP ${response.status}`, 'error');
            return;
        }
        
        const data = await response.json();
        console.log(`[Robot Add] Response data:`, data);
        
        if (data.success) {
            if (!state.robots.includes(namespace)) {
                state.robots.push(namespace);
            }
            // Store the number of joints for this robot (default 6)
            state.robotConfigs[namespace] = data.num_joints || 6;
            
            updateRobotList();
            showNotification(`Added robot: ${namespace} (${state.robotConfigs[namespace]} joints)`, 'success');
            
            // Auto-select if first robot
            if (state.robots.length === 1) {
                selectRobot(namespace);
            }
        } else {
            showNotification(`Failed to add robot: ${data.error || 'Unknown error'}`, 'error');
        }
    } catch (error) {
        console.error('[Robot Add] Error:', error);
        showNotification(`Failed to add robot: ${error.message}`, 'error');
    }
}

async function removeRobot(namespace) {
    try {
        const response = await authFetch('/api/robots/remove', {
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
    // Check if robot is already selected
    const alreadySelected = Object.values(state.robotSlots).includes(namespace);
    
    if (alreadySelected) {
        // If already selected, remove it from slots
        for (let slot in state.robotSlots) {
            if (state.robotSlots[slot] === namespace) {
                state.robotSlots[slot] = null;
                break;
            }
        }
        state.selectedSlotOrder = state.selectedSlotOrder.filter(r => r !== namespace);
        showNotification(`Removed ${namespace} from control`, 'info');
    } else {
        // Find the first empty slot or use the oldest selected robot's slot
        let assignedSlot = null;
        for (let slot of [1, 2]) {
            if (state.robotSlots[slot] === null) {
                assignedSlot = slot;
                break;
            }
        }
        
        // If no empty slot, replace the first one selected
        if (assignedSlot === null) {
            assignedSlot = state.selectedSlotOrder.length > 0 ? 1 : 1;
            const replacedRobot = state.robotSlots[assignedSlot];
            if (replacedRobot) {
                state.selectedSlotOrder = state.selectedSlotOrder.filter(r => r !== replacedRobot);
            }
        }
        
        state.robotSlots[assignedSlot] = namespace;
        state.selectedSlotOrder.push(namespace);
        state.selectedRobot = namespace;
        
        subscribeToRobot(namespace);
        fetchRobotState(namespace);
        showNotification(`${namespace} assigned to Slot ${assignedSlot}`, 'success');
    }
    
    updateRobotList();
    updateSlotAssignments();
}

function updateSlotAssignments() {
    // Update the display labels in each slot
    for (let slot of [1, 2]) {
        const robot = state.robotSlots[slot];
        const label = document.querySelector(`.robot-slot-${slot}`);
        if (label) {
            const displayText = robot ? robot : 'No robot assigned';
            if (label.textContent !== displayText) {
                label.textContent = displayText;
            }
        }
        
        // Also update cartesian panel header
        const cartLabel = document.querySelector(`.robot-slot-${slot}-cart`);
        if (cartLabel) {
            const displayText = robot ? robot : 'No robot assigned';
            if (cartLabel.textContent !== displayText) {
                cartLabel.textContent = displayText;
            }
        }
        
        // Show/hide gripper section based on number of joints
        const slotPanel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
        if (slotPanel) {
            const gripperSections = slotPanel.querySelectorAll('.gripper-section');
            const gripperSliderGroup = slotPanel.querySelector('.gripper-slider-group');
            
            const numJoints = robot && state.robotConfigs[robot] ? state.robotConfigs[robot] : 6;
            const shouldShowGripper = numJoints > 6;
            
            gripperSections.forEach(section => {
                section.style.display = shouldShowGripper ? 'block' : 'none';
            });
            
            if (gripperSliderGroup) {
                gripperSliderGroup.style.display = shouldShowGripper ? 'block' : 'none';
            }
        }
    }
}

function subscribeToRobot(namespace) {
    if (state.socket && state.connected) {
        state.socket.emit('subscribe_robot', { namespace });
    }
}

function updateSelectedRobotDisplay() {
    const robotName = state.selectedRobot || 'No robot selected';
    const el = document.getElementById('selectedRobotJoint');
    if (el) {
        el.textContent = robotName;
    }
}

// ============== State Updates ==============
async function fetchRobotState(namespace) {
    try {
        const response = await authFetch(`/api/robot/${namespace}/state`);
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
    
    // Update feedback for the selected robot
    if (namespace === state.selectedRobot) {
        updateFeedbackDisplay(robotState);
    }
    
    // Update feedback for all slots where this robot is assigned
    for (let slot of [1, 2]) {
        if (state.robotSlots[slot] === namespace) {
            updateFeedbackDisplayForSlot(robotState, slot);
        }
    }
}

function updateFeedbackDisplay(robotState) {
    // Update joint states table
    const jointStates = robotState.joint_states;
    const tbody = document.getElementById('jointStatesBody');
    
    if (jointStates && jointStates.positions && jointStates.positions.length > 0) {
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
    
    // Update Cartesian pose feedback if available
    if (robotState.cartesian_pose) {
        const pose = robotState.cartesian_pose;
        
        // Update position feedback
        document.getElementById('fbPosX').textContent = formatNumber(pose.position.x);
        document.getElementById('fbPosY').textContent = formatNumber(pose.position.y);
        document.getElementById('fbPosZ').textContent = formatNumber(pose.position.z);
        
        // Update orientation feedback
        document.getElementById('fbOrientW').textContent = formatNumber(pose.orientation.w);
        document.getElementById('fbOrientX').textContent = formatNumber(pose.orientation.x);
        document.getElementById('fbOrientY').textContent = formatNumber(pose.orientation.y);
        document.getElementById('fbOrientZ').textContent = formatNumber(pose.orientation.z);
        
        // Update timestamp if available
        if (pose.timestamp) {
            const date = new Date(pose.timestamp);
            document.getElementById('poseTimestamp').textContent = `Last update: ${date.toLocaleTimeString()}`;
        }
    }
}

function updateFeedbackDisplayForSlot(robotState, slot) {
    // Update joint slider labels for a specific slot
    const jointStates = robotState.joint_states;
    
    if (jointStates.names) {
        updateJointSliderLabels(jointStates.names, slot);
    }
    
    // Update Cartesian pose feedback for this slot
    if (robotState.cartesian_pose) {
        const pose = robotState.cartesian_pose;
        console.log(`Auto-syncing cartesian pose for slot ${slot}:`, pose);
        setCartesianPoseForSlot(slot, pose.position, pose.orientation);
    } else {
        console.warn(`No cartesian_pose data in robot state for slot ${slot}. State:`, robotState);
    }
}

// ============== Joint Control ==============
function updateJointSliderLabels(jointNames, slot = null) {
    /**
     * Update the joint slider labels to match actual joint names from the robot
     * If slot is specified, only update sliders in that slot
     */
    if (!jointNames || jointNames.length === 0) return;
    
    let sliderGroups;
    if (slot) {
        const slotPanel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
        sliderGroups = slotPanel ? slotPanel.querySelectorAll('.joint-slider-group') : [];
    } else {
        sliderGroups = document.querySelectorAll('.joint-slider-group');
    }
    
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

// Slot-aware versions of joint position getters/setters
function getJointPositionsForSlot(slot) {
    const positions = [];
    const slotPanel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
    if (!slotPanel) return positions;
    
    const robot = state.robotSlots[slot];
    // Get the number of joints this robot has (default 6 if not configured)
    const numJoints = robot && state.robotConfigs[robot] ? state.robotConfigs[robot] : 6;
    
    // Collect arm joint positions (0-5)
    let jointIndex = 0;
    slotPanel.querySelectorAll('.joint-slider-group').forEach(group => {
        if (jointIndex < numJoints && jointIndex < 6) {
            const value = parseFloat(group.querySelector('.joint-slider').value);
            positions.push(value);
            jointIndex++;
        }
    });
    
    // If robot has gripper joints, get them from the gripper slider
    if (numJoints > 6) {
        const gripperSlider = slotPanel.querySelector('.gripper-slider');
        if (gripperSlider) {
            const gripperValue = parseFloat(gripperSlider.value);
            // Set all 6 gripper joints (indices 6-11) to the same value
            for (let i = 6; i < numJoints; i++) {
                positions.push(gripperValue);
            }
        }
    }
    
    return positions;
}

function setJointPositionsForSlot(slot, positions) {
    const slotPanel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
    if (!slotPanel) return;
    
    // Set arm joint positions (0-5)
    slotPanel.querySelectorAll('.joint-slider-group').forEach((group, i) => {
        if (i < positions.length && i < 6) {
            const slider = group.querySelector('.joint-slider');
            const valueInput = group.querySelector('.joint-value');
            slider.value = positions[i];
            valueInput.value = positions[i].toFixed(2);
        }
    });
    
    // Set gripper slider if there are gripper positions (6+)
    if (positions.length > 6) {
        const gripperSlider = slotPanel.querySelector('.gripper-slider');
        const gripperValue = slotPanel.querySelector('.gripper-value');
        if (gripperSlider) {
            // Use the first gripper joint value (they should all be the same anyway)
            gripperSlider.value = positions[6];
            gripperValue.value = positions[6].toFixed(3);
        }
    }
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
        authFetch(`/api/robot/${state.selectedRobot}/joint_command`, {
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

// Send trigger pulse (True then False) to /Cheese/groupN topic
function sendTrigger(group, btn) {
    // Visual feedback - disable button briefly
    btn.disabled = true;
    btn.classList.add('triggering');
    
    if (state.socket && state.connected) {
        state.socket.emit('send_trigger', { group });
    } else {
        // Fallback to REST API
        authFetch(`/api/trigger/${group}`, {
            method: 'POST'
        }).then(r => r.json()).then(data => {
            if (!data.success) {
                showNotification(`Trigger G${group} failed`, 'error');
            }
        }).catch(() => {
            showNotification(`Trigger G${group} failed`, 'error');
        });
    }
    
    // Re-enable after 200ms (trigger takes 100ms)
    setTimeout(() => {
        btn.disabled = false;
        btn.classList.remove('triggering');
    }, 200);
}

// Send roll holder open trigger pulse
function sendRollHolderTrigger(btn) {
    // Visual feedback - disable button briefly
    btn.disabled = true;
    btn.classList.add('triggering');
    
    if (state.socket && state.connected) {
        state.socket.emit('send_roll_holder_trigger');
    } else {
        // Fallback to REST API
        authFetch('/api/trigger/roll-holder', {
            method: 'POST'
        }).then(r => r.json()).then(data => {
            if (!data.success) {
                showNotification('Roll Holder trigger failed', 'error');
            } else {
                showNotification('Roll Holder opened', 'success');
            }
        }).catch(() => {
            showNotification('Roll Holder trigger failed', 'error');
        });
    }
    
    // Re-enable after 200ms (trigger takes 100ms)
    setTimeout(() => {
        btn.disabled = false;
        btn.classList.remove('triggering');
    }, 200);
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

// Slot-aware command functions
function sendJointCommandForSlot(slot, velocity = null) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`No robot assigned to Slot ${slot}`, 'warning');
        return;
    }
    
    const positions = getJointPositionsForSlot(slot);
    
    if (state.socket && state.connected) {
        state.socket.emit('send_joint_command', {
            namespace: robot,
            positions,
            velocity: velocity
        });
    } else {
        // Fallback to REST API
        const payload = { positions };
        if (velocity !== null) {
            payload.velocity = velocity;
        }
        
        authFetch(`/api/robot/${robot}/joint_command`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        }).then(r => r.json()).then(data => {
            if (data.success) {
                showNotification(`Joint command sent to ${robot}`, 'success');
            } else {
                showNotification('Failed to send joint command', 'error');
            }
        });
    }
}

function syncJointsFromFeedbackForSlot(slot) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`No robot assigned to Slot ${slot}`, 'warning');
        return;
    }
    
    if (!state.robotStates[robot]) {
        showNotification(`No feedback data available for ${robot}`, 'warning');
        return;
    }
    
    const robotState = state.robotStates[robot];
    const positions = robotState.joint_states.positions;
    
    if (positions && positions.length > 0) {
        setJointPositionsForSlot(slot, positions);
        showNotification(`Synced ${robot} from feedback`, 'success');
    }
}

function homeJointsForSlot(slot) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`No robot assigned to Slot ${slot}`, 'warning');
        return;
    }
    
    // Set all joint sliders in this slot to 0
    const slotPanel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
    if (!slotPanel) return;
    
    slotPanel.querySelectorAll('.joint-slider-group').forEach(group => {
        const slider = group.querySelector('.joint-slider');
        const valueInput = group.querySelector('.joint-value');
        slider.value = 0;
        valueInput.value = '0.00';
    });
    
    // Send the joint command to the robot in this slot
    sendJointCommandForSlot(slot);
    showNotification(`${robot} joints homed`, 'success');
}

function readyToRollForSlot(slot) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`No robot assigned to Slot ${slot}`, 'warning');
        return;
    }
    
    // Set specific joint positions for ready to roll
    const slotPanel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
    if (!slotPanel) return;
    
    const positions = [0, 0.30, -0.5, 0, -0.5];  // Default ready to roll positions
    slotPanel.querySelectorAll('.joint-slider-group').forEach((group, index) => {
        if (index < positions.length) {
            const slider = group.querySelector('.joint-slider');
            const valueInput = group.querySelector('.joint-value');
            slider.value = positions[index];
            valueInput.value = positions[index].toFixed(2);
        }
    });
    
    // Send the joint command to the robot in this slot
    sendJointCommandForSlot(slot);
    showNotification(`${robot} ready to roll`, 'success');
}

function moveRollToConveyorForSlot(slot) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`No robot assigned to Slot ${slot}`, 'warning');
        return;
    }
    
    // Stop any existing motion first
    if (motionState.isMoving) {
        stopMotion();
        return;
    }

    const slotPanel = document.querySelector(`.joint-control-panel[data-slot="${slot}"]`);
    if (!slotPanel) return;

    const jointGroups = slotPanel.querySelectorAll('.joint-slider-group');

    // Target joint values for move to conveyor
    const targets = [];
    jointGroups.forEach((_, index) => {
        if (index === 1) targets[index] = -0.5;
        else if (index === 2) targets[index] = 0.5;
        else if (index === 4) targets[index] = -0.5;
        else targets[index] = 0;
    });

    // Set target positions in UI
    jointGroups.forEach((group, index) => {
        const slider = group.querySelector('.joint-slider');
        const valueInput = group.querySelector('.joint-value');
        slider.value = targets[index];
        valueInput.value = targets[index].toFixed(3);
    });
    
    // Send the joint command to the robot in this slot with SLOW velocity
    sendJointCommandForSlot(slot, MOTION_CONFIG.SLOW_VELOCITY);
    showNotification('Move Roll to Conveyor command sent (slow speed)', 'success');
    return;

    // Interpolated mode: Gradually send intermediate positions
    const starts = Array.from(jointGroups, group =>
        parseFloat(group.querySelector('.joint-slider').value)
    );

    const duration = MOTION_CONFIG.DEFAULT_DURATION * 1000; // ms
    const sendInterval = 1000 / MOTION_CONFIG.SEND_RATE_HZ;

    const startTime = performance.now();
    let lastSendTime = 0;
    
    // Start motion tracking
    startMotion('Move Roll to Conveyor');

    function update(now) {
        // Check if motion was cancelled
        if (!motionState.isMoving) {
            return;
        }
        
        const elapsed = now - startTime;
        const t = Math.min(elapsed / duration, 1);
        const easedT = easeInOutQuintic(t);

        jointGroups.forEach((group, index) => {
            const slider = group.querySelector('.joint-slider');
            const valueInput = group.querySelector('.joint-value');

            const value =
                starts[index] + (targets[index] - starts[index]) * easedT;

            slider.value = value;
            valueInput.value = value.toFixed(3);
        });

        // Send at fixed rate
        if (now - lastSendTime >= sendInterval) {
            sendJointCommandForSlot(slot);
            lastSendTime = now;
        }

        if (t < 1) {
            motionState.currentAnimationId = requestAnimationFrame(update);
        } else {
            sendJointCommandForSlot(slot); // final snap
            completeMotion();
            showNotification('Move to conveyor complete', 'success');
        }
    }

    motionState.currentAnimationId = requestAnimationFrame(update);
}

function stopMotionForSlot(slot) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`No robot assigned to Slot ${slot}`, 'warning');
        return;
    }
    
    // Just stop the motion animation (works globally)
    stopMotion();
}

// ============== Cartesian Control ==============
function getCartesianPoseForSlot(slot) {
    return {
        position: {
            x: parseFloat(document.getElementById(`posX-slot-${slot}`).value) || 0,
            y: parseFloat(document.getElementById(`posY-slot-${slot}`).value) || 0,
            z: parseFloat(document.getElementById(`posZ-slot-${slot}`).value) || 0,
        },
        orientation: {
            w: parseFloat(document.getElementById(`orientW-slot-${slot}`).value) || 1,
            x: parseFloat(document.getElementById(`orientX-slot-${slot}`).value) || 0,
            y: parseFloat(document.getElementById(`orientY-slot-${slot}`).value) || 0,
            z: parseFloat(document.getElementById(`orientZ-slot-${slot}`).value) || 0,
        }
    };
}

function setCartesianPoseForSlot(slot, position, orientation) {
    try {
        if (!position || !orientation) {
            console.warn(`Cartesian pose data missing for slot ${slot}:`, {position, orientation});
            return;
        }
        
        document.getElementById(`posX-slot-${slot}`).value = (position.x || 0).toFixed(3);
        document.getElementById(`posY-slot-${slot}`).value = (position.y || 0).toFixed(3);
        document.getElementById(`posZ-slot-${slot}`).value = (position.z || 0).toFixed(3);
        
        document.getElementById(`orientW-slot-${slot}`).value = (orientation.w || 1).toFixed(3);
        document.getElementById(`orientX-slot-${slot}`).value = (orientation.x || 0).toFixed(3);
        document.getElementById(`orientY-slot-${slot}`).value = (orientation.y || 0).toFixed(3);
        document.getElementById(`orientZ-slot-${slot}`).value = (orientation.z || 0).toFixed(3);
        
        console.log(`Cartesian pose synced for slot ${slot}:`, {position, orientation});
    } catch (error) {
        console.error(`Error setting cartesian pose for slot ${slot}:`, error);
        showNotification(`Error syncing cartesian pose for slot ${slot}`, 'error');
    }
}

function sendCartesianCommandForSlot(slot) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`Please assign a robot to Slot ${slot}`, 'warning');
        return;
    }
    
    const pose = getCartesianPoseForSlot(slot);
    if (!pose) return;
    
    if (state.socket && state.connected) {
        state.socket.emit('send_cartesian_command', {
            namespace: robot,
            position: pose.position,
            orientation: pose.orientation
        });
    } else {
        // Fallback to REST API
        authFetch(`/api/robot/${robot}/cartesian_command`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                position: pose.position,
                orientation: pose.orientation
            })
        }).then(r => r.json()).then(data => {
            if (data.success) {
                showNotification(`Cartesian command sent to Slot ${slot}`, 'success');
            } else {
                showNotification(`Failed to send Cartesian command to Slot ${slot}`, 'error');
            }
        });
    }
}

function syncCartesianFromFeedbackForSlot(slot) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`Please assign a robot to Slot ${slot}`, 'warning');
        return;
    }
    
    console.log(`Syncing cartesian feedback for slot ${slot}, robot: ${robot}`);
    
    // Try to fetch latest state from backend first
    authFetch(`/api/robot/${robot}/state`)
        .then(r => r.json())
        .then(data => {
            if (data.success && data.state) {
                // Update the state cache
                state.robotStates[robot] = data.state;
                console.log(`Fetched latest robot state:`, data.state);
                
                // Now try to sync cartesian
                const robotState = data.state;
                if (robotState && robotState.cartesian_pose) {
                    const pose = robotState.cartesian_pose;
                    console.log(`Setting pose from feedback:`, pose);
                    setCartesianPoseForSlot(slot, pose.position, pose.orientation);
                    showNotification(`Synced Cartesian pose from Slot ${slot} feedback`, 'success');
                } else {
                    console.warn(`No cartesian feedback in fetched state:`, robotState);
                    showNotification(`No Cartesian feedback available for Slot ${slot}`, 'warning');
                }
            } else {
                throw new Error('Failed to fetch robot state');
            }
        })
        .catch(error => {
            console.error(`Error fetching robot state for sync:`, error);
            // Fallback to cached state
            const robotState = state.robotStates[robot];
            if (robotState && robotState.cartesian_pose) {
                const pose = robotState.cartesian_pose;
                console.log(`Using cached state, setting pose:`, pose);
                setCartesianPoseForSlot(slot, pose.position, pose.orientation);
                showNotification(`Synced Cartesian pose from Slot ${slot} (cached)`, 'success');
            } else {
                console.warn(`Fallback failed - no cached state`, robotState);
                showNotification(`Unable to sync: No robot feedback available`, 'error');
            }
        });
}

function applyQuickCartesianOffsetForSlot(slot, axis, delta) {
    const robot = state.robotSlots[slot];
    if (!robot) {
        showNotification(`Please assign a robot to Slot ${slot}`, 'warning');
        return;
    }
    
    const positionOffset = { x: 0, y: 0, z: 0 };
    positionOffset[axis] = delta;
    
    if (state.socket && state.connected) {
        state.socket.emit('send_cartesian_offset', {
            namespace: robot,
            position_offset: positionOffset
        });
    } else {
        // Fallback to REST API
        authFetch(`/api/robot/${robot}/cartesian_offset`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ position_offset: positionOffset })
        }).then(r => r.json()).then(data => {
            if (data.success) {
                showNotification(`Applied ${axis.toUpperCase()} offset: ${delta}m to Slot ${slot}`, 'success');
            }
        });
    }
}

// ============== UI Utilities -->
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
