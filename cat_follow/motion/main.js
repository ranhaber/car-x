// main.js

function openTab(evt, tabName) {
    let i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tab-content");
    for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("tab-button");
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
    }
    document.getElementById(tabName).style.display = "block";
    evt.currentTarget.className += " active";
}

// --- API Calls ---
async function apiPost(url, data) {
    try {
        const response = await fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data),
        });
        return await response.json();
    } catch (error) {
        console.error('API POST Error:', error);
        return { error: error.message };
    }
}

async function apiGet(url) {
    try {
        const response = await fetch(url);
        return await response.json();
    } catch (error) {
        console.error('API GET Error:', error);
        return { error: error.message };
    }
}

// --- Calibration Logic ---
function setupCalibration() {
    const runSpeedTestBtn = document.getElementById('run-speed-test');
    const runSteerLeftBtn = document.getElementById('run-steer-test-left');
    const runSteerRightBtn = document.getElementById('run-steer-test-right');
    const saveBtn = document.getElementById('save-calibration');
    const saveStatus = document.getElementById('save-status');

    const speedJsonText = document.getElementById('speed-json');
    const steeringJsonText = document.getElementById('steering-json');

    runSpeedTestBtn.addEventListener('click', () => {
        const speed = document.getElementById('speed-test-val').value;
        apiPost('/api/calibrate/run_speed', { speed: parseInt(speed) });
    });

    runSteerLeftBtn.addEventListener('click', () => {
        const angle = document.getElementById('steer-test-left-val').value;
        apiPost('/api/calibrate/run_steer', { angle: parseInt(angle) });
    });

    runSteerRightBtn.addEventListener('click', () => {
        const angle = document.getElementById('steer-test-right-val').value;
        apiPost('/api/calibrate/run_steer', { angle: parseInt(angle) });
    });

    saveBtn.addEventListener('click', async () => {
        try {
            const speedData = JSON.parse(speedJsonText.value);
            const steeringData = JSON.parse(steeringJsonText.value);
            
            saveStatus.textContent = 'Saving...';
            const response = await apiPost('/api/calibration', {
                speed: speedData,
                steering: steeringData,
            });

            if (response.status === 'ok') {
                saveStatus.textContent = `Saved successfully! (${new Date().toLocaleTimeString()})`;
            } else {
                saveStatus.textContent = `Error: ${response.message || response.error}`;
            }
        } catch (e) {
            saveStatus.textContent = `Error: Invalid JSON format. ${e.message}`;
        }
    });

    // Load initial calibration data
    apiGet('/api/calibration').then(data => {
        if (data.error) {
            speedJsonText.value = `Error loading data: ${data.error}`;
            steeringJsonText.value = `Error loading data: ${data.error}`;
            return;
        }
        speedJsonText.value = JSON.stringify(data.speed, null, 2);
        steeringJsonText.value = JSON.stringify(data.steering, null, 2);
    });
}

// --- Status Logic ---
function setupStatus() {
    const statusDisplay = document.getElementById('status-display');
    setInterval(() => {
        apiGet('/api/status').then(data => {
            statusDisplay.textContent = data.error ? `Error: ${data.error}` : JSON.stringify(data, null, 2);
        });
    }, 2000);
}


// --- Main ---
window.addEventListener('DOMContentLoaded', () => {
    setupCalibration();
    setupStatus();
});