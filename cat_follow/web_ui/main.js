document.addEventListener('DOMContentLoaded', () => {
    // --- Tab Controls ---
    window.openTab = (evt, tabName) => {
        const tabcontent = document.getElementsByClassName("tab-content");
        for (let i = 0; i < tabcontent.length; i++) {
            tabcontent[i].style.display = "none";
        }
        const tablinks = document.getElementsByClassName("tab-button");
        for (let i = 0; i < tablinks.length; i++) {
            tablinks[i].className = tablinks[i].className.replace(" active", "");
        }
        document.getElementById(tabName).style.display = "block";
        evt.currentTarget.className += " active";
    };

    // --- API Helper ---
    const apiPost = (url, data) => {
        return fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data),
        }).then(res => res.json());
    };

    // --- Main Tab Logic ---
    const statusDisplay = document.getElementById('status-display');
    setInterval(() => {
        fetch('/api/status')
            .then(res => res.json())
            .then(data => {
                // Alert on low battery (approx < 6.5V for 2S Li-ion)
                if (data.battery_v && data.battery_v > 0 && data.battery_v < 6.5) {
                    statusDisplay.style.border = "2px solid red";
                    statusDisplay.style.color = "red";
                } else {
                    statusDisplay.style.border = "";
                    statusDisplay.style.color = "";
                }
                statusDisplay.textContent = JSON.stringify(data, null, 2);
            })
            .catch(err => {
                statusDisplay.textContent = `Error fetching status: ${err}`;
            });
    }, 1000);

    document.getElementById('send-target').addEventListener('click', () => {
        const x = document.getElementById('target-x').value;
        const y = document.getElementById('target-y').value;
        apiPost('/api/target', { x: parseFloat(x), y: parseFloat(y) });
    });

    document.getElementById('stop-car').addEventListener('click', () => {
        apiPost('/api/stop', {});
    });

    document.getElementById('resolution-select').addEventListener('change', (e) => {
        apiPost('/api/stream/resolution', { resolution: e.target.value });
    });

    // --- Calibration Tab Logic ---
    const speedJsonText = document.getElementById('speed-json');
    const steeringJsonText = document.getElementById('steering-json');
    const bboxJsonText = document.getElementById('bbox-json');
    const saveStatus = document.getElementById('save-status');

    const loadCalibration = () => {
        fetch('/api/calibration')
            .then(res => res.json())
            .then(data => {
                speedJsonText.value = JSON.stringify(data.speed || {}, null, 2);
                steeringJsonText.value = JSON.stringify(data.steering || {}, null, 2);
                bboxJsonText.value = JSON.stringify(data.bbox_dist || {}, null, 2);
            })
            .catch(err => {
                saveStatus.textContent = `Error loading calibration: ${err}`;
                saveStatus.style.color = 'red';
            });
    };

    document.getElementById('save-calibration').addEventListener('click', () => {
        try {
            const speedData = JSON.parse(speedJsonText.value);
            const steeringData = JSON.parse(steeringJsonText.value);
            const bboxData = JSON.parse(bboxJsonText.value);

            const payload = {
                speed: speedData,
                steering: steeringData,
                bbox_dist: bboxData,
            };

            saveStatus.textContent = 'Saving...';
            saveStatus.style.color = 'black';

            apiPost('/api/calibration', payload)
                .then(res => {
                    if (res.status === 'ok') {
                        saveStatus.textContent = 'Save successful!';
                        saveStatus.style.color = 'green';
                    } else {
                        saveStatus.textContent = `Save failed: ${res.error || res.message}`;
                        saveStatus.style.color = 'red';
                    }
                })
                .catch(err => {
                    saveStatus.textContent = `Save error: ${err}`;
                    saveStatus.style.color = 'red';
                });

        } catch (e) {
            saveStatus.textContent = `Invalid JSON: ${e.message}`;
            saveStatus.style.color = 'red';
        }
    });

    document.getElementById('run-speed-test').addEventListener('click', () => {
        const speed = document.getElementById('speed-test-val').value;
        apiPost('/api/calibrate/run_speed', { speed: parseInt(speed, 10) })
            .then(res => console.log('Speed test response:', res));
    });

    document.getElementById('run-steer-test').addEventListener('click', () => {
        const angle = document.getElementById('steer-test-val').value;
        apiPost('/api/calibrate/run_steer', { angle: parseInt(angle, 10) })
            .then(res => console.log('Steer test response:', res));
    });

    // Initial load
    loadCalibration();
});