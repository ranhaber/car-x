"""
Web server (Flask) for controlling the cat-follow car and viewing status.
"""
import threading
from flask import Flask, jsonify, render_template, request
from cat_follow.motion.calibration_routines import run_speed_test, run_steer_test

# --- Globals to be injected from main_loop ---
# These are placeholders. The main application will set these.
_picarx_instance = None
_calibration_instance = None
_shared_state_instance = None

def set_globals(px, calib, shared):
    """Inject global objects from the main application."""
    global _picarx_instance, _calibration_instance, _shared_state_instance
    _picarx_instance = px
    _calibration_instance = calib
    _shared_state_instance = shared

# --- Flask App ---
app = Flask(__name__, template_folder='templates', static_folder='static')
app.config['SECRET_KEY'] = 'picar-x-is-awesome' # Change this

@app.route('/')
def index():
    """Render the main UI."""
    return render_template('index.html')

# --- API Endpoints ---

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get current robot status."""
    if not _shared_state_instance:
        return jsonify({"error": "System not initialized"}), 500
    
    odometry = _shared_state_instance.get_odometry()
    return jsonify({
        "odometry": {"x": odometry[0], "y": odometry[1], "heading": odometry[2]},
        "detector_model": _shared_state_instance.get_detector_model(),
    })

@app.route('/api/calibration', methods=['GET'])
def get_calibration():
    """Get all current calibration data."""
    if not _calibration_instance:
        return jsonify({"error": "Calibration not initialized"}), 500
    return jsonify(_calibration_instance.get_all_calibration_data())

@app.route('/api/calibration', methods=['POST'])
def save_calibration():
    """Save updated calibration data."""
    if not _calibration_instance:
        return jsonify({"error": "Calibration not initialized"}), 500
    
    data = request.json
    _calibration_instance.set_all_calibration_data(data)
    _calibration_instance.save()
    return jsonify({"status": "ok", "message": "Calibration saved."})

@app.route('/api/calibrate/run_speed', methods=['POST'])
def api_run_speed_test():
    """Run a speed calibration test."""
    if not _picarx_instance:
        return jsonify({"error": "Picarx not initialized"}), 500
    
    data = request.json
    speed = int(data.get('speed', 30))
    duration = float(data.get('duration', 1.0))
    
    # Run in a separate thread to not block the web server
    threading.Thread(target=run_speed_test, args=(_picarx_instance, speed, duration)).start()
    
    return jsonify({"status": "ok", "message": f"Running speed test at speed {speed}."})

@app.route('/api/calibrate/run_steer', methods=['POST'])
def api_run_steer_test():
    """Run a steering calibration test."""
    if not _picarx_instance:
        return jsonify({"error": "Picarx not initialized"}), 500
    
    data = request.json
    angle = int(data.get('angle', 30))
    speed = int(data.get('speed', 30))
    duration = float(data.get('duration', 4.0))

    threading.Thread(target=run_steer_test, args=(_picarx_instance, angle, speed, duration)).start()

    return jsonify({"status": "ok", "message": f"Running steer test with angle {angle}."})

def run_web_server(host='0.0.0.0', port=8080, debug=False):
    """Start the Flask web server."""
    print(f"Starting web server at http://{host}:{port}")
    app.run(host=host, port=port, debug=debug)

if __name__ == '__main__':
    # For standalone testing of the web UI without the full robot app
    print("Running web UI in standalone test mode.")
    from cat_follow.calibration.loader import Calibration
    from cat_follow.motion.calibration_routines import Picarx
    set_globals(px=Picarx(), calib=Calibration(), shared=None)
    run_web_server(debug=True)