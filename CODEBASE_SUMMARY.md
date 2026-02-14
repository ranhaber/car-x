# Car-X Codebase Summary

**Project root:** `C:\Users\rahaber\my_projects\car-x`  
**Summary date:** February 2025

This repository combines three SunFounder libraries for the **PiCar-X** Raspberry Pi robot car: **picar-x** (car control), **vilib** (vision), and **robot-hat** (hardware abstraction). Together they support movement, sensors, camera, AI/LLM voice, and computer vision.

---

## 1. Repository layout

| Directory   | Role |
|------------|------|
| **picar-x/** | PiCar-X Python library and examples (car logic, servos, motors, grayscale, ultrasonic). |
| **vilib/**   | Vision library (picamera2, OpenCV, detection, Flask MJPEG stream). |
| **robot-hat/** | Robot HAT hardware library (GPIO, PWM, ADC, I2C, motors, servos, TTS, STT, LLM). |

**File counts (approximate):** 124 Python, 62 .po (i18n), 33 .rst (docs), configs (.toml, .yaml), media (.png, .jpg, .wav, .mp3, .tflite), shell scripts, device-tree overlays (.dtbo).

---

## 2. picar-x — Car control and examples

**Purpose:** High-level control of the PiCar-X hardware: motors, steering servo, camera pan/tilt servos, grayscale line/cliff sensing, and ultrasonic distance.

### 2.1 Core API (`picar-x/picarx/`)

- **picarx.py — `Picarx`**
  - **Config:** `/opt/picar-x/picar-x.conf`; fileDB for calibration (servos, motor direction, line/cliff references).
  - **Servos:** Camera pan/tilt (P0, P1), direction/steering (P2). Calibration and angle limits (e.g. direction ±30°, pan ±90°, tilt -35°..65°).
  - **Motors:** Two wheels via pins D4, D5 (direction), P13, P12 (PWM). `set_motor_speed(motor, speed)`, `forward(speed)`, `backward(speed)`, `stop()`, with steering compensation.
  - **Grayscale:** 3-channel ADC (A0, A1, A2) for line reference and cliff reference; `get_grayscale_data()`, `get_line_status()`, `get_cliff_status()`, `set_grayscale_reference()`, `set_cliff_reference()`.
  - **Ultrasonic:** Trig D2, Echo D3; `get_distance()`.
  - **Lifecycle:** `reset()`, `close()`.

- **preset_actions.py** — Preset motion/behavior sequences.
- **music.py, led.py** — Sound and LED helpers.
- **tts.py, stt.py, voice_assistant.py, llm.py** — Voice/AI wrappers (build on robot_hat and external voice/LLM stacks).

**Dependencies:** `robot_hat` (Pin, ADC, PWM, Servo, fileDB, Grayscale_Module, Ultrasonic, utils), `vilib` for vision examples.

### 2.2 Examples (`picar-x/example/`)

- **Calibration:** `1.cali_servo_motor.py`, `1.cali_grayscale.py`, `servo_zeroing.py`
- **Movement:** `2.move.py`, `3.keyboard_control.py`
- **Sensors:** `4.avoiding_obstacles.py`, `5.cliff_detection.py`, `6.line_tracking.py`
- **Vision:** `7.computer_vision.py`, `8.stare_at_you.py`, `9.record_video.py`, `10.bull_fight.py`, `11.video_car.py`
- **Control:** `12.app_control.py`
- **Audio:** `13.sound_background_music.py`, `14.voice_promt_car.py`, `15.storytelling_robot.py`, `16.voice_controlled_car.py`
- **AI/LLM:** `17.text_vision_talk.py`, `18.online_llm_test.py`, `19.local_voice_chatbot.py`, `20.treasure_hunt.py`, `21.voice_active_car_gpt.py`, `21.voice_active_car_doubao_cn.py`, `voice_active_car.py`

### 2.3 GPT examples (`picar-x/gpt_examples/`)

- **gpt_car.py** — GPT-driven car behavior.
- **openai_helper.py**, **keys.py**, **utils.py**, **preset_actions.py** — OpenAI integration and shared helpers.

---

## 3. vilib — Vision library

**Purpose:** Camera capture (picamera2), image processing, detection/classification, and optional Flask MJPEG/QR code web streaming.

### 3.1 Core (`vilib/vilib/`)

- **vilib.py**
  - **Camera:** Picamera2, configurable size (default 640×480), vflip/hflip, RGB888 preview, shared buffer.
  - **Pipeline:** Each frame is passed through: color detection → face → traffic sign → QR code → image classification → object detection → hands → pose. Results drawn on image; optional FPS overlay.
  - **Flask app:** Routes `/`, `/mjpg`, `/mjpg.jpg`, `/mjpg.png`, `/qrcode`, `/qrcode.png` for live view and QR code; runs on host `0.0.0.0`, port 9000. Web display and QR display are toggles.
  - **Paths:** Default pictures/videos under user home `Pictures/vilib/`, `Videos/vilib/`.

- **Detection/classification modules:**
  - **color_detection.py** — `color_detect_work()` (e.g. by color name).
  - **face_detection.py** — `set_face_detection_model()`, `face_detect()`.
  - **hands_detection.py** — `DetectHands()` (MediaPipe).
  - **pose_detection.py** — `DetectPose()` (MediaPipe).
  - **image_classification.py** — TFLite; `classify_image()`, `set_input_tensor()`, labels.
  - **objects_detection.py** — TFLite object detection; `detect_objects()`, threshold, labels.
  - **traffic_sign_detection.py** — `traffic_sign_detect()`, contour/area helpers.
  - **qrcode_recognition.py** — `qrcode_recognize()`.
  - **mediapipe_object_detection.py** — `MediapipeObjectDetection`.

- **utils.py** — `run_command()`, `getIP()`, `check_machine_type()`, `load_labels()`.

### 3.2 Examples (`vilib/examples/`)

- **color_detect.py**, **face_detect.py**, **hands_detection.py**, **pose_detection.py**
- **image_classification.py**, **objects_detection.py**, **traffic_sign_detect.py**
- **qrcode_read.py**, **qrcode_making.py**
- **display.py**, **controls.py**, **record_video.py**, **take_photo.py**
- **hsv_threshold_analyzer.py**

---

## 4. robot-hat — Hardware and AI voice stack

**Purpose:** Driver and abstraction for the Robot HAT (MCU, PWM, ADC, I2C, motors, servos, I2S/speaker). Plus TTS, STT, and LLM integrations for voice/AI.

### 4.1 Hardware and low-level (`robot-hat/robot_hat/`)

- **pin.py** — Digital GPIO (Pin).
- **pwm.py** — PWM output.
- **adc.py** — ADC input.
- **i2c.py** — I2C (e.g. `mem_read` for firmware version).
- **servo.py** — Servo angle control.
- **motor.py** — `Motor` / `Motors`; TC1508S (PWM+dir) or TC618S (dual-PWM) modes; period/prescaler, speed, direction.
- **filedb.py** — Key-value config storage (e.g. calibration).
- **config.py** — Config loading.
- **basic.py** — `_Basic_class` base.
- **device.py** — `Devices` / HAT product info (name, product_id, vendor).
- **utils** — MCU reset, speaker enable/disable, etc.
- **led.py**, **music.py** — LED and music playback.

### 4.2 Voice and AI (`robot-hat/robot_hat/`)

- **tts.py** — Wrappers that call `enable_speaker()` and delegate to `sunfounder_voice_assistant`: **Piper**, **Pico2Wave**, **Espeak**, **OpenAI_TTS**.
- **stt.py** — Speech-to-text (e.g. Vosk).
- **llm.py** — Re-exports from `sunfounder_voice_assistant.llm`: **LLM**, **Deepseek**, **Grok**, **Doubao**, **Gemini**, **Qwen**, **OpenAI**, **Ollama**.
- **speaker.py** — Speaker control.
- **voice_assistant.py** — Voice assistant glue.

### 4.3 CLI (`robot-hat/robot_hat/`)

- **__main__:** `reset_mcu`, `enable_speaker`, `disable_speaker`, `version`, `info` (HAT name, PCB, firmware version).

### 4.4 Examples (`robot-hat/examples/`)

- **TTS:** `tts_espeak.py`, `tts_pico2wave.py`, `tts_piper.py`, `tts_openai.py`
- **STT:** `stt_vosk_without_stream.py`, `stt_vosk_stream.py`, `stt_vosk_wake_word.py`, `stt_vosk_wake_word_thread.py`
- **LLM:** `llm_openai.py`, `llm_openai_with_image.py`, `llm_ollama.py`, `llm_ollama_with_image.py`, `llm_gemini.py`, `llm_grok.py`, `llm_deepseek.py`, `llm_qwen.py`, `llm_doubao.py`, `llm_doubao_with_image.py`, `llm_others.py`
- **voice_assistant.py**, **ultrasonic.py**, **led_test.py**, **pin_input.py**

### 4.5 Other (`robot-hat/`)

- **install.py** — Installation script (e.g. `sudo python3 install.py`).
- **dtoverlays/** — Device tree overlays (e.g. `sunfounder-robothat5.dtbo`, `sunfounder-servohat+.dtbo`).
- **docs/** — Sphinx API/docs (RST, locale).
- **tests/** — Unit tests (motor, servo, init angles, button, tone, piper stream).
- **i2samp.sh** — I2S amplifier setup for speaker.
- **pyproject.toml** — Package metadata and build.

---

## 5. Data flow (conceptual)

1. **robot_hat** — Talks to Robot HAT hardware (motors, servos, ADC, I2C, speaker). Provides TTS/STT/LLM wrappers.
2. **picarx** — Uses robot_hat pins/servos/motors and fileDB; implements PiCar-X calibration, movement, grayscale, ultrasonic. Used by picar-x examples and GPT car.
3. **vilib** — Captures camera frames, runs detection/classification pipelines, optionally streams via Flask. Used by picar-x vision examples (e.g. bull fight, stare-at-you, video car).

External stacks (not in this repo): **picamera2**, **sunfounder_voice_assistant**, **OpenAI/LLM APIs**, **TFLite/MediaPipe** models.

---

## 6. Installation (from READMEs)

- **robot_hat:** `git clone` robot-hat, `sudo python3 install.py` (or `pip3 install .`).
- **vilib:** `git clone` vilib (picamera2 branch), `sudo python3 install.py`.
- **picar-x:** Install robot_hat and vilib first, then `pip3 install .` in picar-x. Also requires **sunfounder_controller** (app control) and other deps per official docs.

---

## 7. Licenses and attribution

- **picar-x** and **robot-hat:** GPL-2.0; SunFounder.
- **vilib:** See vilib repo/LICENSE.
- Code and docs are from SunFounder; this summary describes the combined **car-x** tree for development reference.

---

*End of codebase summary.*
