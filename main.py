import time
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from threading import Thread, Timer


# Mock Pin and PWM classes for testing purposes
class MockPin:
    def __init__(self, pin, mode):
        self.pin = pin
        self.mode = mode
        self.state = 0

    def value(self, state=None):
        if state is not None:
            self.state = state
        return self.state


class MockPWM:
    def __init__(self, pin):
        self.pin = pin
        self.duty_value = 0

    def duty(self, duty):
        self.duty_value = duty


# Initialize motor pins and PWM
Motor1_pwm1 = MockPWM(1)
Motor1_pwm2 = MockPWM(2)
Motor2_pwm1 = MockPWM(3)
Motor2_pwm2 = MockPWM(4)
Motor3_pwm1 = MockPWM(5)
Motor3_pwm2 = MockPWM(6)
Motor4_pwm1 = MockPWM(7)
Motor4_pwm2 = MockPWM(8)


# Mock MPU6050 sensor for balancing
class MockMPU6050:
    def get_accel_data(self):
        return {'x': 0, 'y': 0, 'z': 1}

    def get_gyro_data(self):
        return {'x': 0, 'y': 0, 'z': 0}


sensor = MockMPU6050()


# PID controller class for stabilization
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output


# PID controllers for each axis
pitch_pid = PID(1.0, 0.01, 0.1)
roll_pid = PID(1.0, 0.01, 0.1)
yaw_pid = PID(1.0, 0.01, 0.1)

# Target angles for stabilization
target_pitch = 0
target_roll = 0
target_yaw = 0

# Global variables for speed and button state
speed1, speed2, speed3, speed4 = 0, 0, 0, 0
max_speed = 80
acceleration_step = 5
button_pressed = False  # Tracks if any button is pressed


def set_pwm_duty(pwm, duty):
    pwm.duty(duty)


def accelerate(current_speed):
    new_speed = current_speed + acceleration_step
    return min(new_speed, max_speed)


def decelerate(current_speed):
    new_speed = current_speed - acceleration_step
    return max(new_speed, 0)


def adjust_motor_speeds():
    pitch_output = pitch_pid.compute(target_pitch, 0)
    roll_output = roll_pid.compute(target_roll, 0)
    yaw_output = yaw_pid.compute(target_yaw, 0)

    set_pwm_duty(Motor1_pwm1, speed1 + pitch_output - roll_output + yaw_output)
    set_pwm_duty(Motor2_pwm2, speed2 + pitch_output + roll_output - yaw_output)
    set_pwm_duty(Motor3_pwm2, speed3 - pitch_output + roll_output + yaw_output)
    set_pwm_duty(Motor4_pwm1, speed4 - pitch_output - roll_output - yaw_output)


def hover():
    global speed1, speed2, speed3, speed4
    avg_speed = (speed1 + speed2 + speed3 + speed4) // 4
    speed1 = speed2 = speed3 = speed4 = avg_speed
    print(f"[HOVER] Setting all motors to average speed: {avg_speed}")
    adjust_motor_speeds()


def start_hover_timer():
    """Triggers hover if no buttons are pressed after a delay."""
    global button_pressed
    button_pressed = False
    Timer(0.5, check_hover).start()  # 0.5 seconds delay before checking


def check_hover():
    """Checks if no buttons are pressed and triggers hover."""
    if not button_pressed:
        hover()


# Movement functions
def up():
    global speed1, speed2, speed3, speed4, button_pressed
    button_pressed = True
    speed1 = accelerate(speed1)
    speed2 = accelerate(speed2)
    speed3 = accelerate(speed3)
    speed4 = accelerate(speed4)
    print(f"Moving up.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def down():
    global speed1, speed2, speed3, speed4, button_pressed
    button_pressed = True
    speed1 = decelerate(speed1)
    speed2 = decelerate(speed2)
    speed3 = decelerate(speed3)
    speed4 = decelerate(speed4)
    print(f"Moving down.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def forward():
    global speed3, speed4, button_pressed
    button_pressed = True
    speed3 = accelerate(speed3)
    speed4 = accelerate(speed4)
    print(f"Moving forward.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


# Other movement functions (similar)
def stop():
    global speed1, speed2, speed3, speed4, button_pressed
    button_pressed = True
    speed1 = speed2 = speed3 = speed4 = 0
    print(f"Stoping.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def left():
    global speed2, speed4, button_pressed
    button_pressed = True
    speed2 = accelerate(speed2)
    speed4 = accelerate(speed4)
    print(f"Moving left.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def right():
    global speed1, speed3, button_pressed
    button_pressed = True
    speed1 = accelerate(speed1)
    speed3 = accelerate(speed3)
    print(f"Moving right.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def backward():
    global speed1, speed2, button_pressed
    button_pressed = True
    speed1 = accelerate(speed1)
    speed2 = accelerate(speed2)
    print(f"Moving backward.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def rotate_clockwise():
    global speed1, speed2, speed3, speed4, button_pressed
    button_pressed = True
    speed1 = accelerate(speed1)
    speed4 = accelerate(speed4)
    print(f"Rotating clockwise.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def rotate_counter_clockwise():
    global speed1, speed2, speed3, speed4, button_pressed
    button_pressed = True
    speed2 = accelerate(speed2)
    speed3 = accelerate(speed3)
    print(f"Rotating counter-clockwise.Speed1:{speed1},Speed2:{speed2},Speed3:{speed3},Speed4:{speed4}")
    adjust_motor_speeds()
    start_hover_timer()


def process_command(command_str):
    print(f"Processing command: {command_str}")
    commands = {
        "up": up,
        "down": down,
        "forward": forward,
        "backward": backward,
        "left": left,
        "right": right,
        "rotate_clockwise": rotate_clockwise,
        "rotate_counter_clockwise": rotate_counter_clockwise,
        "stop": stop
    }
    if command_str in commands:
        commands[command_str]()
    else:
        raise HTTPException(status_code=400, detail=f"Invalid command: {command_str}")


# FastAPI setup
app = FastAPI()

# HTML control interface
index_html = """<!DOCTYPE html>
<html>
<head>
    <title>Drone Control</title>
    <style>
        body {
            display: block;
            justify-content: space-between;
            margin: 0;
            font-family: Arial, sans-serif;
        }
        .right-controls {
            display: grid;
            grid-template-columns: 50px 50px 50px;
            grid-template-rows: 50px 50px 50px;
            gap: 30px;
            place-items: center;
            margin-top: -210px;
            margin-left: 850px;
        }
        .left-controls {
            display: grid;
            grid-template-columns: 50px 50px 50px;
            grid-template-rows: 50px 50px 50px;
            gap: 30px;
            place-items: center;
            margin-top: 250px;
            margin-left: 250px;
        }
        .button {
            width: 100px;
            height: 100px;
            font-size: 24px;
            display: flex;
            align-items: center;
            justify-content: center;
            cursor: pointer;
            border: none;
            background-color: #007bff;
            color: white;
            border-radius: 50%;
            transition: background-color 0.3s;
        }
        .button:active {
            background-color: #0056b3;
        }
        .forward-button { grid-column: 2; grid-row: 1; }
        .backward-button { grid-column: 2; grid-row: 3; }
        .left-button { grid-column: 1; grid-row: 2; }
        .right-button { grid-column: 3; grid-row: 2; }
        .up-button { grid-column: 2; grid-row: 1; }
        .down-button { grid-column: 2; grid-row: 3; }
        .rotate-clockwise { grid-column: 3; grid-row: 2; }
        .rotate-counterclockwise { grid-column: 1; grid-row: 2; }
    </style>
</head>
<body>
    <div class="left-controls">
        <button class="button up-button" onmousedown="sendCommand('up', 'start')" onmouseup="sendCommand('up', 'stop')">Up</button>
        <button class="button left-button" onmousedown="sendCommand('left', 'start')" onmouseup="sendCommand('left', 'stop')">↤</button>
        <button class="button right-button" onmousedown="sendCommand('right', 'start')" onmouseup="sendCommand('right', 'stop')">↦</button>
        <button class="button down-button" onmousedown="sendCommand('down', 'start')" onmouseup="sendCommand('down', 'stop')">Down</button>
    </div>
    <div class="right-controls">
        <button class="button forward-button" onmousedown="sendCommand('forward', 'start')" onmouseup="sendCommand('forward', 'stop')">⇧</button>
        <button class="button backward-button" onmousedown="sendCommand('backward', 'start')" onmouseup="sendCommand('backward', 'stop')">⇩</button>
        <button class="button rotate-clockwise" onmousedown="sendCommand('rotate_clockwise', 'start')" onmouseup="sendCommand('rotate_clockwise', 'stop')">↻</button>
        <button class="button rotate-counterclockwise" onmousedown="sendCommand('rotate_counter_clockwise', 'start')" onmouseup="sendCommand('rotate_counter_clockwise', 'stop')">↺</button>
    </div>

    <script>
        let commandInterval; // Stores the interval ID

        function sendCommand(command, action) {
            if (action === 'start') {
                // Start sending the command repeatedly
                commandInterval = setInterval(() => {
                    fetch(`/command/${command}`, { method: 'POST' })
                        .catch(error => console.error("Command error:", error));
                }, 1000); // Send command every 1 second
            } else if (action === 'stop') {
                // Stop sending the command
                clearInterval(commandInterval);
            }
        }
    </script>
</body>
</html>"""


@app.get("/", response_class=HTMLResponse)
async def get_index():
    return index_html


@app.post("/command/{command}")
async def send_command(command: str):
    process_command(command)
    return {"status": "ok"}


