#  import RPi.GPIO as GPIO
#  import time
#  
#  # Pin definitions
#  IN1 = 26
#  IN2 = 25
#  IN3 = 29
#  IN4 = 28
#  ENA = 31
#  ENB = 11
#  
#  # Set up GPIO
#  GPIO.setmode(GPIO.BCM)
#  GPIO.setup(IN1, GPIO.OUT)
#  GPIO.setup(IN2, GPIO.OUT)
#  GPIO.setup(IN3, GPIO.OUT)
#  GPIO.setup(IN4, GPIO.OUT)
#  GPIO.setup(ENA, GPIO.OUT)
#  GPIO.setup(ENB, GPIO.OUT)
#  
#  # Set up PWM on the enable pins
#  pwmA = GPIO.PWM(ENA, 1000) # Frequency: 1kHz
#  pwmB = GPIO.PWM(ENB, 1000) # Frequency: 1kHz
#  
#  # Start PWM with a duty cycle of 0 (stop)
#  pwmA.start(0)
#  pwmB.start(0)
#  
#  def set_speed(pwm, speed):
#      """Set the speed of the motor."""
#      pwm.ChangeDutyCycle(speed)
#  
#  def move_forward():
#      """Move both motors forward."""
#      GPIO.output(IN1, GPIO.HIGH)
#      GPIO.output(IN2, GPIO.LOW)
#      GPIO.output(IN3, GPIO.HIGH)
#      GPIO.output(IN4, GPIO.LOW)
#  
#  def move_backward():
#      """Move both motors backward."""
#      GPIO.output(IN1, GPIO.LOW)
#      GPIO.output(IN2, GPIO.HIGH)
#      GPIO.output(IN3, GPIO.LOW)
#      GPIO.output(IN4, GPIO.HIGH)
#  
#  def stop():
#      """Stop both motors."""
#      GPIO.output(IN1, GPIO.LOW)
#      GPIO.output(IN2, GPIO.LOW)
#      GPIO.output(IN3, GPIO.LOW)
#      GPIO.output(IN4, GPIO.LOW)
#  
#  try:
#      while True:
#          # Move forward
#          set_speed(pwmA, 75)  # 75% speed
#          set_speed(pwmB, 75)  # 75% speed
#          move_forward()
#          time.sleep(2)
#  
#          # Stop
#          stop()
#          time.sleep(1)
#  
#          # Move backward
#          set_speed(pwmA, 75)
#          set_speed(pwmB, 75)
#          move_backward()
#          time.sleep(2)
#  
#          # Stop
#          stop()
#          time.sleep(1)
#  
#  except KeyboardInterrupt:
#      pass
#  
#  # Cleanup
#  pwmA.stop()
#  pwmB.stop()
# GPIO.cleanup()
#

from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit
import RPi.GPIO as GPIO

# Pin definitions
IN1 = 26
IN2 = 19
IN3 = 13
IN4 = 12
ENA = 21
ENB = 20

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up PWM on the enable pins
pwmA = GPIO.PWM(ENA, 1000)  # Frequency: 1kHz
pwmB = GPIO.PWM(ENB, 1000)  # Frequency: 1kHz

# Start PWM with a duty cycle of 0 (stop)
pwmA.start(0)
pwmB.start(0)

def set_speed(pwm, speed):
    """Set the speed of the motor."""
    pwm.ChangeDutyCycle(speed)

def move_forward():
    """Move both motors forward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def move_backward():
    """Move both motors backward."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    """Stop both motors."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# Create Flask app
app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def index():
    return render_template_string('''
 <!DOCTYPE html>
<html>
<head>
    <title>Motor Control</title>
    <script src="https://cdn.socket.io/4.0.0/socket.io.min.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 0;
            padding: 0;
            background-color: #f2f2f2;
        }
        h1 {
            margin-top: 20px;
            color: #333;
        }
        .remote {
            display: inline-block;
            margin-top: 20px;
            border: 2px solid #ccc;
            border-radius: 10px;
            padding: 20px;
            background-color: #fff;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
        }
        .button {
            display: inline-block;
            padding: 15px 30px;
            margin: 10px;
            font-size: 18px;
            background-color: #4CAF50;
            border: 2px solid #4CAF50;
            color: #fff;
            border-radius: 10px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .button:hover {
            background-color: #45a049;
        }
        input[type=range] {
            width: 80%;
            margin: 20px auto;
            display: block;
        }
        #status {
            margin-top: 20px;
            font-weight: bold;
        }
    </style>
    <script>
        var socket = io();

        function sendCommand(direction, speed) {
            socket.emit('move', {direction: direction, speed: speed});
        }

        function forward() {
            let speed = document.getElementById('speed').value;
            sendCommand('forward', speed);
        }

        function backward() {
            let speed = document.getElementById('speed').value;
            sendCommand('backward', speed);
        }

        function stop() {
            sendCommand('stop', 0);
        }

        socket.on('response', function(data) {
            document.getElementById('status').innerText = 'Command sent successfully!';
            setTimeout(function() {
                document.getElementById('status').innerText = '';
            }, 2000);
        });
    </script>
</head>
<body>
    <h1>Motor Control</h1>
    <div class="remote">
        <input type="range" id="speed" min="0" max="100" value="75">
        <br>
        <button class="button" onclick="forward()">▲</button><br>
        <button class="button" onclick="backward()">▼</button><br>
        <button class="button" onclick="stop()">■</button>
    </div>
    <div id="status"></div>
</body>
</html>

    ''')

@socketio.on('move')
def handle_move(data):
    direction = data['direction']
    speed = int(data['speed'])
    set_speed(pwmA, speed)
    set_speed(pwmB, speed)
    if direction == 'forward':
        move_forward()
    elif direction == 'backward':
        move_backward()
    elif direction == 'stop':
        stop()
    emit('response', {'status': 'ok'})

if __name__ == '__main__':
    try:
        socketio.run(app, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        GPIO.cleanup()
        pass
    finally:
        # Cleanup
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
